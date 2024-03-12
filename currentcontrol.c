#include "currentcontrol.h"
#include "utilities.h"
#include "ina219.h"
#include "positioncontrol.h"
/*
Hardware Connection:

B10 - H-Bridge Data line
B7 - H-Bridge PWM line


MODES

PWM: drive with Duty Cycle & Direction
IDLE: do nothing
ITEST:
- controller tracks a +/- 200mA 100Hz square wave reference current.
- reference current toggles between +200 and -200 mA every 25 times through the ISR.
- have a static int counter counting from 0 to 99
at 25, 50, 75 reference current changes sign
when ctr reaches 99, change to IDLE mode.

- PI controller reads i_sensor, compares to square wave reference, calculates new PWM duty cycle and motor direction bit
- reference and actual current data saved in arrays for later plotting

*/

static volatile int MotorPWM;

// static volatile float Kp = 1, Ki = 1, Kd = 1;     // control gains (1,1,1 default)
static volatile float Kp = 0.4, Ki = 0.2, Kd = 0; // works well no load
// set up PID variables
int error = 0;
int eint = 0;
int eprev = 0;
int eder = 0;

float u = 0; // 0-100 float representing %
int unew = 0;

// static int EINT_MAX = 1000; // rule of thumb is Ki*EINT_MAX <= max control effort available from actuator.
// 1 Amp on motor = 1000mA
static int EINT_MAX = 1000;

// returns Kp
float get_current_kp()
{
    return Kp;
}

// returns Ki
float get_current_ki()
{
    return Ki;
}

// set Kp
void set_current_kp(float kp)
{
    Kp = kp;
}

// set Ki
void set_current_ki(float ki)
{
    Ki = ki;
}

// clear integral error
void clear_eint()
{
    eint = 0;
}

void CurrentController_Startup()
{
    // disable interrupts
    __builtin_disable_interrupts(); // step 2: disable interrupts at CPU

    make_ITEST_RefWave(0, 200); // init reference current wave for ITEST

    // Timer3 config for PWM @ 20kHz
    // 48MHz, PS=1, Target = 20kHz
    T3CONbits.TCKPS = 0; // Timer3 prescaler N=0 (1:1)
    PR3 = 2399;          // 48MHz/PS/Target = PRValue = 2400
    TMR3 = 0;            // initial TMR3 count is 0
    T3CONbits.ON = 1;    // turn on Timer3

    // OC1 config
    OC1CONbits.OCM = 0b110;   // PWM mode without fault pin; other OC1CON bits are defaults
    OC1R = 0;                 // initialize before turning OC1 on; afterward it is read-only
    RPB7Rbits.RPB7R = 0b0101; // put OC1 on B7
    OC1CONbits.OCTSEL = 1;    // Timer3 is the clock source for OC1
    OC1CONbits.ON = 1;        // turn on OC1

    // Timer 2 config for 5kHz ISR
    T2CONbits.TCKPS = 0b110;                  // N=6 (1:64) set prescaler
    PR2 = (NU32DIP_SYS_FREQ / 64 / 5000) - 1; // set PR for 5kHz
    TMR2 = 0;                                 // Initial TMR2 count is 0
    T2CONbits.ON = 1;                         // Start Timer2

    // Configure Timer2 Interrupt
    IPC2bits.T2IP = 5; // step 4: interrupt priority
    IPC2bits.T2IS = 0; // step 4: subp is 0, which is the default
    IFS0bits.T2IF = 0; // step 5: clear Timer2 interrupt flag
    IEC0bits.T2IE = 1; // step 6: enable Timer2 interrupt

    // Config B10 to DigOut
    // TRISB.TRISB10 = 0; // sets B10 as DigOut.
    TRISBCLR = 0b10000000000; // sets B10 as DigOut.
    MOTOR_DATALINE = 1;       // by default go "cw"

    // re-enable interrupts
    __builtin_enable_interrupts(); // step 7: CPU interrupts enabled
}

// set motor direction and motor power duty cycle [-100, 100]
void set_motor_power_and_direc(int power)
{
    MOTOR_DATALINE = power > 0 ? 1 : 0; // set direction
    // MOTOR_DATALINE = power > 0 ? 0 : 1; // set direction... realized had negative current
    MotorPWM = abs(power);
    if (power > 100) // cap at 100% duty cycle
        MotorPWM = 100;
    OC1RS = MotorPWM * (PR3 + 1) / 100; // set OC1RS to new duty cycle
}

// configure Timer2 to call ISR @ 5kHz. Controls OC1
void __ISR(_TIMER_2_VECTOR, IPL5SOFT) CurrentController(void)
{
    static int ITEST_ctr = 0; // init counter once (static)

    static float i_val;
    volatile static int ref = 200;

    switch (get_operation_mode())
    {
    case IDLE: // Put H-Bridge into Brake Mode
    {
        OC1RS = 0; // turn off PWM, direction not changed
        break;
    }
    case PWM: // Duty cycle set to user input [-100, 100]
    {
        // OC1RS = MotorPWM * (PR3 + 1) / 100; // set OC1RS to new duty cycle
        break;
    }
    case ITEST: // ITEST mode
    {
        i_val = INA219_read_current(); // read measured current

        // PI Controller
        ref = get_ref_current(ITEST_ctr);
        error = ref - i_val;
        eint = eint + error;
        if (eint > EINT_MAX)
        {
            eint = EINT_MAX;
        }
        else if (eint < -EINT_MAX)
        {
            eint = -EINT_MAX;
        }
        u = Kp * error + Ki * eint;

        set_motor_power_and_direc(u / 10);

        // save the data during test cycle to buffers
        if (is_storing_data())
        {
            set_measured_current(i_val, ITEST_ctr); // save current
        }
        ITEST_ctr++;
        // if just finished a cycle of ITEST
        if (ITEST_ctr == get_ITEST_NUM_SAMPS())
        {
            // stop storing data
            set_storing_data_false();
            // go back to IDLE mode
            // set_operation_mode(IDLE);
            set_operation_mode(0);
            ITEST_ctr = 0; // reset in case we want another
        }
        break;
    }

    // other cases
    case HOLD:
    {
        /*
        the current controller uses the current commanded by the position controller
as the reference for the PI current controller.
        */
        i_val = INA219_read_current(); // read measured current

        // PID Controller
        ref = get_posn_PID_output_ref_current(); // need to use negative of ref current TODO: internalize
        // set_motor_power_and_direc(20);
        current_PI(ref, i_val);
        // set_motor_power_and_direc(20);
        break;
    }
    case TRACK:
    {
        /*
        To the current
controller, the TRACK case is identical to the HOLD case: in both cases, the current
controller attempts to match the current commanded by the position controller
*/
        i_val = INA219_read_current();
        ref = get_posn_PID_output_ref_current(); // output of position PID controller is the ref
        current_PI(ref, i_val);
        break;
    }
    }
    IFS0bits.T2IF = 0; // clear Timer2 Interrupt flag
}

// does an iteration of PI control and sets power
void current_PI(float ref, float i_val)
{
    // integrator anti windup
    if (eint > EINT_MAX)
    {
        eint = EINT_MAX;
    }
    else if (eint < -EINT_MAX)
    {
        eint = -EINT_MAX;
    }

    error = ref - i_val;
    eint += error;

    u = Kp * error + Ki * eint;

    set_motor_power_and_direc(u / 10);
}