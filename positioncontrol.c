
#include "positioncontrol.h"
#include "utilities.h"
#include "ina219.h"
#include "encoder.h"
#include "currentcontrol.h"
/*

*/

static volatile float Kp = 0.05, Ki = 0, Kd = 50; // control gains (1,1,1 default)
volatile int desired_ref_angle;                   // angle user inputs in l

volatile int TRAJ_ctr = 0;
volatile float posn_PID_output_ref_current; // output of the position controller (current [mA])
volatile int cur_deg;

float PID_OUTPUT_CONTROL_U[3000];

float get_PID_OUTPUT_CONTROL_U(int idx)
{
    return PID_OUTPUT_CONTROL_U[idx];
}

float get_posn_PID_output_ref_current()
{
    return posn_PID_output_ref_current;
}

// set up PID variables
int error_posn = 0;
int eint_posn = 0;
int eprev_posn = 0;
int eder_posn = 0;

float u_posn = 0; // control

void set_desired_ref_angle(float deg)
{
    desired_ref_angle = deg;
}

void set_position_kp(float kp)
{
    Kp = kp;
}
void set_position_ki(float ki)
{
    Ki = ki;
}
void set_position_kd(float kd)
{
    Kd = kd;
}

float get_position_kp()
{
    return Kp;
}
float get_position_ki()
{
    return Ki;
}
float get_position_kd()
{
    return Kd;
}

// Timer4 for 200Hz ISR
void PositionController_Startup()
{
    __builtin_disable_interrupts(); // step 2: disable interrupts at CPU
    // 48MHz, PS=1, Target = 200Hz
    // 48MHz/PS/Target = PRValue
    T4CONbits.TCKPS = 0b110;                 // N=6 (1:64) set prescaler
    PR4 = (NU32DIP_SYS_FREQ / 64 / 200) - 1; // PR4 = 3749
    TMR4 = 0;
    T4CONbits.ON = 1; // start Timer4

    // Configure Timer4 Interrupt
    IPC4bits.T4IP = 5; // step 4: interrupt priority
    IPC4bits.T4IS = 0; // step 4: subp is 0, which is the default
    IFS0bits.T4IF = 0; // step 5: clear Timer4 interrupt flag
    IEC0bits.T4IE = 1; // step 6: enable Timer4 interrupt

    __builtin_enable_interrupts(); // step 7: CPU interrupts enabled
}

// does an iteration of PID control and set power
// void position_PID(int mode) // mode is either HOLD (0) or TRACK (1)
void position_PID()
{

    cur_deg = get_encoder_deg(); // get cur encoder degrees
                                 // PID control

    u_posn = Kp * error_posn + Ki * eint_posn + Kd * eder_posn; // a degree err value TO current to give to current controller as reference

    // store posn value
    set_measured_posn(cur_deg, TRAJ_ctr);

    posn_PID_output_ref_current = u_posn; // how to visualize this?

    PID_OUTPUT_CONTROL_U[TRAJ_ctr] = u_posn;

    // setup next iteration of PID
    eder_posn = error_posn - eprev_posn;
    eint_posn += error_posn;
    eprev_posn = error_posn;
    ++TRAJ_ctr;

    // posn_PID_output_ref_current = u_posn; // set ref current based on u control

    // if (is_storing_data())
    // {
    //     set_measured_posn(cur_deg, TRAJ_ctr); // store current position (deg)
    // }
}

// configure Timer4 to call ISR @ 200Hz. Runs Position Controller
void __ISR(_TIMER_4_VECTOR, IPL5SOFT) PositionController(void)
{

    switch (get_operation_mode())
    {
    case HOLD:
    {
        position_PID(); // inf until user changes mode.
        TRAJ_ctr = 0;

        // ++TRAJ_ctr;
        // if (TRAJ_ctr == get_TRAJ_NUM_SAMPS()) // just stop recording and continue to hold
        // {
        //     set_storing_data_false();
        //     TRAJ_ctr = 0; // reset
        // }

        break;
    }
    case TRACK:
    {
        /*
        tries to follow the trajectory array with PID.
        Collects data for plotting.
        If reaches N samples, switch to HOLD mode with last angle as HOLDing reference.
        Send data back to client for plotting

        */
        desired_ref_angle = get_ref_posn(TRAJ_ctr); // set reference
        position_PID();

        // stop storing data, set last deg as HOLDing angle, go to HOLD mode
        // if (TRAJ_ctr == get_TRAJ_NUM_SAMPS())
        if (TRAJ_ctr == get_refposnN())
        {
            set_storing_data_false();

            posn_PID_output_ref_current = get_ref_posn(TRAJ_ctr - 1); // set this as the holding value
            set_operation_mode(HOLD);
            TRAJ_ctr = get_refposnN(); // go back one to stay at end...? @SM
        }
        break;
    }
    }
    IFS0bits.T4IF = 0; // clear Timer2 Interrupt flag
}