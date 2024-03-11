// Main!
#include "NU32DIP.h" // constants, functions for startup and UART
#include "encoder.h"
#include "utilities.h"
#include "i2c_master_noint.h"
#include "currentcontrol.h"
#include "positioncontrol.h"
#include "ina219.h"
// include other header files here

/*
Recap
UART1 between PIC and Client
UARTx between PIC and PICO
*/

#define BUF_SIZE 200
int main()
{
    char buffer[BUF_SIZE];
    NU32DIP_Startup();  // cache on, min flash wait, interrupts on, LED/button init, UART1 init
    NU32DIP_YELLOW = 1; // turn off the LEDs. Yellow is ERR LED
    NU32DIP_GREEN = 1;

    __builtin_disable_interrupts();
    // in future, initialize modules or peripherals here

    UART2_Startup();             // init UART2
    INA219_Startup();            // init current sensor (it inits i2c using SDA1, SCL1)
    CurrentController_Startup(); // init Timer2,3 OC1, DigOut for Motor
    // PositionController_Startup(); // init Timer4

    __builtin_enable_interrupts();

    while (1)
    {
        NU32DIP_ReadUART1(buffer, BUF_SIZE); // we expect the next character to be a menu command
        NU32DIP_YELLOW = 1;                  // clear the error LED
        switch (buffer[0])
        {
        case 'b': // read current sensor (mA)
        {
            // char m[50];
            float cur_current = INA219_read_current();
            sprintf(buffer, "%f\r\n", cur_current);
            NU32DIP_WriteUART1(buffer);
            break;
        }
        case 'c': // encoder count
        {
            // send "a" from PIC to PICO for encoder data
            WriteUART2("a");
            while (!get_encoder_flag())
            {
            }
            set_encoder_flag(0);

            int p = get_encoder_count();
            sprintf(buffer, "%d\r\n", p);
            NU32DIP_WriteUART1(buffer); // send encoder count to client
            break;
        }
        case 'd': // encoder deg
        {
            // send "a" from PIC to PICO for encoder data
            WriteUART2("a");
            while (!get_encoder_flag())
            {
            }
            set_encoder_flag(0);

            int deg = get_encoder_deg();
            sprintf(buffer, "%d\r\n", deg);
            NU32DIP_WriteUART1(buffer); // send encoder count to client
            break;
        }
        case 'e': // reset encoder
        {
            // send "b" from PIC to PICO to reset encoder count to 0
            WriteUART2("b");
            break;
        }
        case 'f': // set mode to PWM and set value to user provided
        {
            // set_operation_mode(PWM);
            set_operation_mode(1);
            int power = 0;
            NU32DIP_ReadUART1(buffer, BUF_SIZE); // read the power range
            sscanf(buffer, "%d", &power);
            set_motor_power_and_direc(power);
            break;
        }
        case 'g': // set current gains
        {
            float kp = 0.0;
            float ki = 0.0;
            NU32DIP_ReadUART1(buffer, BUF_SIZE); // read the current gain set
            sscanf(buffer, "%f", &kp);
            NU32DIP_ReadUART1(buffer, BUF_SIZE); // read the current gain set
            sscanf(buffer, "%f", &ki);
            __builtin_disable_interrupts(); // keep ISR disabled as briefly as possible
            set_current_kp(kp);             // copy local variables to globals used by ISR
            set_current_ki(ki);
            clear_eint();                  // cancel out any historic integral error
            __builtin_enable_interrupts(); // only 2 simple C commands while ISRs disabled
            break;
        }
        case 'h': // get current gains
        {
            float kp = get_current_kp();
            float ki = get_current_ki();

            sprintf(buffer, "%f;%f\r\n", kp, ki);
            NU32DIP_WriteUART1(buffer); // send encoder count to client

            break;
        }
        // case 'i': // set position gains
        // {
        //     float kp = 0.0;
        //     float ki = 0.0;
        //     float kd = 0.0;
        //     NU32DIP_ReadUART1(buffer, BUF_SIZE); // read the current gain set
        //     sscanf(buffer, "%f", &kp);
        //     NU32DIP_ReadUART1(buffer, BUF_SIZE); // read the current gain set
        //     sscanf(buffer, "%f", &ki);
        //     NU32DIP_ReadUART1(buffer, BUF_SIZE); // read the current gain set
        //     sscanf(buffer, "%f", &kd);

        //     // __builtin_disable_interrupts(); // keep ISR disabled as briefly as possible
        //     // set_position_kp(kp);
        //     // set_position_ki(ki);
        //     // set_position_kd(kd);
        //     // clear_eint();                  // cancel out any historic integral error
        //     // __builtin_enable_interrupts(); // only 2 simple C commands while ISRs disabled
        //     break;
        // }
        // case 'j': // get position gains
        // {
        //     float kp = get_position_kp();
        //     float ki = get_position_ki();
        //     float kd = get_position_kd();

        //     sprintf(buffer, "%f;%f;%f\r\n", kp, ki, kd);
        //     NU32DIP_WriteUART1(buffer); // send encoder count to client

        //     break;
        // }
        case 'p': // unpower motor (switches to IDLE mode)
        {
            set_operation_mode(IDLE);
            set_motor_power_and_direc(0);
            break;
        }
        case 'r': // get mode
        {

            OperationMode curmode = get_operation_mode();
            // TODO maybe send string rather than int
            sprintf(buffer, "%d\r\n", curmode);
            NU32DIP_WriteUART1(buffer);
            break;
        }
        case 'k': // test current gains (and set to ITEST mode)
        {
            // ITEST on triggers current control ISR
            // set_operation_mode(ITEST);
            set_operation_mode(2);
            set_storing_data_true();
            while (is_storing_data())
            {
                ;
            }

            // V1:
            // get measured and ref arrays and plotpts
            int ITEST_NUM_SAMPS = get_ITEST_NUM_SAMPS();

            // have data to plot, send to client
            for (int i = 0; i < ITEST_NUM_SAMPS; ++i)
            {
                // when first number sent = 1, Python knows we’re done
                sprintf(buffer, "%d %d %d\r\n", ITEST_NUM_SAMPS - i, get_measured_current(i), get_ref_current(i));
                NU32DIP_WriteUART1(buffer);
            }

            // V2:
            // send_measured_v_ref_to_client(get_operation_mode()); // leverages known op_mode to send
            break;
        }
        // case 'l': // set to HOLD mode
        // {
        //     int desired_angle;
        //     NU32DIP_ReadUART1(buffer, BUF_SIZE); // read the current gain set
        //     sscanf(buffer, "%d", &desired_angle);
        //     set_desired_ref_angle(desired_angle); // set desired angle in position controller
        //     // set_operation_mode(HOLD);
        //     set_operation_mode(3);
        //     while (is_storing_data()) // storing HOLD samples, BLOCK
        //     {
        //         ;
        //     }

        //     // Send to client
        //     send_measured_v_ref_to_client(get_operation_mode());
        //     break;
        // }
        // case 'o': // set to TRACK mode
        // {
        //     /*

        //     */
        //     set_operation_mode(TRACK);
        //     run_store_send(TRACK);
        //     break;
        // }
        case 'q':
        {
            // handle q for quit. Later you may want to return to IDLE mode here.
            break;
        }
        default:
        {
            NU32DIP_YELLOW = 0; // turn on LED2 to indicate an error
            break;
        }
        }
    }
    return 0;
}