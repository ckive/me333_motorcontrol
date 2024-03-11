/*
implements current control loop.

*/
#include <xc.h>          // processor SFR definitions
#include <sys/attribs.h> // __ISR macro

#include "NU32DIP.h"

void CurrentController_Startup();
void set_motor_power_and_direc(int power);

int is_storing_data();
void set_storing_data_true();
void set_storing_data_false();
float get_current_kp();
float get_current_ki();
void set_current_kp(float kp);
void set_current_ki(float ki);
void clear_eint();
void current_PI(float ref, float i_val);

#define MOTOR_DATALINE LATBbits.LATB10 // controls direction of motor by feeding H-Bridge 0/1