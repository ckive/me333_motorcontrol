/*
does position control
*/

#include <xc.h>          // processor SFR definitions
#include <sys/attribs.h> // __ISR macro

#include "NU32DIP.h"

void PositionController_Startup();

void set_desired_ref_angle(int deg);

/// PID Getter, Setter
void set_position_kp(float kp);
void set_position_ki(float ki);
void set_position_kd(float kd);
float get_position_kp();
float get_position_ki();
float get_position_kd();
float get_posn_PID_output_ref_current();