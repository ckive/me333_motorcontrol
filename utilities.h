/*
book keeping
*/

#ifndef UTILITIES_H
#define UTILITIES_H

extern char buffer[200];

typedef enum
{
    IDLE,
    PWM,
    ITEST,
    HOLD,
    TRACK
} OperationMode;

void set_operation_mode(OperationMode mode);
OperationMode get_operation_mode();

void make_ITEST_RefWave(int center, int A);
void set_measured_current(float i_val, int idx);
int get_ref_current(int idx);
int get_measured_current(int idx);
int get_ITEST_NUM_SAMPS();

void send_measured_v_ref_to_client(OperationMode op_mode);
void send_to_client(int N, volatile int *measured, volatile int *ref);
void run_store_send(OperationMode mode);

void set_measured_posn(int posn_deg, int idx);
int get_ref_posn(int idx);
int get_TRAJ_NUM_SAMPS();

#endif