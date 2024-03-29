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
float get_ref_current(int idx);
float get_measured_current(int idx);
int get_ITEST_NUM_SAMPS();

void send_measured_v_ref_to_client(OperationMode op_mode);
void send_to_client(int N, volatile float *measured, volatile float *ref);
void run_store_send(OperationMode mode);

void set_measured_posn(int posn_deg, int idx);
float get_ref_posn(int idx);
int get_TRAJ_NUM_SAMPS();
void set_ref_posn(int idx, int val);

int get_refposnN();
void set_refposnN(int N);
void get_and_set_ref_posn();

#endif