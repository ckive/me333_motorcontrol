/*
This module is used for various bookkeeping tasks. It maintains a variable
holding the operating mode (IDLE, PWM, ITEST, HOLD, or TRACK) and arrays
(buffers) to hold data collected during trajectory tracking (TRACK) or current tracking
(ITEST). The interface utilities.h provides functions to set the operating mode, return
the current operating mode, receive the number N of samples to save into the data buffers
during the next TRACK or ITEST, write data to the buffers if N is not yet reached, and to
send the buffer data to the client when N samples have been collected (TRACK or ITEST
has completed).
*/

#include "NU32DIP.h"
#include "utilities.h"

// static const int ITEST_NUM_SAMPS = 100;
// static const int TRAJ_NUM_SAMPS = 3000;
#define ITEST_NUM_SAMPS 100
#define TRAJ_NUM_SAMPS 3000

volatile int ITEST_RefCurrent[ITEST_NUM_SAMPS];      // reference +/- 200mA current cycle
volatile int ITEST_MeasuredCurrent[ITEST_NUM_SAMPS]; // actual current
volatile int TRAJ_RefPosn[TRAJ_NUM_SAMPS];           // ref posn
volatile int TRAJ_MeasuredPosn[TRAJ_NUM_SAMPS];      // actual posn
static int RefPosnN = 0;                             // number of samples (<= 3000)

static volatile OperationMode operating_mode = IDLE; // default IDLE (0)

static volatile int StoringData = 0; // if this flag = 1, currently storing

char buffer[200]; // everyone uses this (extern)

#define BUF_SIZE 200

void set_refposnN(int N)
{
    RefPosnN = N;
}

int get_refposnN()
{
    return RefPosnN;
}

void set_operation_mode(OperationMode mode)
{
    operating_mode = mode;
}

OperationMode get_operation_mode()
{
    return operating_mode;
}

int is_storing_data()
{
    return StoringData;
}

void set_storing_data_true()
{
    StoringData = 1;
}

void set_storing_data_false()
{
    StoringData = 0;
}

int get_ITEST_NUM_SAMPS()
{
    return ITEST_NUM_SAMPS;
}

int get_TRAJ_NUM_SAMPS()
{
    return TRAJ_NUM_SAMPS;
}

// creates the reference (+)200 to (-)200 mA wave
// center = 0, A = 200
// makes 2 cycles of a reference wave
void make_ITEST_RefWave(int center, int A)
{
    int i = 0;
    for (i = 0; i < ITEST_NUM_SAMPS; ++i)
    {
        if (i < ITEST_NUM_SAMPS / 4)
        {
            ITEST_RefCurrent[i] = center + A;
        }
        else if (i < ITEST_NUM_SAMPS / 2)
        {
            ITEST_RefCurrent[i] = center - A;
        }
        else if (i < 3 * ITEST_NUM_SAMPS / 4)
        {
            ITEST_RefCurrent[i] = center + A;
        }
        else
        {
            ITEST_RefCurrent[i] = center - A;
        }
    }
}

// current controller
void set_measured_current(float i_val, int idx)
{
    ITEST_MeasuredCurrent[idx] = i_val;
}

int get_ref_current(int idx)
{
    return ITEST_RefCurrent[idx];
}

int get_measured_current(int idx)
{
    return ITEST_MeasuredCurrent[idx];
}

// position controller
void set_measured_posn(int posn_deg, int idx)
{
    TRAJ_MeasuredPosn[idx] = posn_deg;
}

int get_ref_posn(int idx)
{
    return TRAJ_RefPosn[idx];
}

void set_ref_posn(int idx, int val)
{
    TRAJ_RefPosn[idx] = val;
}

void send_measured_v_ref_to_client(OperationMode op_mode) // type: 0 ITEST, 1 HOLD, 2 TRACK
{
    switch (op_mode)
    {
    case ITEST:
    {
        // ITEST_NUM_SAMPS = 100
        send_to_client(ITEST_NUM_SAMPS, ITEST_MeasuredCurrent, ITEST_RefCurrent);
        break;
    }
    case HOLD:
    {
        send_to_client(get_refposnN(), TRAJ_MeasuredPosn, TRAJ_RefPosn);
        break;
    }
    case TRACK:
    {
        send_to_client(get_refposnN(), TRAJ_MeasuredPosn, TRAJ_RefPosn);
        break;
    }
    }
}

// sends measured and reference values of whatever type to Client
void send_to_client(int N, volatile int *measured, volatile int *ref)
{
    // tell client N
    sprintf(buffer, "%d\r\n", N);
    NU32DIP_WriteUART1(buffer);
    // send rest of results
    for (int i = 0; i < N; ++i)
    {
        // when first number sent = 1, Python knows we’re done
        sprintf(buffer, "%d %d %d\r\n", N - i, measured[i], ref[i]);
        NU32DIP_WriteUART1(buffer);
    }
}

// // after op mode is changed, this sets store data to true, runs until stop storing, then sends to client
void run_store_send(OperationMode mode)
{
    set_storing_data_true();
    while (is_storing_data())
    {
        ;
    }
    send_measured_v_ref_to_client(mode);
}

// for loading and storing trajectory
void get_and_set_ref_posn()
{
    NU32DIP_ReadUART1(buffer, BUF_SIZE); // read number of data points
    sscanf(buffer, "%d", &RefPosnN);

    for (int i = 0; i < RefPosnN; ++i)
    {
        NU32DIP_ReadUART1(buffer, BUF_SIZE);
        sscanf(buffer, "%d", &TRAJ_RefPosn[i]);
    }
}