#include "msp430.h"
#include "ports.h"
#include "wheels.h"
#include "adc.h"
#include "timers.h"
#include "detectors.h"
#include "pid.h"
#include "sm.h"
#include <string.h>

extern volatile unsigned int cycle_count;
extern volatile unsigned int stopwatch_milliseconds;
extern volatile unsigned int stopwatch_seconds;
extern volatile unsigned char display_changed;
extern char display_line[4][11];
volatile unsigned int wheel_periods;
extern volatile unsigned int Time_Sequence;
extern volatile unsigned int Last_Time_Sequence;
extern volatile unsigned int time_change;
extern volatile unsigned int ADC_Left_Detect, ADC_Right_Detect;
volatile unsigned int rightSwitchable = 1, leftSwitchable = 1;
unsigned int temp;
extern char movingDirection;
extern int leftVals[VALUES_TO_HOLD];
extern int rightVals[VALUES_TO_HOLD];


PIDController rightFollowController = {
    .kP = 1,// /16
    .kD = 10000,// /8
    //.kI = 0,
    .error = 0,
    .lastError = 0
    //.lastIntegral = 0
};
PIDController leftFollowController = {
    .kP = 1,// /16
    .kD = 10000,// /8
    //.kI = 0,
    .error = 0,
    .lastError = 0
    //.lastIntegral = 0
};


void ShutoffMotors(void) {
    ShutoffRight();
    ShutoffLeft();
}

void ShutoffRight(void) {
    RIGHT_FORWARD_SPEED = RIGHT_REVERSE_SPEED = WHEEL_OFF;
    rightSwitchable = 0;

    TB1CCTL2 &= ~CCIFG;
    TB1CCR2 = TB1R + TB1CCR2_INTERVAL;
    TB1CCTL2 |= CCIE;
}

void ShutoffLeft(void) {
    LEFT_FORWARD_SPEED = LEFT_REVERSE_SPEED = WHEEL_OFF;
    leftSwitchable = 0;

    TB1CCTL1 &= ~CCIFG;
    TB1CCR1 = TB1R + TB1CCR1_INTERVAL;
    TB1CCTL1 |= CCIE;
}

void MotorSafety(void) {
    if ((RIGHT_FORWARD_SPEED != 0 && RIGHT_REVERSE_SPEED != 0) || (LEFT_FORWARD_SPEED != 0 && LEFT_REVERSE_SPEED != 0)) {
        ShutoffMotors();
        //P1OUT |= RED_LED;
    } else {
        //P1OUT &= ~RED_LED;
    }
}

int RunRightMotor(int val) {
    if(RIGHT_REVERSE_SPEED > 0 && val > 0 || RIGHT_FORWARD_SPEED > 0 && val < 0) {
        ShutoffRight();
    }

    if (val > 0) {
        RIGHT_REVERSE_SPEED = WHEEL_OFF;

        if(rightSwitchable) RIGHT_FORWARD_SPEED = val;

        return 1;//P6IN & R_FORWARD;
    } else if (val == 0) {
        ShutoffRight();
        return rightSwitchable;
    } else {
        RIGHT_FORWARD_SPEED = WHEEL_OFF;

        if(rightSwitchable) RIGHT_REVERSE_SPEED = -val;

        return 1;//P6IN & R_REVERSE;
    }
}

int RunLeftMotor( int val) {
    if(LEFT_REVERSE_SPEED > 0 && val > 0 || LEFT_FORWARD_SPEED > 0 && val < 0) {
        ShutoffLeft();
    }

    if (val > 0) {
        LEFT_REVERSE_SPEED = WHEEL_OFF;

        if(leftSwitchable) LEFT_FORWARD_SPEED = val;

        return 1;//P6IN & L_FORWARD;
    } else if (val == 0) {
        ShutoffLeft();
        return leftSwitchable;
    } else {
        LEFT_FORWARD_SPEED = WHEEL_OFF;

        if(leftSwitchable) LEFT_REVERSE_SPEED = -val;

        return 1;//P6IN & L_REVERSE_2355;
    }
}


int LockMotors(int polR, int polL) {
    return (Drive_Path(polR > 0 ? STRAIGHT_RIGHT : -STRAIGHT_RIGHT, polL > 0 ? STRAIGHT_LEFT : -STRAIGHT_LEFT, LOCK_TIME));
}

int LockMotorsTime(int polR, int polL, int duration) {
    return (Drive_Path(polR > 0 ? STRAIGHT_RIGHT : -STRAIGHT_RIGHT, polL > 0 ? STRAIGHT_LEFT : -STRAIGHT_LEFT, duration));
}

int Drive_Path(int speedR, int speedL, unsigned int duration) {
    RunRightMotor(speedR);
    RunLeftMotor(speedL);

}

