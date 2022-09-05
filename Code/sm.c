#include "msp430.h"
#include "ports.h"
#include "wheels.h"
#include "sm.h"
#include <string.h>
#include "adc.h"
#include "timers.h"
#include "detectors.h"
#include "utils.h"
#include "pid.h"

extern volatile unsigned int cycle_count;
extern volatile unsigned int stopwatch_milliseconds;
extern volatile unsigned int stopwatch_seconds;
extern volatile unsigned char display_changed;
extern char display_line[4][11];
extern volatile unsigned int wheel_periods;
volatile char state = START;
volatile int stateCounter, driveStateCounter;
volatile char nextState = STRAIGHT;
extern volatile unsigned int Time_Sequence;
extern volatile unsigned int Last_Time_Sequence;
extern volatile unsigned int time_change;
volatile unsigned int delayTime = 5;
extern volatile unsigned int ADC_Left_Detect, ADC_Right_Detect;
extern volatile unsigned int rightSwitchable, leftSwitchable;
extern unsigned int temp;
extern char movingDirection;
char enteringDirection = NOT_MOVING;
extern int leftVals[VALUES_TO_HOLD];
extern int rightVals[VALUES_TO_HOLD];
extern volatile unsigned int calibrationMode;
extern unsigned int LBDetect, LWDetect, RBDetect, RWDetect;
extern PIDController rightFollowController, rightAdjustController;
extern PIDController leftFollowController, leftAdjustController;

extern short l_LessBlack, l_LessGray, l_LessWhite, r_LessBlack, r_LessGray, r_LessWhite, l_GreaterBlack, l_GreaterGray, l_GreaterWhite, r_GreaterBlack, r_GreaterGray, r_GreaterWhite, lessWhiteOr, lessWhiteAnd, greaterWhiteOr, greaterWhiteAnd, lessWhiteOr, lessWhiteAnd, greaterWhiteOr, greaterWhiteAnd, lessGrayOr, lessGrayAnd, greaterGrayOr, greaterGrayAnd, lessGrayOr, lessGrayAnd, greaterGrayOr, greaterGrayAnd, lessBlackOr, lessBlackAnd, greaterBlackOr, greaterBlackAnd, lessBlackOr, lessBlackAnd, greaterBlackOr, greaterBlackAnd;

int speedRight, speedLeft;
unsigned int driveTime;

void LineFollow(char direction)
{

    int rFollowSpeed, lFollowSpeed;
    int rAdjustSpeed = (RIGHT_MIN - LF_TURN_DECREMENT);
    int lAdjustSpeed = (LEFT_MIN - LF_TURN_DECREMENT);
    switch (stateCounter)
    {
    case 0:
        EMITTER_ON;
        strcpy(display_line[0], "          ");
        strcpy(display_line[1], "          ");
        strcpy(display_line[2], "          ");
        strcpy(display_line[3], "          ");
        display_changed = 1;

        if (rightSwitchable && leftSwitchable)
            stateCounter++;
        else
            return;

        break;

    case 1:
        // if (l_LessWhite ^ r_LessWhite)
        // {
        //     stateCounter = 10;
        //     break;
        // }
        // if ((ADC_Left_Detect <= LEFT_GRAY_DETECT && ADC_Right_Detect <= RIGHT_GRAY_DETECT) || (ADC_Left_Detect > LEFT_BLACK_DETECT && ADC_Right_Detect > RIGHT_BLACK_DETECT))
        // {
        //     rFollowSpeed = lFollowSpeed = 3000;
        //     ClearPIDController(&leftFollowController);
        //     ClearPIDController(&leftFollowController);
        // }
        // else
        // {
        //ADC_Right_Detect = 10;
        lFollowSpeed = additionSafe(LEFT_FORWARD_SPEED, LEFT_MAX, 2500, GetOutput(&leftFollowController, RIGHT_BLACK_DETECT, ADC_Right_Detect));  // swapped b/c they are physically swapped
        rFollowSpeed = additionSafe(RIGHT_FORWARD_SPEED, RIGHT_MAX, 2500, GetOutput(&rightFollowController, LEFT_BLACK_DETECT, ADC_Left_Detect)); // swapped b/c they are physically swapped
        
        
        // rFollowSpeed = additionSafe(rFollowSpeed, RIGHT_MAX, 2500, -(lFollowSpeed>>2));
        // lFollowSpeed = additionSafe(lFollowSpeed, LEFT_MAX, 2500, -(orig>>2));
        // }

        if (rFollowSpeed >= 5000 && lFollowSpeed >= 5000)
        {
            rFollowSpeed = 3000;
            lFollowSpeed = 3000;
        }

        // if (lessWhiteAnd)
        // {
        //     rFollowSpeed = -RIGHT_MIN >> 1;
        //     lFollowSpeed = -LEFT_MIN >> 1;
        // }

        HEXtoBCD(rFollowSpeed / 100, 2, 6);
        HEXtoBCD(lFollowSpeed / 100, 2, 0);
        Drive_Path(rFollowSpeed, lFollowSpeed, 0);

        break;

    case 2:
        if (l_LessWhite && r_GreaterWhite)
            stateCounter = 3;
        else if (l_GreaterWhite && r_LessWhite)
            stateCounter = 4;
        else
            stateCounter = 1;

        break;

    case 3: // turn left ()
        if (l_LessWhite)
            Drive_Path(rAdjustSpeed, -lAdjustSpeed, 0);
        else if (greaterWhiteAnd)
            stateCounter = 1;
        else
            stateCounter = 4;

        break;

    case 4:
        if (r_LessWhite)
            Drive_Path(-rAdjustSpeed, lAdjustSpeed, 0);
        else if (greaterWhiteAnd)
            stateCounter = 1;
        else
            stateCounter = 3;

        break;

    case 10:
        if (LockMotorsTime(-1, -1, 1))
            stateCounter = 2;

        break;
    case 5:
        ShutoffMotors();
        stateCounter = 0;
        state = START;
        EMITTER_OFF;
        break;
    }
}

void StateMachine(void)
{
    updateDetectors();

    HEXtoBCD(ADC_Left_Detect, 3, 6);
    HEXtoBCD(ADC_Right_Detect, 3, 0);
    display_changed = 1;

    switch (state)
    {

    case (START):
        ShutoffMotors();
        break;

    case (LINEFOLLOW):
        LineFollow(speedRight);
        break;

    case (DONE):
        break;

    default:
        break;
    }
}
