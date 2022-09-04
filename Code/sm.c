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



void LineFollow(char direction) {
   
    int rFollowSpeed, lFollowSpeed;

   
    

    switch(stateCounter) {
        case 0:
            EMITTER_ON;
            display_changed = 1;

            if(rightSwitchable && leftSwitchable)stateCounter++;
            else return;

            break;

        case 1:
          if(ADC_Left_Detect<=LEFT_WHITE_DETECT && ADC_Right_Detect<=RIGHT_WHITE_DETECT) {
            stateCounter = 2;
            break;
          }
            if(ADC_Left_Detect<=LEFT_GRAY_DETECT && ADC_Right_Detect<=RIGHT_GRAY_DETECT) {
              rFollowSpeed = lFollowSpeed = 3000;
              ClearPIDController(&leftFollowController);
              ClearPIDController(&leftFollowController);
            }
            else{
               rFollowSpeed = additionSafe(RIGHT_FORWARD_SPEED, RIGHT_MAX, 2800, GetOutput(&leftFollowController, LEFT_BLACK_DETECT, ADC_Left_Detect)); // swapped b/c they are physically swapped
   
               lFollowSpeed = additionSafe(LEFT_FORWARD_SPEED, LEFT_MAX, 2800, GetOutput(&rightFollowController, RIGHT_BLACK_DETECT, ADC_Right_Detect));// swapped b/c they are physically swapped
            }
          
            if(rFollowSpeed >= 10000 && lFollowSpeed>=10000)rFollowSpeed = lFollowSpeed = 3000;;
          
            Drive_Path(rFollowSpeed, lFollowSpeed, 0);
            
            break;
        
        case 2:
            rFollowSpeed = lFollowSpeed = -2800;
            Drive_Path(rFollowSpeed, lFollowSpeed, 0);  
            if(ADC_Left_Detect>LEFT_WHITE_DETECT && ADC_Right_Detect>RIGHT_WHITE_DETECT) stateCounter = 1;
            
            ClearPIDController(&leftFollowController);
            ClearPIDController(&leftFollowController);
            break;
        case 3:
            ShutoffMotors();
            stateCounter = 0 ;
            state = START;
            EMITTER_OFF;
            break;
    }

}

void StateMachine(void) {
    updateDetectors();
    
    HEXtoBCD(ADC_Left_Detect,3,6);
            HEXtoBCD(ADC_Right_Detect,3,0);
            display_changed = 1;

    switch(state) {

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
