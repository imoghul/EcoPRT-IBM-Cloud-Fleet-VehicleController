#include "iot.h"
#include "msp430.h"
#include "utils.h"
#include <string.h>
#include "wheels.h"
#include "utils.h"
#include "serial.h"
#include "ports.h"
#include "sm.h"

char iot_setup_state = BOOT_UP;
extern volatile char USB0_Char_Tx[];
extern unsigned volatile int pb0_buffered;
extern volatile char USB0_Char_Rx_Process[];
extern volatile char receievedFromPC;
char SSID[SSID_LEN + 1];
char IP[IP_LEN + 1];
extern volatile unsigned char display_changed;
extern char display_line[4][11];
char dotFound;
int midIndex;
extern volatile int stateCounter;
extern volatile char pingFlag;


int Init_IOT(void) {
    int isTransmitting = UCA0IE & UCTXIE;

    switch(iot_setup_state) {
        case (BOOT_UP):
            waitForReady();
            break;

        case CIPMUX_Tx:
            SendIOTCommand(ALLOW_MULTIPLE_CONNECTIONS, CIPMUX_Rx);
            break;

        case CIPMUX_Rx:
            if(isTransmitting) break; // wait for the Tx to completely transmit

            if(pb0_buffered) { // wait for pb to finish buffering
                iot_setup_state = CIPSERVER_Tx;
                clearProcessBuff_0();
            }

            break;

        case CIPSERVER_Tx:
            SendIOTCommand(START_SERVER, CIPSERVER_Rx);
            break;

        case CIPSERVER_Rx:
            if(isTransmitting) break;

            if(pb0_buffered) {
                iot_setup_state = GET_SSID_Tx;
                clearProcessBuff_0();
            }

            break;

        case GET_SSID_Tx:
            SendIOTCommand(SSID_COMMAND, GET_SSID_Rx);
            break;

        case GET_SSID_Rx:
            if(isTransmitting) break;

            getSSID();

            break;

        case GET_IP_Tx:
            SendIOTCommand(IP_COMMAND, GET_IP_Rx);
            break;

        case GET_IP_Rx:
            if(isTransmitting) break;

            getIP();
            displayNetworkInfo();

            break;

        default:
            if(pingFlag) {
                pingFlag = 0;
                SendIOTCommand(PING_COMMAND, IOT_SETUP_FINISHED);
            }

            return 1;
            break;
    }

    return 0;
}


void waitForReady(void) {
    if(pb0_buffered) {
        if(strcmp((char*)USB0_Char_Rx_Process, BOOT_RESPONSE) == 0) iot_setup_state = CIPMUX_Tx;

        clearProcessBuff_0();
    }
}

void SendIOTCommand(char* command, char nextState) {
    strcpy((char*)USB0_Char_Tx, command);
    USCI_A0_transmit();
    iot_setup_state = nextState;
}

void getSSID(void) {
    if(pb0_buffered) {
        if(subStringPos((char*)USB0_Char_Rx_Process, SSID_RESPONSE)) {
            int i;

            for(i = 0; i <= SSID_LEN && USB0_Char_Rx_Process[i + SSID_RESPONSE_LEN + 1] != '\"'; ++i) SSID[i] = USB0_Char_Rx_Process[i + SSID_RESPONSE_LEN + 1];

            SSID[i + SSID_RESPONSE_LEN + 2] = 0; // set the end of the SSID to null
            SSID[SSID_LEN] = 0; //  set end of the array to null

            iot_setup_state = GET_IP_Tx;
        } else iot_setup_state = GET_SSID_Tx;

        clearProcessBuff_0();
    }
}

void getIP(void) {
    if(pb0_buffered) {
        if(subStringPos((char*)USB0_Char_Rx_Process, IP_RESPONSE)) {
            int i;

            for(i = 0; i <= IP_LEN && USB0_Char_Rx_Process[i + IP_RESPONSE_LEN + 1] != '"'; ++i) {
                IP[i] = USB0_Char_Rx_Process[i + IP_RESPONSE_LEN + 1];

                if(IP[i] == '.') {
                    if(++dotFound == 2) midIndex = i;
                }
            }

            IP[i + IP_RESPONSE_LEN + 2] = 0;
            IP[IP_LEN] = 0;
            IP[midIndex] = 0;

            iot_setup_state = IOT_SETUP_FINISHED;
        } else iot_setup_state = GET_IP_Tx;

        clearProcessBuff_0();
    }
}

void displayNetworkInfo(void) {
    centerStringToDisplay(0, SSID);
    displayIP(1);
    display_changed = 1;
}

void displayIP(int pos) {
    strcpy(display_line[pos], "          ");
    strcpy(display_line[pos + 1], "          ");
    centerStringToDisplay(pos, IP);
    centerStringToDisplay(pos + 1, IP + midIndex + 1);
}


void IOTBufferCommands(void) {
    if(pb0_buffered) {
        if(subStringPos((char*)USB0_Char_Rx_Process, DISCONNECTED_RESPONSE))
            iot_setup_state = CIPSERVER_Tx;

        clearProcessBuff_0();
    }

}