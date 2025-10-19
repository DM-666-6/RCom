// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

// Frame constants
#define FLAG 0x7E
#define A_TRANSMITTER 0x03
#define A_RECEIVER 0x01
#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B 

// Global variables
static volatile int alarmEnabled = FALSE;
static volatile int alarmCount = 0;
static volatile int STOP = FALSE;
static LinkLayerRole current_role;
static int current_timeout;          
static int current_retransmissions;

// State machine for frame reception
typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP_STATE
} State;

static void alarmHandler(int signal);
static int setup_alarm_handler();
static int llopen_transmitter();
static int llopen_receiver();
static int llclose_transmitter();
static int llclose_receiver();

// Alarm handler function
static void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
}

/**
 * Configure alarm signal handling
 */
static int setup_alarm_handler()
{
    if (signal(SIGALRM, alarmHandler) == SIG_ERR)
    {
        perror("signal");
        return -1;
    }
    return 0;
}

/**
 * Handles connection establishment for transmitter role
 */
static int llopen_transmitter()
{
    // Build SET frame: FLAG | A | C | BCC | FLAG
    unsigned char set_frame[5] = {
        FLAG,
        A_TRANSMITTER,  // Address field for transmitter
        C_SET,          // Control field for SET
        A_TRANSMITTER ^ C_SET,  // BCC
        FLAG
    };

    int attempts = 0;
    STOP = FALSE;
    alarmCount = 0;

    if (setup_alarm_handler() < 0)
    {
        return -1;
    }

    while (attempts < current_retransmissions && !STOP)
    {
        // Send SET frame
        int bytes_written = writeBytesSerialPort(set_frame, 5);
        if (bytes_written < 5)
        {
            return -1;
        }

        // Set alarm for timeout
        alarm(current_timeout);
        alarmEnabled = TRUE;

        // Wait for UA response
        State state = START;
        unsigned char byte;
        int valid_ua = FALSE;

        while (alarmEnabled && !STOP && !valid_ua)
        {
            int res = readByteSerialPort(&byte);
            if (res < 1)
                continue;

            switch (state)
            {
            case START:
                if (byte == FLAG)
                    state = FLAG_RCV;
                break;

            case FLAG_RCV:
                if (byte == FLAG)
                    state = FLAG_RCV;
                else if (byte == A_TRANSMITTER)
                    state = A_RCV;
                else
                    state = START;
                break;

            case A_RCV:
                if (byte == FLAG)
                    state = FLAG_RCV;
                else if (byte == C_UA)
                    state = C_RCV;
                else
                    state = START;
                break;

            case C_RCV:
                if (byte == FLAG)
                    state = FLAG_RCV;
                else if (byte == (A_TRANSMITTER ^ C_UA))
                    state = BCC_OK;
                else
                    state = START;
                break;

            case BCC_OK:
                if (byte == FLAG)
                {
                    state = STOP_STATE;
                    valid_ua = TRUE;
                    STOP = TRUE;
                    alarm(0);
                    alarmEnabled = FALSE;
                }
                else
                    state = START;
                break;

            default:
                state = START;
            }
        }

        if (valid_ua)
        {
            return 0;
        }

        if (alarmEnabled) {
            alarm(0);
            alarmEnabled = FALSE;
        }

        attempts++;
    }

    if (attempts >= current_retransmissions)
    {
        return -1;
    }

    return 0;
}

/**
 * Handles connection establishment for receiver role
 */
static int llopen_receiver()
{
    State state = START;
    unsigned char byte;
    int valid_set = FALSE;

    if (setup_alarm_handler() < 0)
    {
        return -1;
    }
    
    alarm(current_timeout);
    alarmEnabled = TRUE;

    while (alarmEnabled && !valid_set)
    {
        int res = readByteSerialPort(&byte);
        if (res < 1)
            continue;

        switch (state)
        {
        case START:
            if (byte == FLAG)
                state = FLAG_RCV;
            break;

        case FLAG_RCV:
            if (byte == FLAG)
                state = FLAG_RCV;
            else if (byte == A_TRANSMITTER)
                state = A_RCV;
            else
                state = START;
            break;

        case A_RCV:
            if (byte == FLAG)
                state = FLAG_RCV;
            else if (byte == C_SET)
                state = C_RCV;
            else
                state = START;
            break;

        case C_RCV:
            if (byte == FLAG)
                state = FLAG_RCV;
            else if (byte == (A_TRANSMITTER ^ C_SET))
                state = BCC_OK;
            else
                state = START;
            break;

        case BCC_OK:
            if (byte == FLAG)
            {
                state = STOP_STATE;
                valid_set = TRUE;
                alarm(0);
                alarmEnabled = FALSE;
            }
            else
                state = START;
            break;

        default:
            state = START;
        }
    }

    if (!valid_set)
    {
        return -1;
    }

    // Send UA response
    unsigned char ua_frame[5] = {
        FLAG,
        A_TRANSMITTER,
        C_UA,
        A_TRANSMITTER ^ C_UA,
        FLAG
    };

    int bytes_written = writeBytesSerialPort(ua_frame, 5);
    if (bytes_written < 5)
    {
        return -1;
    }

    return 0;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{

    current_role = connectionParameters.role;
    current_timeout = connectionParameters.timeout;          
    current_retransmissions = connectionParameters.nRetransmissions;

    // Open and configure serial port
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0)
    {
        return -1;
    }

    int result = 0;
    
    if (current_role == LlTx)
    {
        result = llopen_transmitter();
    }
    else // LlRx
    {
        result = llopen_receiver();
    }

    if (result < 0)
    {
        closeSerialPort();
        return -1;
    }

    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    int result = 0;
    
    if (current_role == LlTx) {
        result = llclose_transmitter();
    } else {
        result = llclose_receiver();
    }

    if (closeSerialPort() < 0) {
        return -1;
    }

    return result;
}

static int llclose_transmitter()
{
    unsigned char disc_frame[5] = {
        FLAG,
        A_TRANSMITTER,      
        C_DISC,             
        A_TRANSMITTER ^ C_DISC,  
        FLAG
    };

    int attempts = 0;
    STOP = FALSE;

    while (attempts < current_retransmissions && !STOP) {
        int bytes_written = writeBytesSerialPort(disc_frame, 5);
        if (bytes_written < 5) {
            return -1;
        }

        alarm(current_timeout);
        alarmEnabled = TRUE;

        State state = START;
        unsigned char byte;
        int valid_disc_response = FALSE;

        while (alarmEnabled && !STOP && !valid_disc_response) {
            int res = readByteSerialPort(&byte);
            if (res < 1) continue;

            switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == FLAG) state = FLAG_RCV;
                    else if (byte == A_TRANSMITTER) state = A_RCV;
                    else state = START;
                    break;
                case A_RCV:
                    if (byte == FLAG) state = FLAG_RCV;
                    else if (byte == C_DISC) state = C_RCV;
                    else state = START;
                    break;
                case C_RCV:
                    if (byte == FLAG) state = FLAG_RCV;
                    else if (byte == (A_TRANSMITTER ^ C_DISC)) state = BCC_OK; 
                    else state = START;
                    break;
                case BCC_OK:
                    if (byte == FLAG) {
                        state = STOP_STATE;
                        valid_disc_response = TRUE;
                        STOP = TRUE;
                        alarm(0);
                        alarmEnabled = FALSE;
                    } else state = START;
                    break;
                default:
                    state = START;
            }
        }

        if (valid_disc_response) {
            unsigned char ua_frame[5] = {
                FLAG,
                A_RECEIVER,      
                C_UA,               
                A_RECEIVER ^ C_UA, 
                FLAG
            };
            
            int ua_written = writeBytesSerialPort(ua_frame, 5);
            if (ua_written < 5) {
                return -1;
            }
            return 0;  
        }

        if (alarmEnabled) {
            alarm(0);
            alarmEnabled = FALSE;
        }

        attempts++;
    }

    return -1;
}

static int llclose_receiver()
{
    State state = START;
    unsigned char byte;
    int valid_disc = FALSE;

    alarm(current_timeout * 2);  
    alarmEnabled = TRUE;

    while (alarmEnabled && !valid_disc) {
        int res = readByteSerialPort(&byte);
        if (res < 1) continue;

        switch (state) {
            case START:
                if (byte == FLAG) state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if (byte == FLAG) state = FLAG_RCV;
                else if (byte == A_TRANSMITTER) state = A_RCV;
                else state = START;
                break;
            case A_RCV:
                if (byte == FLAG) state = FLAG_RCV;
                else if (byte == C_DISC) state = C_RCV;
                else state = START;
                break;
            case C_RCV:
                if (byte == FLAG) state = FLAG_RCV;
                else if (byte == (A_TRANSMITTER ^ C_DISC)) state = BCC_OK; 
                else state = START;
                break;
            case BCC_OK:
                if (byte == FLAG) {
                    state = STOP_STATE;
                    valid_disc = TRUE;
                    alarm(0);
                    alarmEnabled = FALSE;
                } else state = START;
                break;
            default:
                state = START;
        }
    }

    if (!valid_disc) {
        return -1;  
    }

    
    unsigned char disc_response[5] = {
        FLAG,
        A_TRANSMITTER,        
        C_DISC,            
        A_TRANSMITTER ^ C_DISC, 
        FLAG
    };

    int bytes_written = writeBytesSerialPort(disc_response, 5);
    if (bytes_written < 5) {
        return -1;
    }

    alarm(current_timeout);
    alarmEnabled = TRUE;
    
    state = START;
    int valid_ua = FALSE;

    while (alarmEnabled && !valid_ua) {
        int res = readByteSerialPort(&byte);
        if (res < 1) continue;

        switch (state) {
            case START:
                if (byte == FLAG) state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if (byte == FLAG) state = FLAG_RCV;
                else if (byte == A_RECEIVER) state = A_RCV;
                else state = START;
                break;
            case A_RCV:
                if (byte == FLAG) state = FLAG_RCV;
                else if (byte == C_UA) state = C_RCV;
                else state = START;
                break;
            case C_RCV:
                if (byte == FLAG) state = FLAG_RCV;
                else if (byte == (A_RECEIVER ^ C_UA)) state = BCC_OK; 
                else state = START;
                break;
            case BCC_OK:
                if (byte == FLAG) {
                    state = STOP_STATE;
                    valid_ua = TRUE;
                } else state = START;
                break;
            default:
                state = START;
        }
    }

    return valid_ua ? 0 : -1;
}
