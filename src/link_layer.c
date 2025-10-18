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

// Global variables
static volatile int alarmEnabled = FALSE;
static volatile int alarmCount = 0;
static volatile int STOP = FALSE;

// State machine for frame reception
typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP_STATE
} State;

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
static int llopen_transmitter(LinkLayer connectionParameters)
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

    while (attempts < connectionParameters.nRetransmissions && !STOP)
    {
        // Send SET frame
        int bytes_written = writeBytesSerialPort(set_frame, 5);
        if (bytes_written < 5)
        {
            return -1;
        }

        // Set alarm for timeout
        alarm(connectionParameters.timeout);
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
                else if (byte == A_RECEIVER)
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
                else if (byte == (A_RECEIVER ^ C_UA))
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

    if (attempts >= connectionParameters.nRetransmissions)
    {
        return -1;
    }

    return 0;
}

/**
 * Handles connection establishment for receiver role
 */
static int llopen_receiver(LinkLayer connectionParameters)
{
    State state = START;
    unsigned char byte;
    int valid_set = FALSE;

    if (setup_alarm_handler() < 0)
    {
        return -1;
    }
    
    alarm(connectionParameters.timeout);
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
        A_RECEIVER,
        C_UA,
        A_RECEIVER ^ C_UA,
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
    // Open and configure serial port
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0)
    {
        return -1;
    }

    int result = 0;
    
    if (connectionParameters.role == LlTx)
    {
        result = llopen_transmitter(connectionParameters);
    }
    else // LlRx
    {
        result = llopen_receiver(connectionParameters);
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
    if (closeSerialPort() < 0)
    {
        return -1;
    }
    
    return 0;
}