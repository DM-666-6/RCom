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

// Configuration constants
#define MAX_FRAME_SIZE 256
#define MAX_RETRIES 10
#define DEBUG_LEVEL 1

// Debug logging
#if DEBUG_LEVEL >= 1
#define LOG_DEBUG(fmt, ...) printf("DEBUG: " fmt, ##__VA_ARGS__)
#else
#define LOG_DEBUG(fmt, ...)
#endif

#define LOG_INFO(fmt, ...) printf("INFO: " fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) printf("ERROR: " fmt, ##__VA_ARGS__)

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
    STOP_STATE,
    ERROR_STATE
} State;

/**
 * Builds a frame with proper structure
 */
static void build_frame(unsigned char* frame, unsigned char address, 
                       unsigned char control, int* frame_size)
{
    frame[0] = FLAG;
    frame[1] = address;
    frame[2] = control;
    frame[3] = address ^ control;  // BCC
    frame[4] = FLAG;
    *frame_size = 5;
    
    LOG_DEBUG("Built frame: FLAG(0x%02X) | A(0x%02X) | C(0x%02X) | BCC(0x%02X) | FLAG(0x%02X)\n",
              frame[0], frame[1], frame[2], frame[3], frame[4]);
}

/**
 * Alarm handler function
 */
static void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
    LOG_DEBUG("Alarm #%d received\n", alarmCount);
}

/**
 * Configure alarm signal handling
 */
static int setup_alarm_handler()
{
    // Use the simple signal() function instead of sigaction for portability
    if (signal(SIGALRM, alarmHandler) == SIG_ERR)
    {
        perror("signal");
        return -1;
    }
    return 0;
}

/**
 * Cleanup resources
 */
static void cleanup_resources()
{
    alarm(0);  // Cancel any pending alarm
    alarmEnabled = FALSE;
    closeSerialPort();
    STOP = TRUE;
}

/**
 * Handles connection establishment for transmitter role
 */
static int llopen_transmitter(LinkLayer connectionParameters)
{
    unsigned char set_frame[5];
    int frame_size;
    build_frame(set_frame, A_TRANSMITTER, C_SET, &frame_size);

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
        int bytes_written = writeBytesSerialPort(set_frame, frame_size);
        if (bytes_written < frame_size)
        {
            LOG_ERROR("Error writing SET frame - wrote %d/%d bytes\n", bytes_written, frame_size);
            cleanup_resources();
            return -1;
        }
        LOG_INFO("SET frame sent (attempt %d)\n", attempts + 1);

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

            LOG_DEBUG("Received byte: 0x%02X, state: %d\n", byte, state);

            switch (state)
            {
            case START:
                if (byte == FLAG)
                    state = FLAG_RCV;
                break;

            case FLAG_RCV:
                if (byte == FLAG)
                    state = FLAG_RCV;  // Stay in FLAG_RCV for consecutive FLAGs
                else if (byte == A_RECEIVER)  // UA should come from receiver
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
                else if (byte == (A_RECEIVER ^ C_UA))  // BCC check
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
                    alarm(0);  // Cancel alarm
                    alarmEnabled = FALSE;
                    LOG_INFO("Valid UA frame received\n");
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
            return 0;  // Success
        }

        if (alarmEnabled) {
            alarm(0);  // Cancel alarm if still enabled
            alarmEnabled = FALSE;
        }

        attempts++;
        if (attempts < connectionParameters.nRetransmissions)
        {
            LOG_INFO("Timeout - retransmitting SET frame\n");
        }
    }

    if (attempts >= connectionParameters.nRetransmissions)
    {
        LOG_ERROR("Failure: Maximum retransmission attempts (%d) reached\n", 
                 connectionParameters.nRetransmissions);
        cleanup_resources();
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

    LOG_INFO("Waiting for SET frame...\n");

    // Set up alarm for SET frame reception timeout
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

        LOG_DEBUG("Received byte: 0x%02X, state: %d\n", byte, state);

        switch (state)
        {
        case START:
            if (byte == FLAG)
                state = FLAG_RCV;
            break;

        case FLAG_RCV:
            if (byte == FLAG)
                state = FLAG_RCV;  // Stay in FLAG_RCV for consecutive FLAGs
            else if (byte == A_TRANSMITTER)  // SET should come from transmitter
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
            else if (byte == (A_TRANSMITTER ^ C_SET))  // BCC check
                state = BCC_OK;
            else
                state = START;
            break;

        case BCC_OK:
            if (byte == FLAG)
            {
                state = STOP_STATE;
                valid_set = TRUE;
                alarm(0);  // Cancel alarm
                alarmEnabled = FALSE;
                LOG_INFO("Valid SET frame received\n");
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
        LOG_ERROR("Timeout waiting for SET frame\n");
        cleanup_resources();
        return -1;
    }

    // Send UA response
    unsigned char ua_frame[5];
    int frame_size;
    build_frame(ua_frame, A_RECEIVER, C_UA, &frame_size);

    int bytes_written = writeBytesSerialPort(ua_frame, frame_size);
    if (bytes_written < frame_size)
    {
        LOG_ERROR("Error writing UA frame - wrote %d/%d bytes\n", bytes_written, frame_size);
        cleanup_resources();
        return -1;
    }

    LOG_INFO("UA frame sent - connection established\n");
    return 0;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    // Validate connection parameters
    if (connectionParameters.nRetransmissions <= 0 || connectionParameters.timeout <= 0)
    {
        LOG_ERROR("Invalid connection parameters: nRetransmissions=%d, timeout=%d\n",
                 connectionParameters.nRetransmissions, connectionParameters.timeout);
        return -1;
    }

    // Open and configure serial port
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0)
    {
        LOG_ERROR("Failed to open serial port %s\n", connectionParameters.serialPort);
        return -1;
    }

    LOG_INFO("Serial port %s opened and configured (baudrate: %d)\n", 
             connectionParameters.serialPort, connectionParameters.baudRate);

    int result = 0;
    
    if (connectionParameters.role == LlTx)
    {
        LOG_INFO("Starting as TRANSMITTER\n");
        result = llopen_transmitter(connectionParameters);
    }
    else // LlRx
    {
        LOG_INFO("Starting as RECEIVER\n");
        result = llopen_receiver(connectionParameters);
    }

    if (result < 0)
    {
        LOG_ERROR("Failed to establish connection\n");
        cleanup_resources();
        return -1;
    }

    LOG_INFO("Connection established successfully\n");
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
   return 0;
}