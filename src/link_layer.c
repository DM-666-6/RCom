// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

// Frame constants
#define FLAG 0x7E
#define A_TRANSMITTER 0x03
#define A_RECEIVER 0x01
#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B 
#define C_RR0 0xAA
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55
#define C_I0 0x00           
#define C_I1 0x80           
#define ESCAPE 0x7D 
#define Escape_1 0x5E
#define Escape_2 0x5d

// Global variables
static volatile int alarmEnabled = FALSE;
static volatile int alarmCount = 0;
static volatile int STOP = FALSE;
static LinkLayerRole current_role;
static int current_timeout;          
static int current_retransmissions;
static bool ns = FALSE;  // False for sending 0, otherwise 1
static volatile bool ack_received = FALSE;
static volatile int reject_received = FALSE;

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
static unsigned char calculate_bcc2(const unsigned char *data, int data_size);
static int send_information_frame(const unsigned char *data, int data_size);
static void process_supervision_frame(unsigned char address, unsigned char control_field);


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

    STOP = FALSE;
    alarmCount = 0;

    while (alarmCount < current_retransmissions && !STOP)
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
    }

    if (alarmCount >= current_retransmissions)
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

    // Receiver waits indefinitely for SET frame (no timeout)
    while (!valid_set)
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
            }
            else
                state = START;
            break;

        default:
            state = START;
        }
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
         if (setup_alarm_handler() < 0)
        {
            closeSerialPort();
            return -1;
        }

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
    if (current_role != LlTx) {
        return -1; 
    }
    
    if (buf == NULL || bufSize <= 0) {
        return -1;
    }
    
    alarmCount = 0;
    STOP = FALSE;
    
    while (alarmCount < current_retransmissions && !STOP) {
        int bytes_sent = send_information_frame(buf, bufSize);
        if (bytes_sent <= 0) {
            return -1;
        }
        
        State state = START;
        unsigned char byte;
        unsigned char address = 0, control = 0;
        
        ack_received = FALSE;
        reject_received = FALSE;
        
        alarm(current_timeout);
        alarmEnabled = TRUE;
        
        while (alarmEnabled && !ack_received && !reject_received) {
            int res = readByteSerialPort(&byte);
            if (res < 1) continue;
            
            switch (state) {
                case START:
                    if (byte == FLAG) {
                        state = FLAG_RCV;
                        address = 0;
                        control = 0;
                    }
                    break;
                    
                case FLAG_RCV:
                    if (byte == FLAG) {
                    } else if (byte == A_TRANSMITTER) {
                        address = byte;
                        state = A_RCV;
                    } else {
                        state = START;
                    }
                    break;
                    
                case A_RCV:
                    if (byte == FLAG) {
                        state = FLAG_RCV;
                    } else if (byte==C_REJ0 || byte==C_REJ1 || byte==C_RR0 || byte==C_RR1) {
                        control = byte;
                        state = C_RCV;
                    }else{
                        state=START;
                    }
                    break;
                    
                case C_RCV:
                    if (byte == FLAG) {
                        state = FLAG_RCV;
                    } else if (byte == (address ^ control)) {
                        state = BCC_OK;
                    } else {
                        state = START;
                    }
                    break;
                    
                case BCC_OK:
                    if (byte == FLAG) {
                        process_supervision_frame(address, control);
                        state = STOP_STATE;
                    } else {
                        state = START;
                    }
                    break;
                    
                default:
                    state = START;
            }
        }
        
        if (alarmEnabled) {
            alarm(0);
            alarmEnabled = FALSE;
        }
        
        if (ack_received) {
            return bufSize;
        } 
    }
    
    if (alarmCount >= current_retransmissions) {
        return -1; 
    }
    
    return bufSize;
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

    alarmCount = 0;
    STOP = FALSE;

    while (alarmCount < current_retransmissions && !STOP) {
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
                    else if (byte == A_RECEIVER) state = A_RCV;
                    else state = START;
                    break;
                case A_RCV:
                    if (byte == FLAG) state = FLAG_RCV;
                    else if (byte == C_DISC) state = C_RCV;
                    else state = START;
                    break;
                case C_RCV:
                    if (byte == FLAG) state = FLAG_RCV;
                    else if (byte == (A_RECEIVER ^ C_DISC)) state = BCC_OK; 
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
    }

    return -1;
}

static int llclose_receiver()
{
    State state = START;
    unsigned char byte;
    int valid_disc = FALSE;

    while (!valid_disc) {
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
                } else state = START;
                break;
            default:
                state = START;
        }
    }

    unsigned char disc_response[5] = {
        FLAG,
        A_RECEIVER,        
        C_DISC,            
        A_RECEIVER ^ C_DISC, 
        FLAG
    };

    int bytes_written = writeBytesSerialPort(disc_response, 5);
    if (bytes_written < 5) {
        return -1;
    }

    state = START;
    int valid_ua = FALSE;

    while (!valid_ua) {
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

    return 0;
}

static unsigned char calculate_bcc2(const unsigned char *data, int data_size) {
    unsigned char bcc2 = 0;
    
    for (int i = 0; i < data_size; i++) {
        bcc2 ^= data[i];
    }
    
    return bcc2;
}

static int byte_stuffing(const unsigned char *input, int input_size, 
                         unsigned char *output) {
    int output_size = 0;
    
    for (int i = 0; i < input_size; i++) {
        if (input[i] == FLAG) {
            output[output_size++] = ESCAPE;
            output[output_size++] = Escape_1;  
        } else if (input[i] == ESCAPE) {
            output[output_size++] = ESCAPE;
            output[output_size++] = Escape_2;  
        } else {
            output[output_size++] = input[i];
        }
    }
    
    return output_size;
}

static int send_information_frame(const unsigned char *data, int data_size) {
    unsigned char control_field = (ns) ? C_I1 : C_I0;
    
    unsigned char bcc1 = A_TRANSMITTER ^ control_field;
    
    unsigned char bcc2 = calculate_bcc2(data, data_size);
 
    int unstuffed_data_size = 3 + data_size + 1;
    unsigned char unstuffed_data[unstuffed_data_size];
    
    unstuffed_data[0] = A_TRANSMITTER;
    unstuffed_data[1] = control_field;
    unstuffed_data[2] = bcc1;
    
    memcpy(&unstuffed_data[3], data, data_size);
    
    unstuffed_data[3 + data_size] = bcc2;
    
    unsigned char stuffed_data[unstuffed_data_size * 2]; 
    int stuffed_data_size = byte_stuffing(unstuffed_data, unstuffed_data_size, stuffed_data);
    
    unsigned char final_frame[stuffed_data_size + 2];
    final_frame[0] = FLAG;
    memcpy(&final_frame[1], stuffed_data, stuffed_data_size);
    final_frame[stuffed_data_size + 1] = FLAG;
    
    int bytes_sent = writeBytesSerialPort(final_frame, stuffed_data_size + 2);
    if (bytes_sent < 0) {
        return -1;
    }
    return bytes_sent;
}

static void process_supervision_frame(unsigned char address, unsigned char control_field) {
    if (address != A_TRANSMITTER) {
        return;
    }
    
    bool expected_nr = (ns) ? FALSE : TRUE; 
    
    switch (control_field) {
        case C_RR0:  
            if (expected_nr == FALSE) { 
                ack_received = TRUE;
                ns = !ns; 
            }
            break;
        case C_RR1:  
            if (expected_nr == TRUE) {
                ack_received = TRUE;
                ns = !ns; 
            }
            break;
        case C_REJ0: 
            if (expected_nr == FALSE) {
                reject_received = TRUE;
            }
            break;
        case C_REJ1: 
            if (expected_nr == TRUE) {
                reject_received = TRUE;
            }
            break;
    }
}


