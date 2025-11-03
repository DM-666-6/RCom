

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



typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP_STATE
} State;




typedef struct {
    unsigned char address;
    unsigned char control;
} FrameHeader;



static void alarmHandler(int signal);
static int setup_alarm_handler();

static int llopen_transmitter();
static int llopen_receiver();
static int llclose_transmitter();
static int llclose_receiver();

static unsigned char calculate_bcc2(const unsigned char *data, int data_size);
static int send_information_frame(const unsigned char *data, int data_size);
static void process_supervision_frame(unsigned char address, unsigned char control_field);
static int byte_destuffing(const unsigned char *input, int input_size, unsigned char *output);


//helper funct ----------------------------------------------------------------------------------------------------

static int send_supervision_frame(unsigned char address, unsigned char control)
{
    unsigned char frame[5] = {
        FLAG,
        address,
        control,
        address ^ control,
        FLAG
    };
    return writeBytesSerialPort(frame, 5);
}


static int receive_supervision_frame(unsigned char expected_A, unsigned char expected_C, FrameHeader *frame)
{
    State state = START;
    unsigned char byte, A = 0, C = 0;

    while (1) {
        int res = readByteSerialPort(&byte);
        if (res < 1) continue;

        switch (state) {
            case START:
                if (byte == FLAG) state = FLAG_RCV;
                break;

            case FLAG_RCV:
                if (byte == FLAG) state = FLAG_RCV;
                else { A = byte; state = A_RCV; }
                break;

            case A_RCV:
                if (byte == FLAG) state = FLAG_RCV;
                else { C = byte; state = C_RCV; }
                break;

            case C_RCV:
                if (byte == FLAG) state = FLAG_RCV;
                else if (byte == (A ^ C)) state = BCC_OK;
                else state = START;
                break;

            case BCC_OK:
                if (byte == FLAG) {
                    if ((expected_A == 0 || A == expected_A) &&
                        (expected_C == 0 || C == expected_C)) {
                        if (frame) {
                            frame->address = A;
                            frame->control = C;
                        }
                        return 0; // frame válida
                    } else {
                        return 1; // frame inesperada
                    }
                } else state = START;
                break;

            default:
                state = START;
        }
    }
    return -1;
}


static int wait_for_supervision(unsigned char expected_A, unsigned char expected_C, int timeout)
{
    FrameHeader frame;
    alarm(timeout);
    alarmEnabled = TRUE;

    while (alarmEnabled) {
        int result = receive_supervision_frame(expected_A, expected_C, &frame);
        if (result == 0) {
            alarm(0);
            alarmEnabled = FALSE;
            return 0;
        }
    }

    alarmEnabled = FALSE;
    return -1; // timeout
}




////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    current_role = connectionParameters.role;
    current_timeout = connectionParameters.timeout;
    current_retransmissions = connectionParameters.nRetransmissions;

    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0)
        return -1;

    int result = 0;
    if (current_role == LlTx) {
        if (setup_alarm_handler() < 0) {
            closeSerialPort();
            return -1;
        }
        result = llopen_transmitter();
    } else {
        result = llopen_receiver();
    }

    if (result < 0) {
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
    if (current_role != LlTx || buf == NULL || bufSize <= 0)
        return -1;

    alarmCount = 0;
    STOP = FALSE;

    while (alarmCount < current_retransmissions && !STOP) {
        int bytes_sent = send_information_frame(buf, bufSize);
        if (bytes_sent <= 0)
            return -1;

        ack_received = FALSE;
        reject_received = FALSE;
        alarm(current_timeout);
        alarmEnabled = TRUE;

        unsigned char byte;
        State state = START;
        unsigned char address = 0, control = 0;

        while (alarmEnabled && !ack_received && !reject_received) {
            int res = readByteSerialPort(&byte);
            if (res < 1) continue;

            switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;

                case FLAG_RCV:
                    if (byte == A_TRANSMITTER) { address = byte; state = A_RCV; }
                    else if (byte != FLAG) state = START;
                    break;

                case A_RCV:
                    if (byte == FLAG) state = FLAG_RCV;
                    else if (byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1) {
                        control = byte; state = C_RCV;
                    } else state = START;
                    break;

                case C_RCV:
                    if (byte == (address ^ control)) state = BCC_OK;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;

                case BCC_OK:
                    if (byte == FLAG) {
                        process_supervision_frame(address, control);
                        state = STOP_STATE;
                    } else state = START;
                    break;

                default: state = START;
            }
        }

        if (alarmEnabled) {
            alarm(0);
            alarmEnabled = FALSE;
        }

        if (ack_received)
            return bufSize;
    }

    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    if (current_role != LlRx || packet == NULL)
        return -1;

    static bool expected_ns = FALSE;
    State state = START;
    unsigned char byte;
    unsigned char address = 0, control = 0;
    unsigned char stuffed_data[(MAX_PAYLOAD_SIZE+4)*2];
    int stuffed_index = 0;
    unsigned char destuffed_data[MAX_PAYLOAD_SIZE+4];
    int data_size = 0;

    while (1) {
        int res = readByteSerialPort(&byte);
        if (res < 1) continue;

        switch (state) {
            case START:
                if (byte == FLAG) state = FLAG_RCV;
                break;

            case FLAG_RCV:
                if (byte == A_TRANSMITTER) { address = byte; state = A_RCV; }
                else if (byte != FLAG) state = START;
                break;

            case A_RCV:
                if (byte == C_I0 || byte == C_I1) { control = byte; state = C_RCV; }
                else if (byte == FLAG) state = FLAG_RCV;
                else state = START;
                break;

            case C_RCV:
                if (byte == (address ^ control)) state = BCC_OK;
                else if (byte == FLAG) state = FLAG_RCV;
                else state = START;
                break;

            case BCC_OK:
                if (byte == FLAG) {
                    int destuffed_size = byte_destuffing(stuffed_data, stuffed_index, destuffed_data);
                    if (destuffed_size >= 1) {
                        unsigned char received_bcc2 = destuffed_data[destuffed_size - 1];
                        unsigned char calculated_bcc2 = calculate_bcc2(destuffed_data, destuffed_size - 1);

                        if (received_bcc2 == calculated_bcc2) {
                            bool received_ns = (control == C_I1);
                            if (received_ns == expected_ns) {
                                data_size = destuffed_size - 1;
                                memcpy(packet, destuffed_data, data_size);
                                expected_ns = !expected_ns;
                                unsigned char rr_control = expected_ns ? C_RR1 : C_RR0;
                                send_supervision_frame(A_TRANSMITTER, rr_control);
                                return data_size;
                            } else {
                                unsigned char rr_control = expected_ns ? C_RR1 : C_RR0;
                                send_supervision_frame(A_TRANSMITTER, rr_control);
                            }
                        } else {
                            unsigned char rej_control = expected_ns ? C_REJ1 : C_REJ0;
                            send_supervision_frame(A_TRANSMITTER, rej_control);
                        }
                    }
                    state = START;
                } else {
                    stuffed_data[stuffed_index++] = byte;
                }
                break;

            default:
                state = START;
        }
    }
    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    int result = 0;

    if (current_role == LlTx)
        result = llclose_transmitter();
    else
        result = llclose_receiver();

    if (closeSerialPort() < 0)
        return -1;

    return result;
}





static int llopen_transmitter()
{
    printf("Sending SET frame\n");

    for (alarmCount = 0; alarmCount < current_retransmissions; alarmCount++) {
        send_supervision_frame(A_TRANSMITTER, C_SET);

        if (wait_for_supervision(A_TRANSMITTER, C_UA, current_timeout) == 0) {
            printf("UA received — connection established\n");
            return 0;
        }

        printf("Timeout waiting for UA, retrying...\n");
    }

    printf("Failed to establish connection after %d retries\n", current_retransmissions);
    return -1;
}

static int llopen_receiver()
{
    FrameHeader frame;
    printf("Waiting for SET frame from transmitter...\n");

    while (1) {
        if (receive_supervision_frame(A_TRANSMITTER, C_SET, &frame) == 0) {
            printf("Valid SET received, sending UA\n");
            send_supervision_frame(A_TRANSMITTER, C_UA);
            return 0;
        }
    }
}

static int llclose_transmitter()
{
    printf("Initiating DISC procedure (transmitter)\n");

    for (alarmCount = 0; alarmCount < current_retransmissions; alarmCount++) {
        send_supervision_frame(A_TRANSMITTER, C_DISC);
        printf("Sent DISC, waiting for DISC from receiver...\n");

        if (wait_for_supervision(A_RECEIVER, C_DISC, current_timeout) == 0) {
            printf("DISC received from receiver, sending UA...\n");
            send_supervision_frame(A_RECEIVER, C_UA);
            return 0;
        }

        printf("Timeout waiting for receiver DISC, retrying...\n");
    }

    printf("Failed to close connection after %d retries\n", current_retransmissions);
    return -1;
}

static int llclose_receiver()
{
    FrameHeader frame;

    printf("Waiting for DISC from transmitter...\n");
    if (receive_supervision_frame(A_TRANSMITTER, C_DISC, &frame) != 0)
        return -1;

    printf("DISC received, sending DISC back\n");
    send_supervision_frame(A_RECEIVER, C_DISC);

    printf("Waiting for UA from transmitter...\n");
    if (receive_supervision_frame(A_RECEIVER, C_UA, &frame) != 0)
        return -1;

    printf("UA received — connection closed cleanly\n");
    return 0;
}



static unsigned char calculate_bcc2(const unsigned char *data, int data_size)
{
    unsigned char bcc2 = 0;
    for (int i = 0; i < data_size; i++)
        bcc2 ^= data[i];
    return bcc2;
}

static int byte_destuffing(const unsigned char *input, int input_size, unsigned char *output)
{
    int output_size = 0;
    bool escape_next = FALSE;

    for (int i = 0; i < input_size; i++) {
        if (escape_next) {
            if (input[i] == Escape_1) output[output_size++] = FLAG;
            else if (input[i] == Escape_2) output[output_size++] = ESCAPE;
            else return -1;
            escape_next = FALSE;
        } else if (input[i] == ESCAPE) {
            escape_next = TRUE;
        } else {
            output[output_size++] = input[i];
        }
    }

    if (escape_next)
        return -1;

    return output_size;
}

static void process_supervision_frame(unsigned char address, unsigned char control_field)
{
    if (address != A_TRANSMITTER)
        return;

    bool expected_nr = (ns) ? FALSE : TRUE;

    switch (control_field) {
        case C_RR0:
            if (!expected_nr) { ack_received = TRUE; ns = !ns; }
            break;
        case C_RR1:
            if (expected_nr) { ack_received = TRUE; ns = !ns; }
            break;
        case C_REJ0:
            if (!expected_nr) reject_received = TRUE;
            break;
        case C_REJ1:
            if (expected_nr) reject_received = TRUE;
            break;
    }
}

static int send_information_frame(const unsigned char *data, int data_size)
{
    unsigned char control_field = (ns) ? C_I1 : C_I0;
    unsigned char bcc1 = A_TRANSMITTER ^ control_field;
    unsigned char bcc2 = calculate_bcc2(data, data_size);

    int unstuffed_size = 3 + data_size + 1;
    unsigned char unstuffed[unstuffed_size];
    unstuffed[0] = A_TRANSMITTER;
    unstuffed[1] = control_field;
    unstuffed[2] = bcc1;
    memcpy(&unstuffed[3], data, data_size);
    unstuffed[3 + data_size] = bcc2;

    unsigned char stuffed[unstuffed_size * 2];
    int stuffed_size = 0;

    for (int i = 0; i < unstuffed_size; i++) {
        if (unstuffed[i] == FLAG) {
            stuffed[stuffed_size++] = ESCAPE;
            stuffed[stuffed_size++] = Escape_1;
        } else if (unstuffed[i] == ESCAPE) {
            stuffed[stuffed_size++] = ESCAPE;
            stuffed[stuffed_size++] = Escape_2;
        } else {
            stuffed[stuffed_size++] = unstuffed[i];
        }
    }

    unsigned char final_frame[stuffed_size + 2];
    final_frame[0] = FLAG;
    memcpy(&final_frame[1], stuffed, stuffed_size);
    final_frame[stuffed_size + 1] = FLAG;

    return writeBytesSerialPort(final_frame, stuffed_size + 2);
}

static void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
}

static int setup_alarm_handler()
{
    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1) {
        perror("sigaction");
        exit(1);
    }
    return 0;
}
