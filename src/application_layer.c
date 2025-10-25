// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


#define START_PACKET 0x01
#define DATA_PACKET 0x02
#define END_PACKET 0x03

#define FILE_SIZE_TYPE 0x00
#define FILE_NAME_TYPE 0x01

unsigned char* createControlPacket(int type, const char* filename, long int file_size, int* packet_size);
unsigned char* createDataPacket(const unsigned char* data, int data_size, int* packet_size);
void parseControlPacket(unsigned char* packet, int packet_size, char* filename, long int* file_size);
unsigned char* parseDataPacket(unsigned char* packet, int packet_size, int* data_size);

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer link_layer;
    strcpy(link_layer.serialPort, serialPort);
    if (strcmp(role, "transmitter") == 0){
        link_layer.role =LlTx;
    }
    else{
       link_layer.role =LlRx; 
    }
    link_layer.baudRate=baudRate;
    link_layer.nRetransmissions=nTries;
    link_layer.timeout=timeout;

    if (llopen(link_layer) < 0) {
        printf("Couldn't establish connection\n");
        exit(-1);
    }

    switch (link_layer.role){
        case LlTx:{
            FILE* file = fopen (filename,"rb");
            if (file==NULL){
                printf("File not found\n");
                exit(-1);
            }

            fseek(file, 0L, SEEK_END);
            long int file_size = ftell(file);
            rewind(file);

            int ctrl_packet_size;
            unsigned char* ctrl_start = createControlPacket(START_PACKET,filename,file_size,&ctrl_packet_size);

            if(llwrite(ctrl_start, ctrl_packet_size) == -1){ 
                printf("error in start packet\n");
                free(ctrl_start);
                fclose(file);
                exit(-1);
            }
            free(ctrl_start);

            unsigned char buffer[MAX_PAYLOAD_SIZE];
            size_t bytes_read;

            while ((bytes_read = fread(buffer, 1, MAX_PAYLOAD_SIZE, file)) > 0) {
                int data_packet_size;
                unsigned char* data_packet = createDataPacket(buffer, bytes_read, &data_packet_size);
                
                if (llwrite(data_packet, data_packet_size) == -1) {
                    printf("error in data packet\n");
                    free(data_packet);
                    fclose(file);
                    exit(-1);
                }
                
                free(data_packet);
        }

            unsigned char* ctrl_end = createControlPacket(END_PACKET,filename,file_size,&ctrl_packet_size);

            if(llwrite(ctrl_end, ctrl_packet_size) == -1){ 
                printf("error in end packet\n");
                free(ctrl_end);
                fclose(file);
                exit(-1);
            }
            free(ctrl_end);

            fclose(file);
            break;
        }

        case LlRx:{
            unsigned char packet[MAX_PAYLOAD_SIZE];
            int packet_size;
            FILE* output_file = NULL;
            long int expected_file_size = 0;
            char output_filename[256];

            while ((packet_size = llread(packet)) < 0);

            if (packet[0] != START_PACKET) {
                printf("Not START packet\n");
                break;
            }

            parseControlPacket(packet,packet_size,output_filename,&expected_file_size);

            output_file = fopen(output_filename, "wb");
            if (output_file == NULL) {
                printf("Error creating output file: %s\n", output_filename);
                break;
            }

            long int total_received=0;
            while (total_received < expected_file_size ){
                packet_size=llread(packet);
                if (packet_size < 0) continue;

                if (packet[0] == DATA_PACKET){
                    int data_size;
                    unsigned char* file_data = parseDataPacket(packet, packet_size, &data_size);
                    fwrite(file_data, 1, data_size, output_file);
                    free(file_data);
                    total_received += data_size;
                }

                if (packet[0] == END_PACKET){
                    printf("Received END Packet\n");
                    break;
                }
            }

            fclose(output_file);
            printf("File transfer complete\n");
            break;
        }
        default:
        exit(-1);
        break;
    }

    llclose();
        
}

unsigned char* createControlPacket(int type, const char* filename, long int file_size, int* packet_size) {
    int filename_len = strlen(filename);
    
    *packet_size = 1 + (1 + 1 + 4) + (1 + 1 + filename_len);
    
    
    unsigned char* packet = (unsigned char*)malloc(*packet_size);
    if (!packet) return NULL;
    
    int position = 0;
    
    if (type==1){packet[position++] = START_PACKET;}
    else {packet[position++] = END_PACKET;}  
    
    // File size parameter (TLV)
    packet[position++] = FILE_SIZE_TYPE;  
    packet[position++] = 0x04;            // L = 4 bytes should be enough for this lab
    
    packet[position++] = (file_size >> 24) & 0xFF;  //msb
    packet[position++] = (file_size >> 16) & 0xFF;
    packet[position++] = (file_size >> 8) & 0xFF;
    packet[position++] = file_size & 0xFF;        //lsb  
    
    // File name parameter (TLV)
    packet[position++] = FILE_NAME_TYPE;  
    packet[position++] = filename_len;    // L = length of filename
    memcpy(&packet[position], filename, filename_len);
    position += filename_len;
    
    return packet;
}

unsigned char* createDataPacket(const unsigned char* data, int data_size, int* packet_size) {
    *packet_size = 1 + 2 + data_size;  
    
    unsigned char* packet = (unsigned char*)malloc(*packet_size);
    
    packet[0] = DATA_PACKET;           
    packet[1] = (data_size >> 8) & 0xFF; 
    packet[2] = data_size & 0xFF;        
    memcpy(&packet[3], data, data_size); 
    
    return packet;
}

void parseControlPacket(unsigned char* packet, int packet_size, char* filename, long int* file_size) {
    int pos = 1; // Skip C
    
    if (packet[pos++] == FILE_SIZE_TYPE) {
        int size_len = packet[pos++]; 
        *file_size = 0;
        for (int i = 0; i < size_len; i++) {
            *file_size = (*file_size << 8) | packet[pos++];
        }
    }
    
    
    if (packet[pos++] == FILE_NAME_TYPE) {
        int name_len = packet[pos++]; 
        memcpy(filename, &packet[pos], name_len);
        filename[name_len] = '\0'; 
    }
}

unsigned char* parseDataPacket(unsigned char* packet, int packet_size, int* data_size) {
    *data_size = (packet[1] << 8) | packet[2];
    
    unsigned char* file_data = (unsigned char*)malloc(*data_size);
    memcpy(file_data, &packet[3], *data_size); 
    
    return file_data;
}
