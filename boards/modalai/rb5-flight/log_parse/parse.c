#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <string.h>

#define FILE_HEADER_SIZE 16
#define MESSAGE_HEADER_SIZE 3
#define MAX_MESSAGE_SIZE 65536

static uint8_t message_data[MAX_MESSAGE_SIZE];
static uint8_t invalid_message_data[MAX_MESSAGE_SIZE];

static uint32_t message_type_counters[26];

static bool in_definitions_section = true;

bool validate_message_header(uint16_t size, uint8_t type) {

    if ((type != 'A') && (type != 'B') &&
        (type != 'C') && (type != 'D') &&
        (type != 'F') && (type != 'I') &&
        (type != 'L') && (type != 'M') &&
        (type != 'O') && (type != 'P') &&
        (type != 'Q') && (type != 'R') &&
        (type != 'S')) return false;

    if ((type == 'A') || (type == 'L')) in_definitions_section = false;

    if (in_definitions_section) {
        if ((type != 'B') && (type != 'F') && (type != 'I') &&
            (type != 'M') && (type != 'P') && (type != 'Q')) return false;
    } else {
        if ((type != 'A') && (type != 'R') && (type != 'D') &&
            (type != 'L') && (type != 'C') && (type != 'S') &&
            (type != 'O') && (type != 'I') && (type != 'M') &&
            (type != 'P') && (type != 'Q')) return false;
    }

    switch (type) {
    case 'A':
        if (size < 6) return false;
        break;
    case 'B':
        if (size != 40) return false;
        break;
    case 'C':
        if (size < 14) return false;
        break;
    case 'D':
        if (size < 4) return false;
        break;
    case 'F':
        if (size < 6) return false;
        break;
    case 'I':
        if (size < 6) return false;
        break;
    case 'L':
        if (size < 10) return false;
        break;
    case 'M':
        if (size < 6) return false;
        break;
    case 'O':
        if (size != 2) return false;
        break;
    case 'P':
        if (size < 6) return false;
        break;
    case 'Q':
        if (size < 6) return false;
        break;
    case 'R':
        if (size != 2) return false;
        break;
    case 'S':
        if (size != 8) return false;
        break;
    }

    return true;
}

int main(int argc, char *argv[]) {
    if (argc > 2) {
        int file_size = 0;
        int bytes_remaining = 0;
        char *file_name = argv[1];
        char *output_file_name = argv[2];

        printf("parse starting on %s\n", file_name);

        struct stat st;

        if (stat(file_name, &st) == 0) {
            file_size = st.st_size;
            printf("File size %d\n", file_size);
            if (file_size < FILE_HEADER_SIZE) {
                fprintf(stderr, "File too short\n");
                return -1;
            }
            bytes_remaining = file_size;
        } else {
            fprintf(stderr, "Couldn't stat file\n");
            return -1;
        }

        int ulg_fd = open(file_name, O_RDONLY);
        if (ulg_fd == -1) {
            fprintf(stderr, "Couldn't open input file %s\n", file_name);
            return -1;
        }
        int out_fd = open(output_file_name, O_CREAT|O_RDWR);
        if (out_fd == -1) {
            fprintf(stderr, "Couldn't open output file %s\n", output_file_name);
            close(ulg_fd);
            return -1;
        }

        // First, check header
        uint8_t header[FILE_HEADER_SIZE];
        read(ulg_fd, header, FILE_HEADER_SIZE);
        bytes_remaining -= FILE_HEADER_SIZE;
        if (strncmp("ULog", header, 4)) {
            fprintf(stderr, "Bad header\n");
            close(ulg_fd);
            close(out_fd);
            return -1;
        }
        write(out_fd, header, FILE_HEADER_SIZE);

        uint32_t valid_messages = 0;
        uint32_t invalid_messages = 0;
        char last_valid_message_type = 0;
        uint16_t last_valid_message_size = 0;
        uint16_t last_data_message_topic_id = 0;
        bool last_message_valid = false;

        while (1) {
            if (bytes_remaining == 0) {
                printf("End of file reached\n");
                break;
            }
            if (bytes_remaining < MESSAGE_HEADER_SIZE) {
                fprintf(stderr, "Invalid message header size. Final %d bytes:", bytes_remaining);
                uint8_t final_bytes[MESSAGE_HEADER_SIZE];
                read(ulg_fd, final_bytes, bytes_remaining);
                for (int i = 0; i < bytes_remaining; i++) {
                    printf(" 0x%x", final_bytes[i]);
                }
                printf("\n");
                break;
            }
            uint8_t message_header[MESSAGE_HEADER_SIZE];
            read(ulg_fd, message_header, MESSAGE_HEADER_SIZE);
            bytes_remaining -= MESSAGE_HEADER_SIZE;
            uint16_t message_size = *((uint16_t*) message_header);
            uint8_t message_type = message_header[2];
            if (bytes_remaining < message_size) {
                fprintf(stderr, "Not enough bytes in file for message type 0x%x %c\n", message_type, message_type);
                fprintf(stderr, "\tAvailable bytes: %d, message size: %d\n", bytes_remaining, message_size);
                break;
            }
            if ( ! validate_message_header(message_size, message_type)) {
                if (last_message_valid) {
                    fprintf(stderr, "Skipping invalid message type 0x%x. Size %u. Offset %u\n", message_type, message_size, file_size - bytes_remaining + MESSAGE_HEADER_SIZE);
                    fprintf(stderr, "Scanning for next valid message\n");
                    // read(ulg_fd, invalid_message_data, message_size);
                    // printf("\n\t0x%.2x 0x%.2x 0x%.2x \n\t", message_header[0], message_header[1], message_header[2]);
                    // for (int i = 0; i < message_size; i++) {
                    //     printf("0x%.2x ", invalid_message_data[i]);
                    //     if (((i + 1) % 8) == 0) printf("\n\t");
                    // }
                    printf("\n\n");
                    fprintf(stderr, "\tLast valid message type: %c, size: %u\n",
                                     last_valid_message_type, last_valid_message_size);
                    fprintf(stderr, "\tLast valid data message topic id: %u\n\n\t", last_data_message_topic_id);
                    for (int i = 0; i < last_valid_message_size; i++) {
                        printf("0x%.2x ", message_data[i]);
                        if (((i + 1) % 8) == 0) printf("\n\t");
                    }
                    printf("\n");
                }
                invalid_messages++;
                last_message_valid = false;
                lseek(ulg_fd, -2, SEEK_CUR);
                bytes_remaining += 2;
                continue;
            } else {
                message_type_counters[message_type - 'A']++;
                read(ulg_fd, message_data, message_size);
                write(out_fd, message_header, MESSAGE_HEADER_SIZE);
                write(out_fd, message_data, message_size);
                valid_messages++;
                last_valid_message_type = message_type;
                last_valid_message_size = message_size;
                if (last_message_valid == false) {
                    fprintf(stderr, "\tFirst new valid message type: %c, size: %u\n\t",
                                     last_valid_message_type, last_valid_message_size);
                    printf("0x%.2x 0x%.2x 0x%.2x\n\t", message_header[0], message_header[1], message_header[2]);
                    for (int i = 0; i < last_valid_message_size; i++) {
                        printf("0x%.2x ", message_data[i]);
                        if (((i + 1) % 8) == 0) printf("\n\t");
                    }
                    printf("\n");
                }
                last_message_valid = true;
                if (last_valid_message_type == 'D') {
                    if (last_valid_message_size >= 2) {
                        last_data_message_topic_id = *((uint16_t*) message_data);
                    } else {
                        fprintf(stderr, "Invalid size for D message: %u\n", last_valid_message_size);
                    }
                }
                if (last_valid_message_type == 'A') {
                    if (last_valid_message_size >= 6) {
                        last_data_message_topic_id = *((uint16_t*) message_data);
                        message_data[last_valid_message_size] = 0;
                        printf("A type: %u, %u, %s\n", message_data[0], *((uint16_t*) &message_data[1]), &message_data[3]);
                    } else {
                        fprintf(stderr, "Invalid size for A message: %u\n", last_valid_message_size);
                    }
                }
            }
            bytes_remaining -= message_size;
        }

        close(ulg_fd);
        fsync(out_fd);
        close(out_fd);

        printf("Found %u valid messages\n", valid_messages);
        printf("Found %u invalid messages\n", invalid_messages);
        uint32_t final_offset = file_size - bytes_remaining;
        double parsed_percentage = ((double) final_offset / (double) file_size) * 100.0;
        printf("Ended at file offset %u. (%.2f)\n", final_offset, parsed_percentage);
        for (int i = 0; i < 26; i++) {
            if (message_type_counters[i]) {
                printf("Message type %c: %u\n", i + 'A', message_type_counters[i]);
            }
        }

    } else {
        fprintf(stderr, "Usage: parse <log file> <output file>\n");
    }

    return 0;
}
