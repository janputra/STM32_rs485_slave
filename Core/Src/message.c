#include "message.h"

void buffer_to_message(circular_buffer* buffer, uint8_t* message)
{
    message[0] = buffer_pop(buffer);
    message[1] = buffer_pop(buffer);
    message[2] = buffer_pop(buffer);
    message[3] = buffer_pop(buffer);
}

void cal_checksum(uint8_t *msg)
{

}

uint8_t check_checksum(uint8_t *msg)
{
return 1;
}