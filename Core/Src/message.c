#include "message.h"

void buffer_to_message(circular_buffer* buffer, message *message)
{
    message->address = buffer_pop(buffer);
    message->function_code = buffer_pop(buffer);
    message->data= buffer_pop(buffer);
    message->checksum = buffer_pop(buffer);
}

void cal_checksum(message *msg)
{

}

uint8_t check_checksum(message msg)
{

}