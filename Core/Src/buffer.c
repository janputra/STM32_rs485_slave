#include "buffer.h"

void buffer_push(circular_buffer *buffer, uint8_t input)
{   
    buffer->data[buffer->head]=input;
    buffer->head = (buffer->head+1)%BUFFER_SIZE;
  
}

uint8_t buffer_pop(circular_buffer *buffer)
{
    uint8_t buf_data = buffer->data[buffer->tail];
    buffer->tail= (buffer->tail+1)%BUFFER_SIZE;
    return buf_data;


}