#include "buffer.h"

void buffer_push(circular_buffer *buffer, uint8_t input)
{   
    buffer->data[buffer->head]=input;
    buffer->head++;
    
    if (buffer->head>63){
        buffer->head=0;
    }

}

uint8_t buffer_pop(circular_buffer *buffer)
{
    uint8_t buf_data = buffer->data[buffer->tail];
    buffer->tail++;

     if (buffer->tail>63){
        buffer->tail=0;
    }
    return buf_data;


}