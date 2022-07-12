#ifndef __MESSAGE_H__
#define __MESSAGE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "buffer.h"


/*
typedef struct __attribute__((packed)) messsage
{
   uint8_t address;
   uint8_t function_code;
   uint8_t data;
   uint8_t checksum;
  
}message;
*/


void buffer_to_message(circular_buffer* buffer, uint8_t *message);
void cal_checksum(uint8_t *buf);
uint8_t check_checksum(uint8_t *buf);

#ifdef __cplusplus
}
#endif
#endif /*__MESSAGE_H__ */
