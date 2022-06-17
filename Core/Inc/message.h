#ifndef __MESSAGE_H__
#define __MESSAGE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "buffer.h"

#define FUNC_WRITE 1
#define FUNC_READ 2

typedef struct __attribute__((packed)) messsage
{
   uint8_t address;
   uint8_t function_code;
   uint8_t data;
   uint8_t checksum;
  /* data */
}message;



void buffer_to_message(circular_buffer* buffer, message *message);
void cal_checksum(message* msg);
uint8_t check_checksum(message msg);

#ifdef __cplusplus
}
#endif
#endif /*__MESSAGE_H__ */
