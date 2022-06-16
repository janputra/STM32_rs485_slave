#ifndef __MESSAGE_H__
#define __MESSAGE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct messsage
{
   uint8_t header;
   uint8_t address;
   uint8_t operation;
   uint8_t lenght;
   uint8_t data[64];
   uint8_t checksum;
  /* data */
}message;


void buffer_to_message(circular_buffer* buffer, message *message);

#ifdef __cplusplus
}
#endif
#endif /*__MESSAGE_H__ */
