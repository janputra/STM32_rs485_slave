#ifndef __BUFFER_H__
#define __BUFFER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct circular_buffer
{
  uint8_t head;
  uint8_t tail;
  uint8_t data[64];

}circular_buffer;

void buffer_push(circular_buffer *buffer, uint8_t input);
uint8_t buffer_pop(circular_buffer *buffer);

#ifdef __cplusplus
}
#endif
#endif /*__BUFFER_H__*/
