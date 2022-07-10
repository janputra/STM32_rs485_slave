#ifndef __BUFFER_H__
#define __BUFFER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define BUFFER_SIZE 64
typedef struct circular_buffer
{
  uint8_t head;
  uint8_t tail;
  uint8_t data[BUFFER_SIZE];

}circular_buffer;

typedef struct tx_msg
{
  uint8_t addr;
  uint8_t func_code;
  uint8_t data;
  /* data */
} tx_msg;

typedef struct buffer_tx_msg
{
  uint8_t head;
  uint8_t tail;
  tx_msg buffer[BUFFER_SIZE];

} buffer_tx_msg;


void buffer_push(circular_buffer *buffer, uint8_t input);
uint8_t buffer_pop(circular_buffer *buffer);

#ifdef __cplusplus
}
#endif
#endif /*__BUFFER_H__*/
