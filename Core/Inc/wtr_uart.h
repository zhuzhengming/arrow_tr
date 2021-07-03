#ifndef WTR_UART_H
#define WTR_UART_H

#include "wtr_definition.h"

#define CH0_BIAS 1024
#define CH1_BIAS 1024
#define CH2_BIAS 1024
#define CH3_BIAS 1024
#define CH_RANGE 660.0

typedef struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t left;
	int8_t right;
}Remote_t;

#endif
