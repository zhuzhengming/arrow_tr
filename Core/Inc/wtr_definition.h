#ifndef WTR_DEFINITION_H
#define WTR_DEFINITION_H

#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include "spi.h"

#include "DJI.h"
#include "GM6020.h"

#define HEAD_LENGTH 2
#define PAYLOAD_LENGTH 31
#define BAG_LENGTH (HEAD_LENGTH+PAYLOAD_LENGTH)

#define PI 3.1415926
#define _SIGN(x) (x>0? 1: (x<0? -1 :0) )

typedef struct {
    float motorPos;
    uint8_t electromagnetOrder;
    float laserDistance;
}__attribute__((packed)) ArrowMsg_t ;

typedef struct {
    ArrowMsg_t left;
    ArrowMsg_t mid;
    ArrowMsg_t right;
    float yaw;
}__attribute__((packed)) RecvMsg_t ;

typedef union {
	uint8_t raw[HEAD_LENGTH + PAYLOAD_LENGTH];
	struct {
		uint8_t head[HEAD_LENGTH];
		union {
			uint8_t payload[PAYLOAD_LENGTH];
			struct {
                ArrowMsg_t left;
                ArrowMsg_t mid;
                ArrowMsg_t right;
                float yaw;
            }__attribute__((packed));
		};
	} __attribute__((packed));
} __attribute__((packed)) Bag_t;

//void PID_Calc(PID_t *pid);


extern const uint8_t header[2];

extern RecvMsg_t recvMsg;
extern Bag_t sendBag;

extern uint8_t JoyStickReceiveData[18];
extern uint8_t UART6ReceiveData[BAG_LENGTH*2];
extern uint8_t CanReceiveData[8];


#endif
