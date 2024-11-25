/*
 * csnp.h
 *
 *      Author: Ryan Huston
 *      CAN-Based Sensor Network Protocol (CSNP)
 */

#ifndef INC_CSNP_H_
#define INC_CSNP_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "main.h"
#include "stm32l4xx_hal.h"
#include "uart.h"



// Uncomment exactly one of the following lines to select the desired node:
#define N1_3843
//#define N2_4313
//#define N3_2137
//#define N4_5643


// STD Node ID's must be between 0x0 and 0x7FF
// EXT Node ID's must be between 0x0 and 0x1FFFFFFF
#define N1_ID   0x34B
#define N2_ID   0x139
#define N3_ID   0x089
#define N4_ID   0x283

#ifdef N1_3843
#define TRANSMIT
#define NODE_1 N1_ID
#define NODE_2 N2_ID
#elif defined N2_4313
#define RECEIVE
#define NODE_1 N1_ID
#define NODE_2 N2_ID
#elif defined N3_2137
#define TRANSMIT
#define NODE_1 N3_ID
#define NODE_2 N4_ID
#elif defined N4_5643
#define RECEIVE
#define NODE_1 N3_ID
#define NODE_2 N4_ID
#else
#error "Please define a valid node"
#endif

// Constants
#define STDIDMAX 0x7FF
#define EXTIDMAX 0x1FFFFFFF

extern CAN_HandleTypeDef hcan1;

// Type Definitions
enum CSNP_MSG_OBJ_ID_TYPE {
    CSNP_FRAME_STD = CAN_ID_STD,
    CSNP_FRAME_EXT = CAN_ID_EXT,
};

enum CAN_MSG_OBJ_FRAME_TYPE {
    CSNP_FRAME_DATA      = CAN_RTR_DATA,
    CSNP_FRAME_RTR       = CAN_RTR_REMOTE,
};

struct CSNP_MSG_HEADER {
    //uint32_t StdId:11;
    //uint32_t ExtId:29;
    uint32_t IDE:3;
    uint32_t RTR:2;
    uint32_t DLC:4;

    // TX Only
    FunctionalState TransmitGlobalTime:1;

    // RX Only
    uint32_t Timestamp:16;
    uint32_t FilterMatchIndex:8;
};

struct CSNP_MSG_OBJ {
    uint32_t msgId;
    struct CSNP_MSG_HEADER header;
    uint8_t *data;
};



typedef enum {
    CSNP_OK = 0x00,
    CSNP_ERROR = 0x01,
    CSNP_BUSY = 0x02,
    CSNP_TIMEOUT = 0x03
} CSNP_StatusTypeDef;

typedef enum {
    CSNP_FRAME_TYPE_SINGLE = 0,
    CSNP_FRAME_TYPE_FIRST = 1,
    CSNP_FRAME_TYPE_CONSECUTIVE = 2,
    CSNP_FRAME_TYPE_FLOW_CONTROL = 3,
    CSNP_FRAME_TYPE_ERROR = 5
} CSNP_FRAME_TYPE;

typedef enum {
    CTS = 0,
    WAIT = 1,
    OVERFLOW = 2,
} CSNP_FLOW_TYPE;

typedef struct {
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
} CSNP_Tx_Frame;

typedef struct {
    uint8_t *data;
    uint32_t length;
} CSNP_Payload;

typedef struct {
    CSNP_FRAME_TYPE type;
    uint32_t rtr;
    uint8_t dataLength;
    uint8_t data[8];
    uint8_t seqNum;
} CSNP_Rx_Parsed;

typedef struct {
    uint16_t BlockSize;
    uint16_t SeparationTime;
} Parameter;

// CAN Initialization and Configuration
CSNP_StatusTypeDef CSNP_Start();
CSNP_StatusTypeDef CSNP_Stop();

// CAN Transmission Functions
CSNP_StatusTypeDef CSNP_Tx_Init(const CAN_TxHeaderTypeDef *header);
CSNP_StatusTypeDef CSNP_Tx_Send(CSNP_Tx_Frame *tx_frame, uint32_t *mailbox);

// CAN Reception Functions
CSNP_StatusTypeDef CSNP_Rx_Start_Msg_Pending(CAN_FilterTypeDef *filterConfig);
bool CSNP_Rx_Wait(CSNP_Rx_Parsed *parsed, uint32_t FilterFIFO, uint32_t id, uint32_t *payloadLength);
CSNP_Payload* CSPN_Rx_Get_Data(uint32_t nodeId,const CAN_FilterTypeDef *filterConfig);

// Helper Functions
bool CSNP_Id_Compare(CAN_RxHeaderTypeDef *header, uint32_t nodeId);
uint32_t CSNP_Rx_Parse_Frame(CSNP_Rx_Parsed *parsed, CAN_RxHeaderTypeDef *header, uint8_t *data);
uint8_t CSNP_Tx_Create_Flow(CSNP_Tx_Frame *tx_frame, uint8_t flag, uint8_t block, uint8_t time);
uint8_t CSNP_Tx_Create_Frame(CSNP_Tx_Frame *tx_frame, uint32_t frameNum, uint8_t *buffer, uint32_t bufLength);
CSNP_StatusTypeDef CSNP_Tx_Send_Data(uint32_t nodeId, uint8_t *dataBuffer, uint32_t length, CAN_FilterTypeDef *filterConfig);

#endif /* INC_CSNP_H_ */
