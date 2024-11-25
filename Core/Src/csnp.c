/*
 * csnp.c
 *
 *      Author: Ryan Huston
 *      CAN-Based Sensor Network Protocol (CSNP)
 *
 *      CSNP Library
 */

/* Include the header file */
#include <csnp.h>

/* Global variables (for internal use) */
static CAN_TxHeaderTypeDef CSNP_txHeader;
uint32_t CSNP_Tx_Mailbox;
static CAN_RxHeaderTypeDef CSNP_RxFifo0MsgHeader;
static CAN_RxHeaderTypeDef CSNP_RxFifo1MsgHeader;
static uint8_t CSNP_RxFifo0MsgData[8];
static uint8_t CSNP_RxFifo1MsgData[8];
static volatile bool CSNP_RxFifo0MsgReceived = false;
static volatile bool CSNP_RxFifo1MsgReceived = false;

/* Function: CSNP_Start
 * Initializes and starts the CAN peripheral.
 *
 * Parameters:
 *
 * Returns:
 *   CSNP_OK on success, CSNP_ERROR on null pointer input or CAN initialization failure.
 */
CSNP_StatusTypeDef CSNP_Start() {
    // Start the CAN peripheral
    if( HAL_CAN_Start(&hcan1) != HAL_OK){
        return CSNP_ERROR;
    }
    return CSNP_OK;
}

/* Function: CSNP_Tx_Init
 * Initializes the transmit header information.
 *
 * Parameters:
 *   header: Pointer to the CAN_TxHeaderTypeDef structure containing the header information.
 *
 * Returns:
 *   CSNP_OK on success, CSNP_ERROR on null pointer input.
 */
CSNP_StatusTypeDef CSNP_Tx_Init(const CAN_TxHeaderTypeDef *header) {
    if( header == NULL){
        return CSNP_ERROR;
    }
    // Copy the entire header structure using memcpy
    memcpy(&CSNP_txHeader, header, sizeof(CAN_TxHeaderTypeDef));
    return CSNP_OK;
}

/*
 * Sends a CAN message.
 *
 * Parameters:
 *   tx_frame: Pointer to a CSNP_Tx_Frame structure containing the message to be sent.
 *   mailbox: Pointer to a variable to store the mailbox used for transmission.
 *
 * Returns:
 *   CSNP_OK on success, CSNP_ERROR on failure.
 */
CSNP_StatusTypeDef CSNP_Tx_Send(CSNP_Tx_Frame *tx_frame, uint32_t *mailbox) {
    if( HAL_CAN_AddTxMessage(&hcan1, &tx_frame->header, tx_frame->data, mailbox) != HAL_OK){
        // handle error
        return CSNP_ERROR;
    }
    while( HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3){
    }
    return CSNP_OK;
}

/* Function: CSNP_Rx_Start_Msg_Pending
 * Enables message pending interrupts for the specified CAN filter.
 *
 * Parameters:
 *   filterConfig: Pointer to the CAN filter configuration structure.
 *
 * Returns:
 *   CSNP_OK on success, CSNP_ERROR on null pointer input or CAN configuration failure.
 */
CSNP_StatusTypeDef CSNP_Rx_Start_Msg_Pending(CAN_FilterTypeDef *filterConfig) {
    if( filterConfig == NULL){
        return CSNP_ERROR;
    }
    if( HAL_CAN_ConfigFilter(&hcan1, filterConfig) != HAL_OK
            || HAL_CAN_ActivateNotification(&hcan1,
                    filterConfig->FilterFIFOAssignment ? CAN_IT_RX_FIFO1_MSG_PENDING : CAN_IT_RX_FIFO0_MSG_PENDING)
                    != HAL_OK){
        return CSNP_ERROR;
    }
    return CSNP_OK;
}

/* Function: CSNP_Rx_Wait
 * Waits for a CAN message to be received and checks if it matches the expected node ID.
 *
 * Parameters:
 *   parsed: Pointer to a CSNP_Rx_Parsed structure to store parsed message data.
 *   FilterFIFO: Specifies the FIFO to check.
 *   nodeId: The expected node ID of the message.
 *   payloadLength: Pointer to store the length of the received payload.
 *
 * Returns:
 *   true if a message is received and matches the node ID, false otherwise.
 */
bool CSNP_Rx_Wait(CSNP_Rx_Parsed *parsed, uint32_t FilterFIFO, uint32_t nodeId, uint32_t *payloadLength) {
    while( !(FilterFIFO ? CSNP_RxFifo1MsgReceived : CSNP_RxFifo0MsgReceived)){
    }
    // Determine the active FIFO
    CAN_RxHeaderTypeDef *header;
    uint8_t *data;
    if( FilterFIFO){
        header = &CSNP_RxFifo1MsgHeader;
        data = CSNP_RxFifo1MsgData;
        CSNP_RxFifo1MsgReceived = false;
    } else{
        header = &CSNP_RxFifo0MsgHeader;
        data = CSNP_RxFifo0MsgData;
        CSNP_RxFifo0MsgReceived = false;
    }
    // Check if the received message matches the expected node ID
    if( CSNP_Id_Compare(header, nodeId)){
        *payloadLength = CSNP_Rx_Parse_Frame(parsed, header, data);
        return true;
    } else {
        return false;
    }
}

uint8_t findFirstSetBit(uint32_t num) {
    uint8_t position = 0;
    while (!(num & 1)) {
        num >>= 1;
        position++;
    }
    return position;
}

uint16_t CSNP_Copy_Parsed_ToPayload(CSNP_Payload* payload, CSNP_Rx_Parsed *parsed, uint8_t upper){
    uint32_t offset = 0;

        switch(parsed->type){
            case CSNP_FRAME_TYPE_SINGLE:
            case CSNP_FRAME_TYPE_FIRST:
                payload->data = (uint8_t*) realloc(payload->data, sizeof(uint8_t) * payload->length);
                if (payload->data == NULL) {
                    free(payload->data);
                    return -1;
                }
                break;
            case CSNP_FRAME_TYPE_CONSECUTIVE:
                offset = ((upper << 4) + parsed->seqNum) * 7 - 1;
                break;
            case CSNP_FRAME_TYPE_FLOW_CONTROL:
                break;
            default: // CSNP_FRAME_TYPE_ERROR
                return -1;
                break;
        }
        memcpy(payload->data + offset, parsed->data, parsed->dataLength);
        return ((upper << 4) + parsed->seqNum);

}

/* Function: CSPN_Rx_Get_Data
 * Retrieves a complete message from the CAN bus.
 *
 * Parameters:
 *   nodeId: The node ID of the expected message.
 *   filterConfig: Pointer to the CAN filter configuration structure.
 *
 * Returns:
 *   Pointer to a CSNP_Payload structure containing the received data, or NULL on error.
 */
// This function retrieves CAN data for a specific node ID
CSNP_Payload* CSPN_Rx_Get_Data(uint32_t nodeId, const CAN_FilterTypeDef *filterConfig) {
    // Check for invalid filter configuration or node ID
    if (filterConfig == NULL ||
        (CSNP_txHeader.IDE == CAN_ID_STD && (nodeId < 0 || nodeId > STDIDMAX)) ||
        (CSNP_txHeader.IDE == CAN_ID_EXT && (nodeId < 0 || nodeId > EXTIDMAX))) {

        return NULL;
    }

    // Allocate memory for the payload struct
    CSNP_Payload *payload = (CSNP_Payload*) malloc(sizeof(CSNP_Payload));
    if (payload == NULL) {
        free(payload);
        return NULL;
    }
    payload->data = NULL;
    payload->length = 1;

    // Initialize payload length
    uint32_t payloadLength = 0;
    CSNP_Rx_Parsed parsed;
    parsed.seqNum = 0;

    CSNP_Tx_Frame frame;
    frame.header = CSNP_txHeader;
    uint16_t blockCount = 1;
    uint8_t upper = 0;
    uint32_t preSeq = 0;
    uint32_t received = 0xFFFF;
    uint32_t offset = 0;
    uint32_t mask;
    Parameter parameter = { .BlockSize = 16, .SeparationTime = 10};

    while(offset < payload->length){
        uint32_t ticks = HAL_GetTick();
        while (!CSNP_Rx_Wait(&parsed, filterConfig->FilterFIFOAssignment, nodeId, &payloadLength)) {
        }
        if(parsed.type == CSNP_FRAME_TYPE_FIRST || parsed.type == CSNP_FRAME_TYPE_SINGLE){
            payload->length = payloadLength;
        }
        if (parsed.seqNum < preSeq) {
            upper++;
        }
        preSeq = parsed.seqNum;
        received &= ~(1 << CSNP_Copy_Parsed_ToPayload(payload, &parsed, upper));
        if(parameter.BlockSize == blockCount || parsed.type == CSNP_FRAME_TYPE_FIRST){
            parameter.SeparationTime = (HAL_GetTick() - ticks) / parameter.BlockSize;
            mask = (1 << parameter.BlockSize) - 1;
            if((mask & received) || received == 0){
                CSNP_Tx_Create_Flow(&frame, CTS, parameter.BlockSize, parameter.SeparationTime);
                blockCount = 1;
            } else {
                frame.header.RTR = CAN_RTR_REMOTE;
                frame.data[0] = upper;
                frame.data[1] = findFirstSetBit(received);
                if(frame.data[1] - parameter.BlockSize  >= -1){
                    frame.data[0] = upper;
                } else {
                    frame.data[0] = upper - 1;
                }
                frame.header.DLC = 2;
                CSNP_Tx_Create_Frame(&frame, 0, frame.data, frame.header.DLC);
            }
            CSNP_Tx_Send( &frame, &CSNP_Tx_Mailbox);  // Transmit the CAN frame
        } else {
            blockCount++;
        }
        offset = ((upper << 4) + parsed.seqNum) * 7 + 6;
    }
    return payload;
}

/*
 * Compares the node ID with the appropriate identifier (extended or standard)
 *
 * Parameters:
 *   header: Pointer to a CAN_RxHeaderTypeDef structure containing the received message header.
 *   nodeId: The node ID to compare against.
 *
 * Returns:
 *   true if the node IDs match, false otherwise.
 */
bool CSNP_Id_Compare(CAN_RxHeaderTypeDef *header, uint32_t nodeId) {
    // Compare the node ID with the appropriate identifier (extended or standard)
    return nodeId == (header->IDE ? header->ExtId : header->StdId);
}

/*
 * Parses a received CAN message frame.
 *
 * Parameters:
 *   parsed: Pointer to a CSNP_Rx_Parsed structure to store the parsed data.
 *   header: Pointer to a CAN_RxHeaderTypeDef structure containing the received message header.
 *   data: Pointer to the received message data.
 *
 * Returns:
 *   The total length of the message if it's a multi-frame message, 0 otherwise.
 */
uint32_t CSNP_Rx_Parse_Frame(CSNP_Rx_Parsed *parsed, CAN_RxHeaderTypeDef *header, uint8_t *data) {
    parsed->type = data[0] >> 4;
    parsed->rtr = header->RTR;

    switch( parsed->type){
        case CSNP_FRAME_TYPE_SINGLE:
            parsed->dataLength = header->DLC - 1;
            // Error checking for mismatched data length
            if( parsed->dataLength != (0x0F & data[0])){
                // Handle error condition
            }
            memcpy(parsed->data, &data[1], parsed->dataLength);
            return parsed->dataLength;
        case CSNP_FRAME_TYPE_FIRST:
            parsed->dataLength = header->DLC - 2;
            memcpy(parsed->data, &data[2], parsed->dataLength);
            return ((0x0F & data[0]) << 8) + data[1];
        case CSNP_FRAME_TYPE_CONSECUTIVE:
            parsed->seqNum = 0x0F & data[0];
            parsed->dataLength = header->DLC - 1;
            memcpy(parsed->data, &data[1], parsed->dataLength);
            return 0; // No total length for consecutive frames
        case CSNP_FRAME_TYPE_FLOW_CONTROL:
            parsed->data[0] = 0x0F & data[0];
            parsed->dataLength = header->DLC;
            memcpy(&parsed->data[1], &data[1], parsed->dataLength - 1);
            return 0; // No total length for flow control frames
        default:
            // Handle invalid frame type
            return 0;
    }
}

/*
 * Creates a flow control frame.
 *
 * Parameters:
 *   tx_frame: Pointer to a CSNP_Tx_Frame structure to store the flow control frame.
 *   flag: Flag indicating the type of flow control frame.
 *   block: Block size for data transfer.
 *   time: Time interval for the next flow control frame.
 *
 * Returns:
 *   1 on success, 0 on failure.
 */
uint8_t CSNP_Tx_Create_Flow(CSNP_Tx_Frame *tx_frame, uint8_t flag, uint8_t block, uint8_t time) {
    if( tx_frame == NULL){
        return 0; // Handle null pointers
    }
    if(tx_frame->header.RTR != CAN_RTR_DATA){
        tx_frame->header.RTR = CAN_RTR_DATA;
    }
    // Directly assign values to the data array
    tx_frame->data[0] = 0x30 | flag;
    tx_frame->data[1] = block;
    tx_frame->data[2] = time;
    tx_frame->header.DLC = 3;

    return 1;
}

/**
 * Creates a CAN message frame based on the given data and frame number.
 *
 * Parameters:
 * tx_frame: Pointer to the CAN message frame to be filled.
 * frameNum: The sequence number of the frame (0 for the first frame).
 * buffer: Pointer to the data buffer to be transmitted.
 * bufLength: The length of the data buffer in bytes.
 *
 * Returns:
 * CSNP_FRAME_TYPE_SINGLE: If the frame is a single frame.
 * CSNP_FRAME_TYPE_FIRST: If the frame is the first frame of a segmented message.
 * CSNP_FRAME_TYPE_CONSECUTIVE: If the frame is a consecutive frame of a segmented message.
 * CSNP_FRAME_TYPE_ERROR: If an error occurs during frame creation.
 */
uint8_t CSNP_Tx_Create_Frame(CSNP_Tx_Frame *tx_frame, uint32_t frameNum, uint8_t *buffer, uint32_t bufLength) {
    if( tx_frame == NULL || buffer == NULL){
        return CSNP_FRAME_TYPE_ERROR; // Handle null pointers
    }

    if( frameNum == 0){
        if( bufLength < 8){
            // Single frame
            tx_frame->data[0] = bufLength;
            tx_frame->header.DLC = bufLength + 1;
            memcpy(tx_frame->data + 1, buffer, bufLength); // Copy all data
            return CSNP_FRAME_TYPE_SINGLE;
        } else{
            // First frame
            tx_frame->data[0] = 0x10 + (0x0F & (bufLength >> 8));
            tx_frame->data[1] = 0xFF & bufLength;
            tx_frame->header.DLC = 8;
            memcpy(tx_frame->data + 2, buffer, 6);
            return CSNP_FRAME_TYPE_FIRST;
        }
    } else{
        // Consecutive frame
        uint32_t offset = 7 * frameNum - 1;
        if( offset >= bufLength || bufLength < 8){
            return CSNP_FRAME_TYPE_ERROR; // Check for invalid offset or buffer length
        }
        tx_frame->data[0] = 0x20 + (0x0F & frameNum);
        tx_frame->header.DLC = (bufLength - offset < 7) ? bufLength - offset + 1 : 8;
        memcpy(tx_frame->data + 1, &buffer[offset], tx_frame->header.DLC - 1);
        return CSNP_FRAME_TYPE_CONSECUTIVE;
    }
    return CSNP_FRAME_TYPE_ERROR;
}

/**
 * Transmits data to a specific CAN node with flow control.
 *
 * This function prepares and transmits CAN messages to the specified node using the provided data buffer and length.
 * It implements flow control by sending and receiving flow control frames to ensure reliable data transmission.
 *
 * Parameters:
 * nodeId: The ID of the CAN node to transmit data to.
 * dataBuffer: Pointer to the buffer containing the data to transmit.
 * length: The length of the data in bytes.
 * filterConfig: Pointer to the CAN filter configuration for reception.
 *
 *Returns:
 * CSNP_OK on successful transmission, CSNP_ERROR on failure.
 */
CSNP_StatusTypeDef CSNP_Tx_Send_Data(uint32_t nodeId, uint8_t *dataBuffer, uint32_t length, CAN_FilterTypeDef *filterConfig) {
  CSNP_Rx_Parsed parsed;
  CSNP_Tx_Frame frame;
  frame.header = CSNP_txHeader; // Assuming CSNP_txHeader is pre-defined

  uint8_t delay;
  uint8_t block = 0;
  uint8_t count = 0;
  uint32_t payloadLength;

  // Loop until all data is sent or an error occurs
  for (int i = 0; CSNP_Tx_Create_Frame(&frame, i, dataBuffer, length) != CSNP_FRAME_TYPE_ERROR; i++) {
    CSNP_Tx_Send(&frame, &CSNP_Tx_Mailbox);
    // Check for flow control
    if (i == 0 || block == count) {
      // Wait for response with timeout handling
        uint32_t startTime = HAL_GetTick();
        while (!CSNP_Rx_Wait(&parsed, filterConfig->FilterFIFOAssignment, nodeId, &payloadLength)) {
            if (HAL_GetTick() - startTime >= 100) {
                CSNP_Tx_Send(&frame, &CSNP_Tx_Mailbox);
                startTime = HAL_GetTick(); // Reset the timer
            }
        }
      // Check for overflow error
      if (parsed.data[0] == OVERFLOW) {
        return CSNP_ERROR;
      }

      // Handle missing frame requests
      while (parsed.rtr == CAN_RTR_REMOTE) {
        // Reconstruct frame based on received block and offset
        CSNP_Tx_Create_Frame(&frame, (parsed.data[0] << 4) + parsed.data[1], dataBuffer, length);
        CSNP_Tx_Send(&frame, &CSNP_Tx_Mailbox);

        // Wait for next response
        while (!CSNP_Rx_Wait(&parsed, filterConfig->FilterFIFOAssignment, nodeId, &payloadLength)) {
        }
      }
      // Update block and delay from received data
      block = parsed.data[1];
      delay = parsed.data[2];
      count = 0;
    }
    count++;
    HAL_Delay(delay);
  }
  return CSNP_OK;
}

/*
 * Interrupt service routine for CAN RX FIFO 0.
 *
 * Parameters:
 *   hcan: Pointer to the CAN handle.
 *
 * Returns:
 *   None.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if( HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CSNP_RxFifo0MsgHeader, CSNP_RxFifo0MsgData) != HAL_OK){
        // Handle error
    }
    CSNP_RxFifo0MsgReceived = true;
}

/*
 * Interrupt service routine for CAN RX FIFO 1.
 *
 * Parameters:
 *   hcan: Pointer to the CAN handle.
 *
 * Returns:
 *   None.
 */

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if( HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &CSNP_RxFifo1MsgHeader, CSNP_RxFifo1MsgData) != HAL_OK){
        // Handle error
    }
    CSNP_RxFifo0MsgReceived = true;
}
