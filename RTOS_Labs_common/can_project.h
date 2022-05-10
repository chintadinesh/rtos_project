// *********can0.h ***************
// Runs on LM4F120/TM4C123
// Use CAN0 to communicate on CAN bus
// CAN0Rx PE4 (8) I TTL CAN module 0 receive.
// CAN0Tx PE5 (8) O TTL CAN module 0 transmit.

// Jonathan Valvano
// May 2, 2015

/* This example accompanies the books
   Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers, Volume 3,  
   ISBN: 978-1466468863, Jonathan Valvano, copyright (c) 2015

   Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers, Volume 2
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

/*
Source: RTOS_Labs_common/can0.c

I used can0 as a starter code to extend the functionality of sending data of
multiple sizes.

Communication hierarchy:
message -> frame

Communication characteristics:
1. Although CAN can communicata data that is far larger, we maintain message
    frames of size of 256B. We chose 256 because we want to use the first byte
    representing the size of the frame
2. If the CRC fails, we ask to retransmit the entire message again. The MCUs
    are connected via wiers. The communication is expected to be very
    reliable. Hence, we are getting rid of the overhead associated with ACK
    and NACK messages.

Frame format:
1B size: 0 to 255
1B subtype: DATA, SIZE. The SIZE type messages are used by the memory
            allocators for the heap.
1B crc: 
1-252B data:


For now, we are trying a simplier implementation:
The first four bytes that are sent in any message will be the unsigned 32 bit
integer size of the message that is being sent.
*/

#ifndef __CAN0_H__
#define __CAN0_H__
#define CAN_BITRATE             1000000
#include <stdint.h>

#define CAN_MSGT_SIZE 0
#define CAN_MSGT_DATA 1

#define CAN_FRAME_SIZE 8
//#define CAN_FRAME_SIZE 16   // not working
#define CAN_MAXFIFOSIZE 64     // maximum size of FIFO buffer

// **************** CAN Frame structure ****************
typedef struct can_msg {
  uint8_t size;
  uint8_t type;
  uint8_t crc;
  uint8_t data[253];
} can_msg_t;
   

// **************** CAN Mailbox and FIFO definitions - START ****************

// ******** CAN_Fifo_Init ************
// Initialize the Fifo to be empty
void CAN_Fifo_Init(uint32_t size);
// ******** CAN_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
int CAN_Fifo_Put(uint8_t data);

// ******** CAN_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
uint8_t CAN_Fifo_Get(void);

// ******** CAN_Fifo_Size ************
// Check the status of the Fifo
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to CAN_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to CAN_Fifo_Get will spin or block
int32_t CAN_Fifo_Size(void);

// Initialize CAN port
// rcvID is the receiver ID
// xmtID is the transmitter ID
void CAN0_Open(uint32_t rcvId, uint32_t xmtID);

// send 4 bytes of data to other microcontroller 
void CAN0_SendData(uint8_t data[CAN_FRAME_SIZE]);

// Assumption: data pointer has the size number of bytes allocated.
//void CAN0_ReceiveMessage(uint32_t size, uint8_t* data); 
//uint8_t* CAN0_ReceiveMessage(void);
// unqueue size number of bytes from the FIFO queue and
// put in the ptr pointer.
void CAN0_ReceiveMessage(uint32_t size, uint8_t* ptr);

// convert the next 4 bytes in the FIFO into uint32_t type and
// get rid of extra 4 bytes from the first frame
uint32_t CAN0_ReceiveSize(void);

void CAN0_SendMessage(uint32_t size, uint8_t* data);

// timer proc for sendind the data
void can_timerproc(void);

#endif //  __CAN0_H__
