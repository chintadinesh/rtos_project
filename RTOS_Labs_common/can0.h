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

#ifndef __CAN0_H__
#define __CAN0_H__
#define CAN_BITRATE             1000000
#include <stdint.h>

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
void CAN0_SendData(uint8_t data[7]);

#endif //  __CAN0_H__
