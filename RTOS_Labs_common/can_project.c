// can0.c
// Runs on LM4F120/TM4C123
// Use CAN0 to communicate on CAN bus PE4 and PE5
// 

// Jonathan Valvano
// May 2, 2015

/* This example accompanies the books
   Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers, Volume 3,  
   ISBN: 978-1466468863, Jonathan Valvano, copyright (c) 2015

   Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers, Volume 2
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
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
// MCP2551 Pin1 TXD  ---- CAN0Tx PE5 (8) O TTL CAN module 0 transmit
// MCP2551 Pin2 Vss  ---- ground
// MCP2551 Pin3 VDD  ---- +5V with 0.1uF cap to ground
// MCP2551 Pin4 RXD  ---- CAN0Rx PE4 (8) I TTL CAN module 0 receive
// MCP2551 Pin5 VREF ---- open (it will be 2.5V)
// MCP2551 Pin6 CANL ---- to other CANL on network 
// MCP2551 Pin7 CANH ---- to other CANH on network 
// MCP2551 Pin8 RS   ---- ground, Slope-Control Input (maximum slew rate)
// 120 ohm across CANH, CANL on both ends of network
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/hw_can.h"
#include "../inc/hw_ints.h"
#include "../inc/hw_memmap.h"
#include "../inc/hw_types.h"
#include "../inc/can.h"
#include "../inc/cpu.h"
#include "../inc/debug.h"
#include "../inc/interrupt.h"
//#include "../RTOS_Labs_common/can0.h"
#include "../RTOS_Labs_common/OS.h"

#include "../RTOS_Labs_common/can_project.h"
#include "../RTOS_Labs_common/heap.h"
#include "../RTOS_Labs_common/OS.h"

#include <string.h>
#include "../inc/CortexM.h"


// reverse these IDs on the other microcontroller

uint32_t CAN_RCV_ID=2; // set dynamically at time of CAN0_Open
uint32_t CAN_XMT_ID=4; 

#define NULL 0
// reverse these IDs on the other microcontroller

// **************** CAN Mailbox and FIFO definitions - START ****************

//#define CAN_MAXFIFOSIZE 2048      // maximum size of FIFO buffer
#define CAN_MAXFIFOFRAMESIZE 32      // maximum size of FIFO Frame buffer 

uint32_t CAN_FIFOSize;               // size of FIFO from application
uint8_t volatile *CAN_FIFOPutPt;
uint8_t volatile *CAN_FIFOGetPt;
uint8_t CAN_FIFO[CAN_MAXFIFOSIZE];
Sema4Type CAN_FIFOCurrentSize;              // 0 if empty, FIFOSize if full
uint32_t CAN_DataLost = 0;                  // Data lost due to FIFO full

/*
// data types for CAN frame FIFO
uint32_t CAN_FIFOFrameSize;               // size of FIFO from application
uint8_t volatile *CAN_FIFOFramePutPt;
uint8_t volatile *CAN_FIFOFrameGetPt;
can_msg_t CAN_FIFOFrame[CAN_MAXFIFOFRAMESIZE];
Sema4Type CAN_FIFOFrameCurrentSize;              // 0 if empty, FIFOSize if full
uint32_t CAN_FrameDataLost = 0;                  // Data lost due to FIFO full
*/


// ******** CAN_Fifo_Init ************
// Initialize the Fifo to be empty
void CAN_Fifo_Init(uint32_t size){
  CAN_FIFOSize = size;                 // size of FIFO from application
  CAN_FIFOPutPt = CAN_FIFOGetPt = &CAN_FIFO[0];    // initially empty
  OS_InitSemaphore(&CAN_FIFOCurrentSize, 0);
};

/*
// ******** CAN_Fifo_Frame_Init ************
// Initialize the Fifo Frame to be empty
void CAN_Fifo_Frame_Init(uint32_t size){
  CAN_FIFOFrameFrameSize = size;                 // size of FIFO from application
  CAN_FIFOFramePutPt = CAN_FIFOFrameGetPt = &CAN_FIFOFrame[0];    // initially empty
  OS_InitSemaphore(&CAN_FIFOFrameCurrentSize, 0);
};
*/

// ******** CAN_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
int CAN_Fifo_Put(uint8_t data){
  if (CAN_FIFOCurrentSize.value == CAN_FIFOSize) {
    return 0;  // FIFO full, lost data
  }
  else {
    *CAN_FIFOPutPt = data;    // put data

    // Adjust put pointer
    CAN_FIFOPutPt++;
    if (CAN_FIFOPutPt == &CAN_FIFO[CAN_FIFOSize]) {
      CAN_FIFOPutPt = &CAN_FIFO[0];  // wrap around
    }

    OS_Signal(&CAN_FIFOCurrentSize);
  }

  return 1;   // success
};  

/*
// ******** CAN_Fifo_Frame_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
int CAN_Fifo_Frame_Put(can_msg_t frame){
  if (CAN_FIFOFrameCurrentSize.value == CAN_FIFOFrameSize) {
    return 0;  // FIFO full, lost data
  }
  else {
    *CAN_FIFOFramePutPt = frame;    // put data

    // Adjust put pointer
    CAN_FIFOFramePutPt++;
    if (CAN_FIFOFramePutPt == &CAN_FIFOFrame[CAN_FIFOFrameSize]) {
      CAN_FIFOFramePutPt = &CAN_FIFOFrame[0];  // wrap around
    }

    OS_Signal(&CAN_FIFOFrameCurrentSize);
  }

  return 1;   // success
};  
*/

// ******** CAN_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
uint8_t CAN_Fifo_Get(void){
  uint8_t data;

  OS_Wait(&CAN_FIFOCurrentSize);   // block if FIFO empty

  data = *CAN_FIFOGetPt;          // get data

  // Adjust get pointer
  CAN_FIFOGetPt++;
  if (CAN_FIFOGetPt == &CAN_FIFO[CAN_FIFOSize]) {
    CAN_FIFOGetPt = &CAN_FIFO[0];
  }

  return data;
};

// ******** CAN_Fifo_Size ************
// Check the status of the Fifo
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to CAN_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to CAN_Fifo_Get will spin or block
int32_t CAN_Fifo_Size(void){
  return CAN_FIFOCurrentSize.value;     // sempahore value is its current size
};

// **************** CAN FIFO definitions - END ****************

//*****************************************************************************
//
// The CAN controller interrupt handler.
//
//*****************************************************************************
uint8_t data[CAN_FRAME_SIZE];
void CAN0_Handler(void){ 
  uint32_t ulIntStatus, ulIDStatus;
  int i;
  memset(data, 0, CAN_FRAME_SIZE);
  tCANMsgObject xTempMsgObject;
  xTempMsgObject.pucMsgData = data;
  ulIntStatus = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE); // cause?
  if(ulIntStatus & CAN_INT_INTID_STATUS){  // receive?
    ulIDStatus = CANStatusGet(CAN0_BASE, CAN_STS_NEWDAT);
    for(i = 0; i < 32; i++){    //test every bit of the mask
      if( (0x1 << i) & ulIDStatus){  // if active, get data
        CANMessageGet(CAN0_BASE, (i+1), &xTempMsgObject, true);

        if(xTempMsgObject.ulMsgID == CAN_RCV_ID){
          // we know that we always send anreceive data in terms of packets
          // hence we loop over FRAM size and extract the data
          for(int j = 0; (j < CAN_FRAME_SIZE) ; j++){
            if(CAN_Fifo_Put(data[j]) == 0){
              CAN_DataLost++;
            } 
          } // end of inner for-loop
        }

      } // end of condition check
    } // end of outer for-loop
  }
  CANIntClear(CAN0_BASE, ulIntStatus);  // acknowledge
}

//Set up a message object.  Can be a TX object or an RX object.
void static CAN0_Setup_Message_Object( uint32_t MessageID, \
                                uint32_t MessageFlags, \
                                uint32_t MessageLength, \
                                uint8_t * MessageData, \
                                uint32_t ObjectID, \
                                tMsgObjType eMsgType){
  tCANMsgObject xTempObject;
  xTempObject.ulMsgID = MessageID;          // 11 or 29 bit ID
  xTempObject.ulMsgLen = MessageLength;
  xTempObject.pucMsgData = MessageData;
  xTempObject.ulFlags = MessageFlags;
  CANMessageSet(CAN0_BASE, ObjectID, &xTempObject, eMsgType);
}
// Initialize CAN port
void CAN0_Open(uint32_t rcvId, uint32_t xmtID){
  uint32_t volatile delay; 

  // enable changing the can IDs at run time
  int32_t status = StartCritical();

  CAN_RCV_ID = rcvId; 
  CAN_XMT_ID = xmtID;

  SYSCTL_RCGCCAN_R |= 0x00000001;  // CAN0 enable bit 0
  SYSCTL_RCGCGPIO_R |= 0x00000010;  // RCGC2 portE bit 4
  for(delay=0; delay<10; delay++){};
  GPIO_PORTE_AFSEL_R |= 0x30; //PORTE AFSEL bits 5,4
// PORTE PCTL 88 into fields for pins 5,4
  GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R&0xFF00FFFF)|0x00880000;
  GPIO_PORTE_DEN_R |= 0x30;
  GPIO_PORTE_DIR_R |= 0x20;
      
  CANInit(CAN0_BASE);
  CANBitRateSet(CAN0_BASE, 80000000, CAN_BITRATE);
  CANEnable(CAN0_BASE);
// make sure to enable STATUS interrupts
  CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
// Set up filter to receive these IDs
// in this case there is just one type, but you could accept multiple ID types
  //CAN0_Setup_Message_Object(CAN_RCV_ID, MSG_OBJ_RX_INT_ENABLE, 7, NULL, CAN_RCV_ID, MSG_OBJ_TYPE_RX); 
  CAN0_Setup_Message_Object(CAN_RCV_ID, MSG_OBJ_RX_INT_ENABLE, CAN_FRAME_SIZE, NULL, CAN_RCV_ID, MSG_OBJ_TYPE_RX);
  NVIC_EN1_R = (1 << (INT_CAN0 - 48)); //IntEnable(INT_CAN0);

  EndCritical(status);

  return;
}

// send 7 bytes of data to other microcontroller 
void CAN0_SendData(uint8_t data[CAN_FRAME_SIZE]){
// in this case there is just one type, but you could accept multiple ID types
  CAN0_Setup_Message_Object(CAN_XMT_ID, NULL, CAN_FRAME_SIZE, data, CAN_XMT_ID, MSG_OBJ_TYPE_TX);
}

uint8_t can_send_message[CAN_FRAME_SIZE];
void CAN0_SendMessage(uint32_t size, uint8_t* data){
  memset((void *) can_send_message, 0 , CAN_FRAME_SIZE);

  // We use big endian format while transfering the size.
  can_send_message[0] = (uint8_t)(size >> 24);
  can_send_message[1] = (uint8_t)((size << 8) >> 24);
  can_send_message[2] = (uint8_t)((size << 16) >> 24);
  can_send_message[3] = (uint8_t)((size << 24) >> 24);

  //CAN0_Setup_Message_Object(CAN_XMT_ID, NULL, CAN_FRAME_SIZE, can_send_message, CAN_XMT_ID, MSG_OBJ_TYPE_TX);
  //for(int i = 0; i < 6400; i++); // delay
  OS_MailBox_Send(can_send_message);


  while(size > 0){
    uint8_t msg_size = (size > CAN_FRAME_SIZE) ? CAN_FRAME_SIZE : size; // current bytes to be sent
    memset((void*) can_send_message, 0, CAN_FRAME_SIZE);
    memcpy(can_send_message, data, msg_size); // copy those bytes

    //CAN0_Setup_Message_Object(CAN_XMT_ID, NULL, CAN_FRAME_SIZE, 
    //            can_send_message, CAN_XMT_ID, MSG_OBJ_TYPE_TX); // send
    OS_MailBox_Send(can_send_message);

    size -= msg_size; // update
    data += msg_size; // update the data pointer
    for(int i = 0; i < 6400; i++); // delay
  }

  return;

  /*
  uint32_t curr_sent = 4;
  
  if(size <= 252) {
    memcpy(can_send_message + 4, data, size); CAN0_Setup_Message_Object(CAN_XMT_ID, NULL, 256, can_send_message, CAN_XMT_ID, MSG_OBJ_TYPE_TX); }
  else {
    memcpy(can_send_message + 4, data, 252); 
    CAN0_Setup_Message_Object(CAN_XMT_ID, NULL, 256, can_send_message, CAN_XMT_ID, MSG_OBJ_TYPE_TX);
    size -= 252;
    data += 252;

    while(size > 0){
      uint8_t msg_size = (size > 256) ? 256 : size; // current bytes to be sent
      memcpy(can_send_message, data, msg_size); // copy those bytes
      CAN0_Setup_Message_Object(CAN_XMT_ID, NULL, msg_size, 
                  can_send_message, CAN_XMT_ID, MSG_OBJ_TYPE_TX); // send
      size -= msg_size; // update
      data += msg_size; // update the data pointer
    }
  }
  */
}

// The first four bytes of a message is the size of the message.
// We simply pick up four bytes from the FIFO and make a size object.
uint32_t CAN0_ReceiveSize(void){
  uint32_t size = 0;

  // we assume that the first frame is dedicated to the size in bytes of the
  // complete message that is being sent

  // first four bytes of the packet belongs to the size.
  // the message is in big endian format for the size. 
  size |= (CAN_Fifo_Get() << 24);
  size |= (CAN_Fifo_Get()<< 16);
  size |= (CAN_Fifo_Get()<< 8);
  size |= CAN_Fifo_Get();

  // discard CAN_FRAME_SIZE - 4 packets after this. They must be gibberish
  for(int i =0; i < CAN_FRAME_SIZE - 4; i++)
    CAN_Fifo_Get(); // discard

  return size;
}

// NOTE: DESIGN 
// We always send in terms of packetes.
// The first packet sent will have the size of the message.
// To communicate the packets to the application, we maintain a packet based
// FIFO. There is no byte level FIFO anymore. The msgLength is not reliable.
// Hence, we need to communicate in terms of packets.
// Assumption: data pointer has the size number of bytes allocated.
void CAN0_ReceiveMessage(uint32_t size, uint8_t* data){

  // once we get the size. Allocate the memory on heap and return the pointer
  //uint8_t* data = (uint8_t *)Heap_Malloc(size);

  //uint8_t nr_pckts = (size % 252) ? size/252 : size/252 + 1;
  // we need the data pointer to be big enough to store the data.
  int i;

  for(i = 0; i < size; i++)
    *(data + i) = CAN_Fifo_Get();

  // Discard the other irrelevent bits in the frame
  int ceil_size =  ((size % 8 ) == 0)  ? size 
                                        : (size + 8 - (size %8));
  for(; i < ceil_size; i++)
    CAN_Fifo_Get();

  return;
}


/*
// ******** CAN_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
int CAN_fifo_frame_put(can_msg_t frame){
  if (CAN_FIFOCurrentSize.value == CAN_FIFOSize) {
    return 0;  // FIFO full, lost data
  }
  else {
    *CAN_FIFOPutPt = frame;    // put data

    // Adjust put pointer
    CAN_FIFOPutPt++;
    if (CAN_FIFOPutPt == &CAN_FIFO[CAN_FIFOSize]) {
      CAN_FIFOPutPt = &CAN_FIFO[0];  // wrap around
    }

    OS_Signal(&CAN_FIFOCurrentSize);
  }

  return 1;   // success
};  
*/

extern uint8_t MailboxDataValid;
uint8_t can_send_message_timer[CAN_FRAME_SIZE];
void can_timerproc(void) {
  if(MailboxDataValid){
    memset((void *) can_send_message_timer, 0 , CAN_FRAME_SIZE);
    OS_MailBox_Recv((uint8_t*)can_send_message_timer);
    CAN0_Setup_Message_Object(CAN_XMT_ID, 
                              NULL, 
                              CAN_FRAME_SIZE, 
                              can_send_message_timer, 
                              CAN_XMT_ID, 
                              MSG_OBJ_TYPE_TX); // send
  }

  return;
}
