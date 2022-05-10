// Lab5.c
// Runs on LM4F120/TM4C123
// Real Time Operating System for Lab 5

// Jonathan W. Valvano 3/29/17, valvano@mail.utexas.edu
// Andreas Gerstlauer 3/1/16, gerstl@ece.utexas.edu
// EE445M/EE380L.6 
// You may use, edit, run or distribute this file 
// You are free to change the syntax/organization of this file

// LED outputs to logic analyzer for use by OS profile 
// PF1 is preemptive thread switch
// PF2 is first periodic background task (if any)
// PF3 is second periodic background task (if any)
// PC4 is PF4 button touch (SW1 task)

// Outputs for task profiling
// PD0 is idle task
// PD1 is button task

// Button inputs
// PF0 is SW2 task
// PF4 is SW1 button input

// Analog inputs
// PE3 Ain0 sampled at 2kHz, sequencer 3, by Interpreter, using software start

#include <stdint.h>
#include <stdio.h> 
#include "../inc/tm4c123gh6pm.h"
#include "../inc/CortexM.h"
#include "../inc/LaunchPad.h"
#include "../inc/PLL.h"
#include "../inc/LPF.h"
#include "../RTOS_Labs_common/UART0int.h"
#include "../RTOS_Labs_common/ADC.h"
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/heap.h"
#include "../RTOS_Labs_common/Interpreter.h"
#include "../RTOS_Labs_common/ST7735.h"
#include "../RTOS_Labs_common/eDisk.h"
#include "../RTOS_Labs_common/eFile.h"

#include <string.h>
#include "../RTOS_Lab5_ProcessLoader/loader.h"

// can bus for the communication
#include "../RTOS_Labs_common/can_project.h"

// CAN IDs are set dynamically at time of CAN0_Open
// Reverse on other microcontroller
#define RCV_ID 2
#define XMT_ID 4

extern uint32_t CAN_DataLost;

extern uint32_t CAN_RCV_ID; // set dynamically at time of CAN0_Open
extern uint32_t CAN_XMT_ID; 

uint32_t NumCreated;   // number of foreground threads created
uint32_t IdleCount;    // CPU idle counter
uint32_t NumSamples;
int32_t DataLost;
uint32_t FilterWork;
uint32_t PIDWork;
uint32_t x[64];
uint32_t y[64];

//---------------------User debugging-----------------------
extern int32_t MaxJitter1;             // largest time jitter between interrupts in usec

#define PD0  (*((volatile uint32_t *)0x40007004))
#define PD1  (*((volatile uint32_t *)0x40007008))
#define PD2  (*((volatile uint32_t *)0x40007010))
#define PD3  (*((volatile uint32_t *)0x40007020))

void PortD_Init(void){ 
  SYSCTL_RCGCGPIO_R |= 0x08;       // activate port D
  while((SYSCTL_RCGCGPIO_R&0x08)==0){};      
  GPIO_PORTD_DIR_R |= 0x0F;        // make PD3-0 output heartbeats
  GPIO_PORTD_AFSEL_R &= ~0x0F;     // disable alt funct on PD3-0
  GPIO_PORTD_DEN_R |= 0x0F;        // enable digital I/O on PD3-0
  GPIO_PORTD_PCTL_R = ~0x0000FFFF;
  GPIO_PORTD_AMSEL_R &= ~0x0F;;    // disable analog functionality on PD
}


//------------------Task 1--------------------------------
// background thread executes with SW1 button
// one foreground task created with button push

// ***********ButtonWork*************
void ButtonWork(void){  heap_stats_t heap;
  uint32_t myId = OS_Id(); 
  PD1 ^= 0x02;
  if(Heap_Stats(&heap)) OS_Kill();
  PD1 ^= 0x02;
  ST7735_Message(1,0,"Heap size  =",heap.size); 
  ST7735_Message(1,1,"Heap used  =",heap.used);  
  ST7735_Message(1,2,"Heap free  =",heap.free);
  ST7735_Message(1,3,"Heap waste =",heap.size - heap.used - heap.free);
  PD1 ^= 0x02;
  OS_Kill();  // done, OS does not return from a Kill
} 

//************SW1Push*************
// Called when SW1 Button pushed
// Adds another foreground task
// background threads execute once and return
/*
void SW1Push(void){
  if(OS_MsTime() > 20){ // debounce
    if(OS_AddThread(&ButtonWork,100,2)){
      NumCreated++; 
    }
    OS_ClearMsTime();  // at least 20ms between touches
  }
}
*/

//************SW2Push*************
// Called when SW2 Button pushed
// Adds another foreground task
// background threads execute once and return
void SW2Push(void){
  if(OS_MsTime() > 20){ // debounce
    if(OS_AddThread(&ButtonWork,100,2)){
      NumCreated++; 
    }
    OS_ClearMsTime();  // at least 20ms between touches
  }
}

//--------------end of Task 1-----------------------------

//------------------Idle Task--------------------------------
// foreground thread, runs when nothing else does
// never blocks, never sleeps, never dies
// inputs:  none
// outputs: none
void Idle(void){
  IdleCount = 0;          
  while(1) {
    IdleCount++;
    PD0 ^= 0x01;
    WaitForInterrupt();
  }
}


//--------------end of Idle Task-----------------------------

void SW1Push(void);

//*******************final user main DEMONTRATE THIS TO TA**********
int realmain(void){ // realmain
  OS_Init();        // initialize, disable interrupts
  PortD_Init();     // debugging profile
  MaxJitter1 = 0;    // in 1us units

	// Initialize CAN with given IDs
  // CAN0_Open(RCV_ID,XMT_ID);

	// Initialize CAN FIFO
	CAN_Fifo_Init(CAN_MAXFIFOSIZE);
	
  // hardware init
  ADC_Init(0);  // sequencer 3, channel 0, PE3, sampling in Interpreter

  // Heap_Init();  // heap initialized inside OS_Launch() for our implementation
  
  // attach background tasks
  OS_AddPeriodicThread(&disk_timerproc,TIME_1MS,0);   // time out routines for disk  
  OS_AddSW1Task(&SW1Push,2);
  OS_AddSW2Task(&SW2Push,2);  

  // create initial foreground threads
  NumCreated = 0;
  NumCreated += OS_AddThread(&Interpreter,128,2); 
  NumCreated += OS_AddThread(&Idle,128,5);  // at lowest priority 
 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//+++++++++++++++++++++++++DEBUGGING CODE++++++++++++++++++++++++
// ONCE YOUR RTOS WORKS YOU CAN COMMENT OUT THE REMAINING CODE
// 

//*****************File System Test*************************
// Tests integration of FAT filesystem for Lab 5. This should succeed. 
// Warning: this reformats the disk, all existing data will be lost
void diskError(const char* errtype, uint32_t n){
  printf(errtype);
  printf(" disk error %u",n);
  OS_Kill();
}
char const string1[]="Filename = %s";
char const string2[]="File size = %lu bytes";
char const string3[]="Number of Files = %u";
char const string4[]="Number of Bytes = %lu";
uint32_t FileTestRunning = 0; 
// commenting because size exceeds linker's limits. 
/*
void TestDirectory(void){ char *name; unsigned long size; 
  unsigned int num;
  unsigned long total;
  num = 0;
  total = 0;
  printf("\n\r");
  if(eFile_DOpen(""))           diskError("eFile_DOpen",0);
  while(!eFile_DirNext(&name, &size)){
    printf(string1, name);
    printf("  ");
    printf(string2, size);
    printf("\n\r");
    total = total+size;
    num++;    
  }
  printf(string3, num);
  printf("\n\r");
  printf(string4, total);
  printf("\n\r");
  if(eFile_DClose())            diskError("eFile_DClose",0);
}
void TestFile(void){   int i; char data; 
  printf("\n\rEE445M/EE380L, Lab 5 eFile test\n\r");
  ST7735_DrawString(0, 1, "eFile test      ", ST7735_WHITE);
  // simple test of eFile
  if(eFile_Init())              diskError("eFile_Init",0); 
  if(eFile_Mount())             diskError("eFile_Mount",0);
  if(eFile_Format())            diskError("eFile_Format",0); 
  TestDirectory();
  if(eFile_Create("file1"))     diskError("eFile_Create",0);
  if(eFile_WOpen("file1"))      diskError("eFile_WOpen",0);
  for(i=0;i<1000;i++){
    if(eFile_Write('a'+i%26))   diskError("eFile_Write",i);
    if(i%52==51){
      if(eFile_Write('\n'))     diskError("eFile_Write",i);  
      if(eFile_Write('\r'))     diskError("eFile_Write",i);
    }
  }
  if(eFile_WClose())            diskError("eFile_WClose",0);
  TestDirectory();
  if(eFile_ROpen("file1"))      diskError("eFile_ROpen",0);
  for(i=0;i<1000;i++){
    if(eFile_ReadNext(&data))   diskError("eFile_ReadNext",i);
    UART_OutChar(data);
  }
  if(eFile_Delete("file1"))     diskError("eFile_Delete",0);
  TestDirectory();
  if(eFile_Unmount())           diskError("eFile_Unmount",0);
  printf("Successful test\n\r");
  ST7735_DrawString(0, 1, "eFile successful", ST7735_YELLOW);
  FileTestRunning=0; // launch again
  OS_Kill();
}

void SWPushFile(void){
  if(FileTestRunning==0){
    FileTestRunning = 1;  // prevents you from starting two test threads
    NumCreated += OS_AddThread(&TestFile,128,1);  // test eFile
  }
}

int TestmainFile(void){   // TestmainFile
  OS_Init();           // initialize, disable interrupts
  PortD_Init();
  FileTestRunning = 1; 

  // attach background tasks
  OS_AddPeriodicThread(&disk_timerproc,TIME_1MS,0);   // time out routines for disk
  OS_AddSW1Task(&SWPushFile,2);    // PF4, SW1
  OS_AddSW2Task(&SWPushFile,2);    // PF0, SW2
  
  // create initial foreground threads
  NumCreated = 0;
  NumCreated += OS_AddThread(&TestFile,128,1);  
  NumCreated += OS_AddThread(&Idle,128,3); 
 
  OS_Launch(10*TIME_1MS); // doesn't return, interrupts enabled in here
  return 0;               // this never executes
}
*/
//*****************Test project 0*************************
// This is the simplest configuration, 
// Just see if you can import your OS
// no UART interrupts
// no SYSTICK interrupts
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
// no calls to semaphores

/*
uint32_t Count1;   // number of times thread1 loops
uint32_t Count2;   // number of times thread2 loops
uint32_t Count3;   // number of times thread3 loops
void Thread1(void){
  Count1 = 0;          
  for(;;){
    PD0 ^= 0x01;       // heartbeat
    Count1++;
  }
}
void Thread2(void){
  Count2 = 0;          
  for(;;){
    PD1 ^= 0x02;       // heartbeat
    Count2++;
  }
}
void Thread3(void){
  Count3 = 0;          
  for(;;){
    PD2 ^= 0x04;       // heartbeat
    Count3++;
  }
}

int Testmain0(void){  // Testmain0
  OS_Init();          // initialize, disable interrupts
  PortD_Init();       // profile user threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1,128,1); 
  NumCreated += OS_AddThread(&Thread2,128,2); 
  NumCreated += OS_AddThread(&Thread3,128,3); 
  // Count1 Count2 Count3 should be equal or off by one at all times
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}
*/

//*****************Test project 1*************************
// Heap test, allocate and deallocate memory
/*
void heapError(const char* errtype,const char* v,uint32_t n){
  printf(errtype);
  printf(" heap error %s%u",v,n);
  OS_Kill();
}
heap_stats_t stats;
void heapStats(void){
  if(Heap_Stats(&stats))  heapError("Heap_Stats","",0);
  ST7735_Message(1,0,"Heap size  =",stats.size); 
  ST7735_Message(1,1,"Heap used  =",stats.used);  
  ST7735_Message(1,2,"Heap free  =",stats.free);
  ST7735_Message(1,3,"Heap waste =",stats.size - stats.used - stats.free);
}
int16_t* ptr;  // Global so easier to see with the debugger
int16_t* p1;   // Proper style would be to make these variables local
int16_t* p2;
int16_t* p3;
uint8_t* q1;
uint8_t* q2;
uint8_t* q3;
uint8_t* q4;
uint8_t* q5;
uint8_t* q6;
int16_t maxBlockSize;
uint8_t* bigBlock;
void TestHeap(void){  int16_t i;  
  ST7735_DrawString(0, 0, "Heap test            ", ST7735_WHITE);
  printf("\n\rEE445M/EE380L, Lab 5 Heap Test\n\r");
  if(Heap_Init())         heapError("Heap_Init","",0);

  ptr = Heap_Malloc(sizeof(int16_t));
  if(!ptr)                heapError("Heap_Malloc","ptr",0);
  *ptr = 0x1111;

  if(Heap_Free(ptr))      heapError("Heap_Free","ptr",0);

  ptr = Heap_Malloc(1);
  if(!ptr)                heapError("Heap_Malloc","ptr",1);

  if(Heap_Free(ptr))      heapError("Heap_Free","ptr",1);

  p1 = (int16_t*) Heap_Malloc(1 * sizeof(int16_t));
  if(!p1)                 heapError("Heap_Malloc","p",1);
  p2 = (int16_t*) Heap_Malloc(2 * sizeof(int16_t));
  if(!p2)                 heapError("Heap_Malloc","p",2);
  p3 = (int16_t*) Heap_Malloc(3 * sizeof(int16_t));
  if(!p3)                 heapError("Heap_Malloc","p",3);
  p1[0] = 0xAAAA;
  p2[0] = 0xBBBB;
  p2[1] = 0xBBBB;
  p3[0] = 0xCCCC;
  p3[1] = 0xCCCC;
  p3[2] = 0xCCCC;
  heapStats();

  if(Heap_Free(p1))       heapError("Heap_Free","p",1);
  if(Heap_Free(p3))       heapError("Heap_Free","p",3);

  if(Heap_Free(p2))       heapError("Heap_Free","p",2);
  heapStats();

  for(i = 0; i <= (stats.size / sizeof(int32_t)); i++){
    ptr = Heap_Malloc(sizeof(int16_t));
    if(!ptr) break;
  }
  if(ptr)                 heapError("Heap_Malloc","i",i);
  heapStats();
  
  printf("Realloc test\n\r");
  if(Heap_Init())         heapError("Heap_Init","",1);
  q1 = Heap_Malloc(1);
  if(!q1)                 heapError("Heap_Malloc","q",1);
  q2 = Heap_Malloc(2);
  if(!q2)                 heapError("Heap_Malloc","q",2);
  q3 = Heap_Malloc(3);
  if(!q3)                 heapError("Heap_Malloc","q",3);
  q4 = Heap_Malloc(4);
  if(!q4)                 heapError("Heap_Malloc","q",4);
  q5 = Heap_Malloc(5);
  if(!q5)                 heapError("Heap_Malloc","q",5);

  *q1 = 0xDD;
  q6 = Heap_Realloc(q1, 6);
  heapStats();

  for(i = 0; i < 6; i++){
    q6[i] = 0xEE;
  }
  q1 = Heap_Realloc(q6, 2);
  heapStats();

  printf("Large block test\n\r");
  if(Heap_Init())         heapError("Heap_Init","",2);
  heapStats();
  maxBlockSize = stats.free;
  bigBlock = Heap_Malloc(maxBlockSize);
  for(i = 0; i < maxBlockSize; i++){
    bigBlock[i] = 0xFF;
  }
  heapStats();
  if(Heap_Free(bigBlock)) heapError("Heap_Free","bigBlock",0);

  bigBlock = Heap_Calloc(maxBlockSize);
  if(!bigBlock)           heapError("Heap_Calloc","bigBlock",0);
  if(*bigBlock)           heapError("Zero initialization","bigBlock",0);
  heapStats();

  if(Heap_Free(bigBlock)) heapError("Heap_Free","bigBlock",0);
  heapStats();
  
  printf("Successful heap test\n\r");
  ST7735_DrawString(0, 0, "Heap test successful", ST7735_YELLOW);
  OS_Kill();
}

void SW1Push1(void){
  if(OS_MsTime() > 20){ // debounce
    if(OS_AddThread(&TestHeap,128,1)){
      NumCreated++;
    }
    OS_ClearMsTime();  // at least 20ms between touches
  }
}

int Testmain1(void){   // Testmain1
  OS_Init();           // initialize, disable interrupts
  PortD_Init();

  // attach background tasks
  OS_AddSW1Task(&SW1Push1,2);
    
  // create initial foreground threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&TestHeap,128,1);  
  NumCreated += OS_AddThread(&Idle,128,3); 
 
  OS_Launch(10*TIME_1MS); // doesn't return, interrupts enabled in here
  return 0;               // this never executes
}
*/

//*****************Test project 2*************************
// Process management test, add and reclaim dummy process
/*
void TestUser(void){ uint32_t id; uint32_t time;
  id = OS_Id();
  PD2 ^= 0x04;
  ST7735_Message(0,1, "Hello world: ", id);
  time = OS_Time();
  OS_Sleep(1000);
  time = (((OS_TimeDifference(time, OS_Time()))/1000ul)*125ul)/10000ul;
  ST7735_Message(0,2, "Sleep time: ", time);
  PD2 ^= 0x04;
  OS_Kill();
}

//  OS-internal OS_AddProcess function
extern int OS_AddProcess(void(*entry)(void), void *text, void *data, 
  unsigned long stackSize, unsigned long priority); 

void TestProcess(void){ heap_stats_t heap1, heap2;
  // simple process management test, add process with dummy code and data segments
  ST7735_DrawString(0, 0, "Process test         ", ST7735_WHITE);
  printf("\n\rEE445M/EE380L, Lab 5 Process Test\n\r");
  PD1 ^= 0x02;
  if(Heap_Stats(&heap1)) OS_Kill();
  PD1 ^= 0x02;
  ST7735_Message(1,0,"Heap size  =",heap1.size); 
  ST7735_Message(1,1,"Heap used  =",heap1.used);  
  ST7735_Message(1,2,"Heap free  =",heap1.free);
  ST7735_Message(1,3,"Heap waste =",heap1.size - heap1.used - heap1.free);
  PD1 ^= 0x02;
  if(!OS_AddProcess(&TestUser,Heap_Calloc(128),Heap_Calloc(128),128,1)){
    printf("OS_AddProcess error");
    OS_Kill();
  }
  PD1 ^= 0x02;
  OS_Sleep(2000); // wait long enough for user thread and hence process to exit/die
  PD1 ^= 0x02;
  if(Heap_Stats(&heap2)) OS_Kill();
  PD1 ^= 0x02;
  ST7735_Message(1,0,"Heap size  =",heap2.size); 
  ST7735_Message(1,1,"Heap used  =",heap2.used);  
  ST7735_Message(1,2,"Heap free  =",heap2.free);
  ST7735_Message(1,3,"Heap waste =",heap2.size - heap2.used - heap2.free);
  PD1 ^= 0x02;
  if((heap1.free != heap2.free)||(heap1.used != heap2.used)){
    printf("Process management heap error");
    OS_Kill();
  }
  printf("Successful process test\n\r");
  ST7735_DrawString(0, 0, "Process test successful", ST7735_YELLOW);
  OS_Kill();  
}

void SW2Push2(void){
  if(OS_MsTime() > 20){ // debounce
    if(OS_AddThread(&TestProcess,128,1)){
      NumCreated++;
    }
    OS_ClearMsTime();  // at least 20ms between touches
  }
}

int Testmain2(void){   // Testmain2 
  OS_Init();           // initialize, disable interrupts
  PortD_Init();

  // attach background tasks
  OS_AddSW1Task(&SW1Push1,2);  // PF4, SW1
  OS_AddSW2Task(&SW2Push2,2);  // PF0, SW2
  
  // create initial foreground threads
  NumCreated = 0;
  NumCreated += OS_AddThread(&TestProcess,128,1);  
  NumCreated += OS_AddThread(&Idle,128,3); 
 
  OS_Launch(10*TIME_1MS); // doesn't return, interrupts enabled in here
  return 0;               // this never executes
}
*/
//*****************Test project 3*************************
// Test supervisor calls (SVC exceptions)
// Using inline assembly, syntax is dependent on the compiler
// The following code compiles in Keil 5.x (even though the UI complains)
__asm uint32_t SVC_OS_Id(void){
    SVC #0
    BX  LR
  }
__asm void SVC_OS_Kill(void){
    SVC #1
    BX  LR
  }
__asm void SVC_OS_Sleep(uint32_t t){
    SVC #2
    BX  LR
  }
__asm uint32_t SVC_OS_Time(void){
    SVC #3
    BX  LR
  }
__asm int SVC_OS_AddThread(void(*t)(void), uint32_t s, uint32_t p){
    SVC #4
    BX  LR
  }
uint32_t line = 0;
void TestSVCThread(void){ uint32_t id;	
  id = SVC_OS_Id();
  PD3 ^= 0x08;
  ST7735_Message(0,line++, "Thread: ", id);
  SVC_OS_Sleep(500);
  ST7735_Message(0,line++, "Thread dying: ", id);
  PD3 ^= 0x08;
  SVC_OS_Kill();
}
void TestSVC(void){ uint32_t id; uint32_t time;
  // simple SVC test, mimicking real user program
  ST7735_DrawString(0, 0, "SVC test         ", ST7735_WHITE);
  printf("\n\rEE445M/EE380L, Lab 5 SCV Test\n\r");
  id = SVC_OS_Id();
  PD2 ^= 0x04;
  ST7735_Message(0,line++, "SVC test: ", id);
  SVC_OS_AddThread(TestSVCThread, 128, 1);
  time = SVC_OS_Time();
  SVC_OS_Sleep(1000);
	time = (((OS_TimeDifference(time, SVC_OS_Time()))/1000ul)*125ul)/10000ul;
  ST7735_Message(0,line++, "Sleep time: ", time);
  PD2 ^= 0x04;
  if(line != 4) {
    printf("SVC test error");
    OS_Kill();
  }
  printf("Successful SVC test\n\r");
  ST7735_Message(0,0, "SVC test done ", id);
  SVC_OS_Kill();
}

void SWPush3(void){
  if(line>=4){
    line = 0;
    if(OS_AddThread(&TestSVC,128,1)){
      NumCreated++;
    }
  }
}

int Testmain3(void){   // Testmain3 
  OS_Init();           // initialize, disable interrupts
  PortD_Init();

  // attach background tasks
  OS_AddSW1Task(&SWPush3,2);  // PF4, SW1
  OS_AddSW2Task(&SWPush3,2);  // PF0, SW2
  
  // create initial foreground threads
  NumCreated = 0;
  NumCreated += OS_AddThread(&TestSVC,128,1);  
  NumCreated += OS_AddThread(&Idle,128,3); 
 
  OS_Launch(10*TIME_1MS); // doesn't return, interrupts enabled in here
  return 0;               // this never executes
}

/*
################
PROCESS MIGRATION

Assumptions:
1. At the time of migration, there is no IO. We simplify the process for now
and say that we don't have any migration in the process at all.
2. To start with, we just verify in the debugger that the process is updating
some variable.

###############
*/

/*
BLOCK the process from running.
The scheduler should stop running any threads that belongs to the process.
If we have IO, we make sure that the IO is complete. Or, we can also make the
design to do the IO in special mode, during which we do not do the migration.

Return 0 on success.

Q:To make sure that enough resources are present in the other mcu.
A: This is not really a requirement.

Q: Every IO needs to be atomic. We are not supposed to migrate a process when
  it is in the middile of an IO operation. In a noral OS, each IO goes through
  OS. Hence, there won't be any migration during the IO.
A: We will maintain one single semaphore in the PCB that gets blocked during
  any IO.
TODO: Write the migrating process long and simple

TODO: use IO semaphore while doing IO operation

TODO: Add a switch to set the CAN transmit and receive ids based on which
thread is pressed.
*/


/*
//Reverse the transmit and receive ids
void pf0_swap_can_ids(void){
  uint32_t tmp_id = CAN_RCV_ID;
  CAN_RCV_ID = CAN_XMT_ID;
  CAN_XMT_ID = tmp_id; 

  return;
}

int prepare_for_migration(int pcb_id){
  pcbType pcb = OS_get_pcb(pcb_id);

  // wait until the semaphore for IO is signalled.
  OS_Wait(&pcb.io_sema);
  
  // for threads in the PCB, set the run flag to 0.
  OS_mark_for_migration(pcb_id);

  // setup the can
  pf0_swap_can_ids();
  CAN0_Open(CAN_RCV_ID,CAN_XMT_ID);

  return 0;
}

//NOTE: the PCB of the migrated task is different from that of the previous
tasks.
//NOTE: Also the TCBs of each of the tasks are also different.
int end_migration(int PCB){

  return 0;
}


// return non zero on error
int send_process(void){

  // Send the PCB
  CAN0_SendMessage(PCB);

  for(int i = 0; i < PCB.threads; i++){
    // Send the TCBs that belong to the process
    CAN0_SendMessage(get_tcb(i));

    // Send the stacks
    CAN0_SendMessage(get_stack(i));
  }

  // Send the data heap 
  CAN0_SendMessage(data_heap);

  // Send the instruction heap
  CAN0_SendMessage(instruction_heap);

  ST7735_Message(0, 7, "DataLost: ", CAN_DataLost); 

  return 0; 
}

// return 0 on success
int receive_process(void){
  // Send the PCB
  pcb = CAN_Fifo_Get();    // IR right

  for(int i = 0; i < PCB.threads; i++){
    // Allocate a new thread
    new_thread = get_free_thread();

    // Overwrie the PCB to which the new thread belongs
    overwrite_the_pcb(new_thread, pcb);

    // Overwrite the data with the received data

    // Send the TCBs that belong to the process
    put_tcb(new_thread) = CAN_Fifo_Get();

    // Send the stacks
    put_stack(new_thread) = CAN_Fifo_Get();
  }

  // send the size of heap
  new_data_heap_size = CAN_Fifo_Get();
  malloc(new_data_heap_size);
  // Send the data heap 
  data_heap = CAN_Fifo_Get();

  new_instruction_heap_size = CAN_Fifo_Get();
  malloc(new_instruction_heap_size);
  // Send the instruction heap
  instruction_heap = CAN_Fifo_Get();

  return 0;
}

int end_migration(int pcb){
  // set all the tasks to running states
}
*/

uint32_t Errors = 0;
// helper function delays until sw2 is pressed and released, toggling the LED
void waitforsw2touch(void){
  // wait for SW2 pressed
  while((GPIO_PORTF_DATA_R&0x01) == 0x01){
    if(Errors == 0){
      GPIO_PORTF_DATA_R |= 0x08;
      Clock_Delay(1333333);        // wait ~0.25 sec
      GPIO_PORTF_DATA_R &= ~0x08;
      Clock_Delay(1333333);        // wait ~0.25 sec
    } else{
      GPIO_PORTF_DATA_R |= 0x02;
      Clock_Delay(1333333);        // wait ~0.25 sec
      GPIO_PORTF_DATA_R &= ~0x02;
      Clock_Delay(1333333);        // wait ~0.25 sec
    }
  }
  // wait for SW2 released
  while((GPIO_PORTF_DATA_R&0x01) == 0x00){};
}

#define TEXT_SIZE 1016
#define DATA_SIZE 120

extern int32_t Stacks[NUMTHREADS][STACKSIZE];  // threads with id = n will have stack at position n
Sema4Type mgrt_pr_lncd_sema; 

void launch_migrating_process(void);

void send_data(void) {
  tcbType* tcb_walker_ptr;

  
  // to synchronize and launch the migrating process
  OS_InitSemaphore(&mgrt_pr_lncd_sema, 0);
  NumCreated += OS_AddThread(&launch_migrating_process,128,0); 
  OS_Wait(&mgrt_pr_lncd_sema);

  waitforsw2touch();

  PF1 ^= 0x02;

  pcbType* pcb_ptr = OS_get_pcb_by_fl_name("root");

  OS_mark_for_migration(pcb_ptr->id);

  CAN0_SendMessage(sizeof(*pcb_ptr), (uint8_t *)pcb_ptr);

  OS_tcb_init_walker(&tcb_walker_ptr, pcb_ptr->id);
  //OS_Sleep(1);

  // replace text with some known sequence
  /*
  for(int i = 0; i < TEXT_SIZE; i++){
    void* txt = tcb_walker_ptr->pcb->text;
    *((uint8_t *)txt + i) = i;
  }
  */
  
  if(tcb_walker_ptr != NULL){
    CAN0_SendMessage(TEXT_SIZE, (uint8_t *)(tcb_walker_ptr->pcb->text));
    //OS_Sleep(1);
    CAN0_SendMessage(DATA_SIZE, (uint8_t *)(tcb_walker_ptr->pcb->data));
    //OS_Sleep(1);
  }

  while(tcb_walker_ptr !=  NULL){
    uint32_t rel_sp = (uint32_t )(tcb_walker_ptr->sp - Stacks[tcb_walker_ptr->id]);
    //send the relative stack position
    CAN0_SendMessage(sizeof(rel_sp), (uint8_t *)(&rel_sp));
    //OS_Sleep(1);


    CAN0_SendMessage(sizeof(*tcb_walker_ptr), (uint8_t *)tcb_walker_ptr);
    //OS_Sleep(1);

    // send the stack
    CAN0_SendMessage(sizeof(Stacks[tcb_walker_ptr->id]),
                      (uint8_t *)(Stacks[tcb_walker_ptr->id]));

    //OS_Sleep(1);

    OS_tcb_walk(&tcb_walker_ptr, pcb_ptr->id);
  }
  
  PF1 ^= 0x02;
  //CAN0_SendData(sending_data);
  //TODO: Kill the migrated threads.

  Clock_Delay(13333333);        // wait 2.5 sec


  OS_Kill();
}

int32_t stack[STACKSIZE];

extern tcbType* RunPt;

void receive_data(void) {

  tcbType tcb;

  //PF1 ^= 0x02;
  // allocate a pcb
  pcbType* pcb_ptr = GetFreeProcess();
  //TODO: fill pcb with meaningful data

  // get the pcb
  uint32_t size = CAN0_ReceiveSize(); 

  CAN0_ReceiveMessage(size, (uint8_t*) pcb_ptr);
  // modify the pcb id if required

  //get the text
  size = CAN0_ReceiveSize(); 
  uint8_t* text = (uint8_t *)Heap_Malloc(size); // 8 bytes for meta data
  memset((void*)text, 0, size);

  CAN0_ReceiveMessage(size, text);

  pcb_ptr->text = text;

  // get the data
  size = CAN0_ReceiveSize(); 
  void* data = (void *)Heap_Malloc(size); // 8 bytes for the meta data
  memset((void*)data, 0, size);

  CAN0_ReceiveMessage(size, (uint8_t*)data);

  pcb_ptr->data = data;

  uint32_t rel_sp;


  // we don't need to block the threads since they are not added to the run
  // queue.
  for(int i = 0; i < pcb_ptr->threads; i++) {

    // get the relative stack position
    size = CAN0_ReceiveSize(); 
    CAN0_ReceiveMessage(size, (uint8_t*) (&rel_sp));

    // get the tcb
    size = CAN0_ReceiveSize(); 

    CAN0_ReceiveMessage(size, (uint8_t*)(&tcb));

    // copy the relevant information to the tcb_ptr
    tcbType* tcb_ptr = GetFreeThread();
    tcb_ptr->pcb = pcb_ptr;
    tcb_ptr->priority = tcb.priority; 
    tcb_ptr->sleep = tcb.sleep; // this may not be accurate
    tcb_ptr->blockedTime = tcb.blockedTime;
    tcb_ptr->sp = Stacks[tcb_ptr->id] + rel_sp;

    *(Stacks[tcb_ptr->id] + rel_sp - 2) = (int32_t)(&RunPt); //Set the RunPt

    tcb_ptr->is_migrating = tcb.is_migrating; // we don't want the thread to run yet

    // get the stack
    size = CAN0_ReceiveSize(); 
    CAN0_ReceiveMessage(size, (uint8_t*)(Stacks[tcb_ptr->id]));

    OS_add_migrated_thread(tcb_ptr);

    //Set R9 to point to correct data
    Stacks[tcb_ptr->id][STACKSIZE - 11] = (int32_t)(pcb_ptr->data);  // R9

    // for debugging
    int p = 0;
  }

  int32_t status = StartCritical();

  tcbType* tcb_ptr = &tcb;

  //clear the is_migrating flag 
  OS_tcb_init_walker(&tcb_ptr, pcb_ptr->id);
  while(tcb_ptr != NULL){
    tcb_ptr->is_migrating = 0;

    OS_tcb_walk(&tcb_ptr, pcb_ptr->id);
  }


  EndCritical(status);

  OS_Kill();
}

//************SW1Push*************
// Called when SW1 Button pushed
// switch the can IDs
// background threads execute once and return
void SW1Push(void) {
  CAN0_Open(XMT_ID, RCV_ID);
  if(OS_AddThread(&send_data,128,1)){
    NumCreated++; 
  }

  return;
}

extern const ELFSymbol_t symtab[];
void launch_migrating_process(void){

	ELFEnv_t env = { symtab, 1};

  if(!eFile_Mount()) {
    //UART_OutString("Mount Completed!\n\r");
  }
  else {
    //UART_OutString("Error\n\r");
  }

  if (!exec_elf("USER.AXF", &env)) {
    //UART_OutString("Error launching process\r\n");
  }

  OS_Signal(&mgrt_pr_lncd_sema);

  OS_Kill();
} 

int realmain_can_receive(void) {
  LaunchPad_Init();
  OS_Init();        // initialize, disable interrupts
  PortD_Init();     // debugging profile
  MaxJitter1 = 0;    // in 1us units

  // for sending the data
  OS_MailBox_Init();

	// Initialize CAN FIFO
	CAN_Fifo_Init(CAN_MAXFIFOSIZE); 

	// Initialize CAN with given IDs
  //CAN0_Open(XMT_ID, RCV_ID);
  CAN0_Open(RCV_ID, XMT_ID);

  // hardware init
  ADC_Init(0);  // sequencer 3, channel 0, PE3, sampling in Interpreter

  // Heap_Init();  // heap initialized inside OS_Launch() for our implementation
  
  // attach background tasks
  OS_AddPeriodicThread(&disk_timerproc,TIME_1MS,0);   // time out routines for disk  
  OS_AddPeriodicThread(&can_timerproc,126 * TIME_1US,0);   // time out routines for can
  OS_AddSW1Task(&SW1Push,2);
  //OS_AddSW2Task(&SW2Push,2);  

  // create initial foreground threads
  NumCreated = 0;
  //NumCreated += OS_AddThread(&Interpreter,128,3); 
  NumCreated += OS_AddThread(&receive_data,128,2); 
  NumCreated += OS_AddThread(&Idle,128,5);  // at lowest priority 
 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

int realmain_can_send(void) {
  LaunchPad_Init();
  OS_Init();        // initialize, disable interrupts
  PortD_Init();     // debugging profile
  MaxJitter1 = 0;    // in 1us units

	// Initialize CAN with given IDs
  //CAN0_Open(RCV_ID,XMT_ID);
  CAN0_Open(XMT_ID,  // rcv id
            RCV_ID);  // transmit id

	// Initialize CAN FIFO
	CAN_Fifo_Init(CAN_MAXFIFOSIZE);
	
  // hardware init
  ADC_Init(0);  // sequencer 3, channel 0, PE3, sampling in Interpreter

  // Heap_Init();  // heap initialized inside OS_Launch() for our implementation
  
  // attach background tasks
  OS_AddPeriodicThread(&disk_timerproc,TIME_1MS,0);   // time out routines for disk  
  OS_AddPeriodicThread(&can_timerproc,10 * TIME_1US,0);   // time out routines for can
  //OS_AddSW1Task(&SW1Push,2);
  //OS_AddSW2Task(&SW2Push,2);  

  // create initial foreground threads
  NumCreated = 0;
  //NumCreated += OS_AddThread(&Interpreter,128,2); 
  NumCreated += OS_AddThread(&send_data,128,2); 
  NumCreated += OS_AddThread(&Idle,128,5);  // at lowest priority 
 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}




//*******************Trampoline for selecting main to execute**********
int main(void) { 			// main
  //realmain();
  //Testmain3();
  //realmain_can_send();
  realmain_can_receive();
  //realmain_text_send();
  //realmain_text_receive();
}
