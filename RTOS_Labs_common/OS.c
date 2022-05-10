// *************os.c**************
// EE445M/EE380L.6 Labs 1, 2, 3, and 4 
// High-level OS functions
// Students will implement these functions as part of Lab
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano 
// Jan 12, 2020, valvano@mail.utexas.edu


#include <stdint.h>
#include <stdio.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/CortexM.h"
#include "../inc/PLL.h"
#include "../inc/LaunchPad.h"
#include "../inc/BSP.h"
#include "../inc/Timer1A.h"
#include "../inc/Timer2A.h"
#include "../inc/Timer3A.h"
#include "../inc/Timer4A.h"
#include "../inc/Timer5A.h"
#include "../inc/WTimer0A.h"
#include "../inc/ADCT0ATrigger.h"
#include "../inc/EdgeInterruptPortF.h"
#include "../inc/LED.h"
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/ST7735.h"
#include "../RTOS_Labs_common/UART0int.h"
#include "../RTOS_Labs_common/eFile.h"
#include "../RTOS_Labs_common/heap.h"
#include "../RTOS_Labs_common/can_project.h"


#include <string.h>

// OS component definitions
#define JITTERSIZE1 64
#define JITTERSIZE2 64
//#define MAXFIFOSIZE 2048      // maximum size of FIFO buffer
#define MAXFIFOSIZE 32      // maximum size of FIFO buffer
#define PF1       (*((volatile uint32_t *)0x40025008))	
#define PF2       (*((volatile uint32_t *)0x40025010))	
#define PF3       (*((volatile uint32_t *)0x40025020))


// OS component declarations
uint32_t OSTimeMs = 0;                  // system time in msec
tcbType tcbs[NUMTHREADS];				        // statically allocating memory for tcbs
pcbType pcbs[NUMPROCESS];               // statically allocating memor for pcbs
tcbType *RunPt = 0;                     // pointer to the currently runing active thread
tcbType *FreePt = 0;					          // pointer to the next available free thread
int32_t Stacks[NUMTHREADS][STACKSIZE];  // threads with id = n will have stack at position n
static uint8_t PeriodicThreads = 1;     // number of periodic tasks based on hardware timer
uint32_t FIFOSize;               // size of FIFO from application
uint32_t volatile *FIFOPutPt;
uint32_t volatile *FIFOGetPt;
uint32_t FIFO[MAXFIFOSIZE];
Sema4Type FIFOCurrentSize;              // 0 if empty, FIFOSize if full
//uint32_t MailboxData;
uint8_t MailboxData[CAN_FRAME_SIZE];
Sema4Type MailboxFree;                  // 1 if free, 0 if not
//Sema4Type MailboxDataValid;             // 1 if data available, 0 if no data
uint8_t MailboxDataValid = 0;


// Performance Measurements
uint32_t NumKilled;
int32_t MaxJitter1 = 0;             // largest time jitter between interrupts in usec
uint32_t const JitterSize1=JITTERSIZE1;
uint32_t JitterHistogram1[JITTERSIZE1]={0,};
int32_t MaxJitter2 = 0;             // largest time jitter between interrupts in usec
uint32_t const JitterSize2=JITTERSIZE2;
uint32_t JitterHistogram2[JITTERSIZE2]={0,};
uint32_t CPUUtil;                   // calculated CPU utilization (in 0.01%)

// let the user take care of it
static Sema4Type  pcb_sem;
static Sema4Type  tcb_sem;

static uint32_t pcb_curr_traverse_id = 0;
static uint32_t tcb_curr_traverse_id = 0;

// External handlers for button switches
void (*SW1_Task)(void);        // application function executed by button interrupt handler
void (*SW2_Task)(void);        // application function executed by button interrupt handler


//******** Scheduler *************** 
// Handle logic for running threads, sleeping, blocking.
// Adjusts RunPt accordingly.
// Input:  none
// Output: none
void Scheduler(void) {
  tcbType *pt;
  tcbType *bestPt;
  uint16_t bestPriority = 256;

  RunPt = RunPt->next;    // move at least one

  // Search for highest priority thread not blocked or sleeping
  pt = bestPt = RunPt;
  do {
    if(pt->is_migrating) {// don't schdule processes marked for migration
      pt = pt->next;
      continue;
    }

    if (pt->blocked == NULL && pt->priority < bestPriority && pt->sleep <= OSTimeMs) {
      bestPriority = pt->priority;
      bestPt = pt;
    }
    pt = pt->next;
  } while (pt != RunPt);  // look through a complete loop

  RunPt = bestPt;     // update RunPt with next thread
};

//******** OS_LockScheduler *************** 
// Temporarily prevent foreground thread switch (but allow background interrupts).
// Inputs:  none
// Outputs: none
unsigned long OS_LockScheduler(void){
  unsigned long old = NVIC_ST_CTRL_R; 
  NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE+NVIC_ST_CTRL_CLK_SRC; 
  return old;
};

//******** OS_LockScheduler *************** 
// Resume foreground thread switching.
// Inputs:  previous
// Outputs: none
void OS_UnLockScheduler(unsigned long previous){
  NVIC_ST_CTRL_R = previous;
};

//******** SysTick_Init ***************
// Initialize Systick timer.
// Inputs:  interrupt period
// Outputs: none
void SysTick_Init(unsigned long period){
  STCTRL = 0; // disable SysTick during setup
  STCURRENT = 0; // any write to current clears it
  SYSPRI3 =(SYSPRI3&0x00FFFFFF)|0xC0000000; // priority 7
  STRELOAD = period - 1; // reload value
  STCTRL = 0x00000007; // enable, core clock and interrupt arm
};

//******** SysTick_Handler *************** 
// Systick Interrupt Handler
// SysTick interrupt happens every 10 ms
// used for preemptive thread switch 
// Inputs:  none
// Outputs: none
void SysTick_Handler(void) {
	ContextSwitch();
};

//******** InitTCBPool ***************
// Initialize pool of TCBs. Adjust id and pointers.
// Inputs:  none
// Outputs: none
void InitTCBPool(void) {
  // Initialize first element
  tcbs[0].sp = &Stacks[0][0];
  tcbs[0].next = &tcbs[1];
  tcbs[0].previous = NULL;
  tcbs[0].id = 0;
  tcbs[0].blockedTime = 0xFFFFFFFF;
  tcbs[0].pcb = NULL;
  tcbs[0].is_migrating = 0;


  // Initialize all middle elements
  for (int i = 1; i < NUMTHREADS - 1; i++) {
    tcbs[i].sp = &Stacks[i][0];
    tcbs[i].next = &tcbs[i + 1];
    tcbs[i].previous = &tcbs[i - 1];
    tcbs[i].id = i;
    tcbs[i].blockedTime = 0xFFFFFFFF;
    tcbs[i].pcb = NULL;
    tcbs[i].is_migrating = 0;
  }

  // Initialize last element
  tcbs[NUMTHREADS - 1].sp = &Stacks[NUMTHREADS - 1][0];
  tcbs[NUMTHREADS - 1].next = NULL;
  tcbs[NUMTHREADS - 1].previous = &tcbs[NUMTHREADS - 2];
  tcbs[NUMTHREADS - 1].id = NUMTHREADS - 1;
  tcbs[NUMTHREADS - 1].blockedTime = 0xFFFFFFFF;
  tcbs[NUMTHREADS - 1].pcb = NULL;
  tcbs[NUMTHREADS - 1].is_migrating = 0;

  // All tcbs are free at the beginning
  FreePt = &tcbs[NUMTHREADS - 1];
};

void InitPCBPool(void) {
  for (int i = 0; i < NUMPROCESS; i++) 
    pcbs[i].id = -1;

  return;
}

/*----------------------------------------------------------------------------
  PortF Initialization (LED)
 *----------------------------------------------------------------------------*/
void PortF_Init(void){ 
  SYSCTL_RCGCGPIO_R |= 0x20;       // activate port F
  while((SYSCTL_RCGCGPIO_R&0x20)==0){};
  GPIO_PORTF_DIR_R |= 0x0E;        // make PF3-1 output heartbeats
  GPIO_PORTF_AFSEL_R &= ~0x0E;     // disable alt funct on PF3-0
  GPIO_PORTF_DEN_R |= 0x0E;        // enable digital I/O on PF3-0
}

// ******** OS_Init ************
// Initialize operating system, disable interrupts until OS_Launch.
// Initialize OS controlled I/O: serial, ADC, systick, LaunchPad I/O and timers. 
// Interrupts not yet enabled.
// input:  none
// output: none
void OS_Init(void){
  DisableInterrupts();    // interrupts enabled later in StartOS()
  PLL_Init(Bus80MHz);
  ST7735_InitR(INITR_REDTAB); // LCD initialization
  PortF_Init();
  UART_Init();  // serial I/O for interpreter
  OS_ClearMsTime(); // start a periodic interrupt to maintain time
  InitTCBPool();    // initialize pool of TCBs
  InitPCBPool();    // initialize pool of PCBs

  // note that  there could only be one traverser at a time
  // be careful with this
  OS_InitSemaphore(&pcb_sem, 1);
  OS_InitSemaphore(&tcb_sem, 1);
};

//******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 12.5ns clock cycles for each time slice
//         you may select the units of this parameter
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field
// It is ok to limit the range of theTimeSlice to match the 24-bit SysTick
void OS_Launch(uint32_t theTimeSlice){
	SYSPRI3 = SYSPRI3 ^= 0x00E00000; // Configure PendSV interrupt (priority 6)
  SysTick_Init(theTimeSlice);      // Initialize Configure SysTick interrupt (priority 7)
  Heap_Init();                     // Initialize the heap
  StartOS();
};

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, int32_t value){
  semaPt->value = value;
};

// ******** OS_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt){
  DisableInterrupts();

  semaPt->value = semaPt->value - 1;  // decrement semaphore

  // Check semaphore state
  if (semaPt->value < 0) {
    RunPt->blocked = semaPt;  // block on this semaphore
    RunPt->blockedTime = (1000 * OSTimeMs) + (OS_Time() / 80);  // blocked at this time (bounded waiting)
    EnableInterrupts();
    OS_Suspend();
  }

  EnableInterrupts();
}; 

// ******** OS_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt){
  tcbType *pt;
  tcbType *bestPt;
  uint16_t bestPriority = 256;    // 1 below lowest priority
  uint32_t bestTime = 0xFFFFFFFF; // latest time possible
  DisableInterrupts();

  semaPt->value = semaPt->value + 1;  // increment semaphore

  if (semaPt->value <= 0) {
    // Search for highest priority thread blocked on this semaphore or longest waiting
    pt = RunPt;
    bestPt = NULL;
    do {
      if(pt->is_migrating){ // don't consider migrating threads
        pt = pt->next;    // move at least one
        continue;
      }

      if (pt->blocked == semaPt 
          && (pt->priority < bestPriority 
                || (pt->priority == bestPriority 
                      && pt->blockedTime < bestTime))) {

        bestPriority = pt->priority;
        bestTime = pt->blockedTime;
        bestPt = pt;
      }
      pt = pt->next;    // move at least one
    } while (pt != RunPt);  // look through a complete loop

    if (bestPt != NULL) {
      bestPt->blocked = NULL;   // wake up
    }

    // Context switch if higher priority thread woken up
    if (bestPriority < RunPt->priority) {
      EnableInterrupts();
      OS_Suspend();
    }
  }

  EnableInterrupts();
}; 

// ******** OS_bWait ************
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt){
  DisableInterrupts();

  // Check semaphore state
  if (semaPt->value == 0) {
    RunPt->blocked = semaPt;  // block on this semaphore
    RunPt->blockedTime = (1000 * OSTimeMs) + (OS_Time() / 80);  // blocked at this time (bounded waiting)
    EnableInterrupts();
    OS_Suspend();
  }
  else {
    semaPt->value = 0;    // acquire sempahore
  }

  EnableInterrupts();
}; 

// ******** OS_bSignal ************
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4Type *semaPt){
  tcbType *pt;
  tcbType *bestPt;
  uint16_t bestPriority = 256;    // 1 below lowest priority
  uint32_t bestTime = 0xFFFFFFFF; // latest time possible
  DisableInterrupts();

  if (semaPt->value == 0) {
    // Search for highest priority thread blocked on this semaphore or longest waiting
    pt = RunPt;
    bestPt = NULL;
    do {
      if (pt->blocked == semaPt && (pt->priority < bestPriority || (pt->priority == bestPriority && pt->blockedTime < bestTime))) {
        bestPriority = pt->priority;
        bestTime = pt->blockedTime;
        bestPt = pt;
      }
      pt = pt->next;    // move at least one
    } while (pt != RunPt);  // look through a complete loop

    if (bestPt != NULL) {
      bestPt->blocked = NULL;   // wake up
      // Context switch if higher priority thread woken up
      if (bestPriority < RunPt->priority) {
        EnableInterrupts();
        OS_Suspend();
      }
    }
    else {
      semaPt->value = 1;    // increment semaphore
    }
  }

  EnableInterrupts();
}; 

//******** GetFreeThread *************** 
// Helper function for OS_AddThread(). Obtain a free thread.
// Inputs:  none
// Outputs: pointer to free tcb
tcbType* GetFreeThread(void){
  if (FreePt == NULL)   // no free threads available
    return NULL;

  tcbType *newThreadPt = FreePt;  // next element in the pool
  FreePt = FreePt->previous;      // update FreePt to next free thread

  return newThreadPt;
};

//******** SetInitialStack *************** 
// Helper function to initialize stack for new threads.
// Inputs:  thread id
// Outputs: none
void SetInitialStack(int i) {
  tcbs[i].sp = &Stacks[i][STACKSIZE - 16];  // thread stack pointer
  Stacks[i][STACKSIZE-1] = 0x01000000; // Thumb bit
  Stacks[i][STACKSIZE-3] = 0x14141414; // R14 (LR)
  Stacks[i][STACKSIZE-4] = 0x12121212; // R12
  Stacks[i][STACKSIZE-5] = 0x03030303; // R3
  Stacks[i][STACKSIZE-6] = 0x02020202; // R2
  Stacks[i][STACKSIZE-7] = 0x01010101; // R1
  Stacks[i][STACKSIZE-8] = 0x00000000; // R0
  Stacks[i][STACKSIZE-9] = 0x11111111; // R11
  Stacks[i][STACKSIZE-10] = 0x10101010; // R10
  Stacks[i][STACKSIZE-11] = 0x09090909; // R9
  Stacks[i][STACKSIZE-12] = 0x08080808; // R8
  Stacks[i][STACKSIZE-13] = 0x07070707; // R7
  Stacks[i][STACKSIZE-14] = 0x06060606; // R6
  Stacks[i][STACKSIZE-15] = 0x05050505; // R5
  Stacks[i][STACKSIZE-16] = 0x04040404; // R4
};

//******** OS_AddThread *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
int OS_AddThread(void(*task)(void), 
    uint32_t stackSize, uint32_t priority){
  int32_t status;
  
  // Obtain free thread from the pool
  status = StartCritical();
  tcbType *newThreadPt = GetFreeThread();
  if (newThreadPt == NULL) {  // no free threads available
    EndCritical(status);
    return 0;
  }
  EndCritical(status);

  // Initialize stack
  SetInitialStack(newThreadPt->id);
  Stacks[newThreadPt->id][STACKSIZE - 2] = (int32_t)(task);	  // PC

  // Configure new thread
  newThreadPt->sp = &Stacks[newThreadPt->id][STACKSIZE - 16]; // set sp parameter in TCB
  newThreadPt->priority = priority;   // set priority parameter in TCB
  newThreadPt->sleep = 0;             // set sleep parameter in TCB
  newThreadPt->blocked = NULL;        // set blocked parameter in TCB

  // Initialize/Update Round-Robin circular list
  status = StartCritical();
  if (RunPt == 0) {   // first and only thread
    newThreadPt->next = newThreadPt;
    newThreadPt->previous = newThreadPt;
    RunPt = newThreadPt;
  }
  else {
    // Update next pointers
    newThreadPt->next = RunPt->next;
    RunPt->next = newThreadPt;
    // Update previous pointers
    newThreadPt->previous = RunPt;
    newThreadPt->next->previous = newThreadPt;
  }
  EndCritical(status);

  return 1;   // success
};

//******** OS_AddThreadP *************** 
// add a modified foregound thread (with a parent process) to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
//         pointer to parent process control block
//         pointer to process data section in memory
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
int OS_AddThreadP(void(*task)(void), 
    uint32_t stackSize, uint32_t priority, pcbType *pcb){
  int32_t status;
  
  // Obtain free thread from the pool
  status = StartCritical();
  tcbType *newThreadPt = GetFreeThread();
  if (newThreadPt == NULL) {  // no free threads available
    EndCritical(status);
    return 0;
  }
  EndCritical(status);

  // Initialize stack
  SetInitialStack(newThreadPt->id);
  Stacks[newThreadPt->id][STACKSIZE - 2] = (int32_t)(task);   // PC
  Stacks[newThreadPt->id][STACKSIZE - 11] = (int32_t)(pcb->data);  // R9

  // Configure new thread
  newThreadPt->sp = &Stacks[newThreadPt->id][STACKSIZE - 16]; // set sp parameter in TCB
  newThreadPt->priority = priority;   // set priority parameter in TCB
  newThreadPt->sleep = 0;             // set sleep parameter in TCB
  newThreadPt->blocked = NULL;        // set blocked parameter in TCB
  newThreadPt->pcb = pcb;             // set parent process parameter in TCB

  // Update parent process
  status = StartCritical();
  newThreadPt->pcb->threads += 1;       // increase active thread count

  // Initialize/Update Round-Robin circular list
  if (RunPt == 0) {   // first and only thread
    newThreadPt->next = newThreadPt;
    newThreadPt->previous = newThreadPt;
    RunPt = newThreadPt;
  }
  else {
    // Update next pointers
    newThreadPt->next = RunPt->next;
    RunPt->next = newThreadPt;
    // Update previous pointers
    newThreadPt->previous = RunPt;
    newThreadPt->next->previous = newThreadPt;
  }
  EndCritical(status);

  return 1;
}

//******** GetFreeProcess *************** 
// Helper function for OS_AddProcess(). Obtain a free process.
// Inputs:  none
// Outputs: pointer to free tcb
pcbType* GetFreeProcess(void){
  // Loop through pool
  int32_t status = StartCritical();
  for (int i = 0; i < NUMPROCESS; i++) {
    if (pcbs[i].threads == 0) {
      EndCritical(status);
      return &pcbs[i];      // free process
    }
  }

  EndCritical(status);
  return NULL;    // no free process available
}

//******** OS_AddProcess *************** 
// add a process with foregound thread to the scheduler
// Inputs: pointer to a void/void entry point
//         pointer to process text (code) segment
//         pointer to process data segment
//         number of bytes allocated for its stack
//         priority (0 is highest)
// Outputs: 1 if successful, 0 if this process can not be added
// This function will be needed for Lab 5
// In Labs 2-4, this function can be ignored
/*
int OS_AddProcess(void(*entry)(void), void *text, void *data, 
  unsigned long stackSize, unsigned long priority,
  char fl_name[10]){
  */
int OS_AddProcess(void(*entry)(void), void *text, void *data, 
  unsigned long stackSize, unsigned long priority){

  int32_t status = StartCritical();

  // Obtain free process from the pool
  pcbType *newProcessPt = GetFreeProcess();
  if (newProcessPt == NULL) {   // no free process available
    EndCritical(status);
    return 0;
  }

  // Configure new process
  newProcessPt->threads = 0;
  newProcessPt->text = text;
  newProcessPt->data = data;
  newProcessPt->id = OS_get_pcb_id(newProcessPt); 
  OS_InitSemaphore(&newProcessPt->io_sema, 1);
  memset(newProcessPt->fl_name, 0, OS_PCB_FL_NAME_LEN);
  strcpy(newProcessPt->fl_name, "root");

  // Add initial thread
  if (!OS_AddThreadP(entry, stackSize, priority, newProcessPt)) {
    newProcessPt->threads = 0;
    newProcessPt->text = NULL;
    newProcessPt->data = NULL;
    newProcessPt->id = -1;
    newProcessPt->io_sema.value = 0;
    memset(newProcessPt->fl_name, 0, OS_PCB_FL_NAME_LEN);
    return 0;   // error
  }

  return 1;
};


//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
uint32_t OS_Id(void){
  return RunPt->id;
};


//******** OS_AddPeriodicThread *************** 
// add a background periodic task
// typically this function receives the highest priority
// Inputs: pointer to a void/void background function
//         period given in system time units (12.5ns)
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// You are free to select the time resolution for this function
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal   OS_AddThread
// This task does not have a Thread ID
// In lab 1, this command will be called 1 time
// In lab 2, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, this command will be called 0 1 or 2 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddPeriodicThread(void(*task)(void), 
   uint32_t period, uint32_t priority){
  int32_t status = StartCritical();

  // There are 4 hardware timers available (1 - 4). Timer 0 is used by ADC, Timer 5 is used by OS.
  switch (PeriodicThreads) {
    case 1:
      Timer1A_Init(task, period, priority);
      break;
    case 2:
      Timer2A_Init(task, period, priority);
      break;
    case 3:
      Timer3A_Init(task, period, priority);
      break;
    case 4:
      Timer4A_Init(task, period, priority);
      break;
    default:
      EndCritical(status);
      return 0;
  }
  PeriodicThreads++;    // increase periodic threads count for next periodic thread
  
  EndCritical(status);

  return 1;   // success
};


/*----------------------------------------------------------------------------
  PF1 Interrupt Handler
 *----------------------------------------------------------------------------*/
void GPIOPortF_Handler(void){
  if(GPIO_PORTF_RIS_R&0x10){  // SW1 touch
    GPIO_PORTF_ICR_R = 0x10;  // acknowledge flag4
    // LED_RedToggle();         // profile
    SW1_Task();
  }

  if(GPIO_PORTF_RIS_R&0x01){  // SW2 touch
    GPIO_PORTF_ICR_R = 0x01;  // acknowledge flag0
    // LED_BlueToggle();         // profile
    SW2_Task();
  }
};

//******** OS_AddSW1Task *************** 
// add a background task to run whenever the SW1 (PF4) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal   OS_AddThread
// This task does not have a Thread ID
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW1Task(void(*task)(void), uint32_t priority){
  int32_t status = StartCritical();

  SYSCTL_RCGCGPIO_R |= 0x00000020;  // (a) activate clock for port F	
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F	

  GPIO_PORTF_CR_R |= 0x12;           // allow changes to PF4-0	
  GPIO_PORTF_DIR_R |=  0x02;        // output on PF3,2,1 	
  GPIO_PORTF_DIR_R &= ~0x10;        // (c) make PF4,0 in (built-in button)	
  GPIO_PORTF_AFSEL_R &= ~0x12;      //     disable alt funct on PF4,0	
  GPIO_PORTF_DEN_R |= 0x10;         //     enable digital I/O on PF4   	

  GPIO_PORTF_PCTL_R &= ~0x000FFFFF; // configure PF4 as GPIO	

  GPIO_PORTF_AMSEL_R = 0;           //     disable analog functionality on PF	
  GPIO_PORTF_PUR_R |= 0x10;         //     enable weak pull-up on PF4	

  GPIO_PORTF_IS_R &= ~0x10;         // (d) PF4 is edge-sensitive	
  GPIO_PORTF_IBE_R &= ~0x10;        //     PF4 is not both edges	
  GPIO_PORTF_IEV_R &= ~0x10;        //     PF4 falling edge event	

  GPIO_PORTF_ICR_R = 0x10;          // (e) clear flag4	
  GPIO_PORTF_IM_R |= 0x10;          // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***	
  // NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5	
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|(priority << 17); // (g) assign priority	

  NVIC_EN0_R = 0x40000000;          // (h) enable interrupt 30 in NVIC	

  SW1_Task = task;   // thread called by interrupt handler

  EndCritical(status);

  return 1;
};

//******** OS_AddSW2Task *************** 
// add a background task to run whenever the SW2 (PF0) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed user task will run to completion and return
// This task can not spin block loop sleep or kill
// This task can call issue OS_Signal, it can call OS_AddThread
// This task does not have a Thread ID
// In lab 2, this function can be ignored
// In lab 3, this command will be called will be called 0 or 1 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW2Task(void(*task)(void), uint32_t priority){
  int32_t status = StartCritical();
  SYSCTL_RCGCGPIO_R |= 0x00000020;  // (a) activate clock for port F	
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F	

  GPIO_PORTF_CR_R |= 0x01;           // allow changes to PF0	
  GPIO_PORTF_DIR_R |=  0x00;        // output on PF none
  GPIO_PORTF_DIR_R &= ~0x01;        // (c) make PF0 in (built-in button)	
  GPIO_PORTF_AFSEL_R &= ~0x01;      //     disable alt funct on PF4,0	
  GPIO_PORTF_DEN_R |= 0x01;         //     enable digital I/O on PF4   	

  GPIO_PORTF_PCTL_R &= ~0x000FFFFF; // configure PF1 as GPIO	

  GPIO_PORTF_AMSEL_R = 0;           //     disable analog functionality on PF	
  GPIO_PORTF_PUR_R |= 0x01;         //     enable weak pull-up on PF4	

  GPIO_PORTF_IS_R &= ~0x01;         // (d) PF1 is edge-sensitive	
  GPIO_PORTF_IBE_R &= ~0x01;        //     PF1 is not both edges	
  GPIO_PORTF_IEV_R &= ~0x01;        //     PF1 falling edge event	

  GPIO_PORTF_ICR_R = 0x01;          // (e) clear flag4	
  GPIO_PORTF_IM_R |= 0x01;          // (f) arm interrupt on PF1 *** No IME bit as mentioned in Book ***	
  //NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5	
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|(priority << 17); // (g) assign priority

  NVIC_EN0_R = 0x40000000;          // (h) enable interrupt 30 in NVIC	
  SW2_Task = task;   // thread called by interrupt handler

  EndCritical(status);

  return 1;
};

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(uint32_t sleepTime){
  RunPt->sleep = OSTimeMs + sleepTime;   // update sleep parameter in TCB
  OS_Suspend();  // suspend execution of current thread
};

// ******** PutFreeThread ************
// Helper function for OS_Kill(). Add free thread to pool.
// Inputs:  pointer to killed thread
// Outputs: none
void PutFreeThread(tcbType *currentPt) {	
  tcbType *nextPt = currentPt->next;  // next thread	
  tcbType *previousPt = currentPt->previous;  // previous thread		

  // Add current thread to free pool and adjust pointers accordingly
  if (FreePt == NULL) {
    currentPt->previous = NULL;
    FreePt = currentPt;
  }
  else {
    FreePt->next = currentPt;
    currentPt->previous = FreePt;
    FreePt = currentPt;
  } 
  
  // Adjust next and previous running threads	
  previousPt->next = nextPt;	
  nextPt->previous = previousPt;	
}

// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void){
  DisableInterrupts();

  // Check if thread from a  process
  if (RunPt->pcb != NULL) {
    RunPt->pcb->threads -= 1;
    if (RunPt->pcb->threads == 0) {
      // Add process to free pool
      RunPt->pcb->threads = 0;
      Heap_Free(RunPt->pcb->text);
      RunPt->pcb->text = NULL;
      Heap_Free(RunPt->pcb->data);
      RunPt->pcb->data = NULL;
      RunPt->pcb->id = -1;
      memset(RunPt->pcb->fl_name, 0, OS_PCB_FL_NAME_LEN);
    }
  }

  // Add thread to free pool
  PutFreeThread(RunPt);
  NumKilled++;

  OS_Suspend();         // perform a context switch

  // Check if SVC ISR
  if (SYSHNDCTRL&0x00000080) {
    return; // return to finalize interrupt
  }

  for(;;){};        // can not return
}; 

// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
void OS_Suspend(void){
  ContextSwitch();
  EnableInterrupts();     // Re-enable interrupts as OS_Kill() disabled them
};
  
// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
void OS_Fifo_Init(uint32_t size){
  FIFOSize = size;                 // size of FIFO from application
  FIFOPutPt = FIFOGetPt = &FIFO[0];    // initially empty
  OS_InitSemaphore(&FIFOCurrentSize, 0);
};

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(uint32_t data){
  if (FIFOCurrentSize.value == FIFOSize) {
    return 0;  // FIFO full, lost data
  }
  else {
    *FIFOPutPt = data;    // put data

    // Adjust put pointer
    FIFOPutPt++;
    if (FIFOPutPt == &FIFO[FIFOSize]) {
      FIFOPutPt = &FIFO[0];  // wrap around
    }

    OS_Signal(&FIFOCurrentSize);
  }

  return 1;   // success
};  

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
uint32_t OS_Fifo_Get(void){
  uint32_t data;

  OS_Wait(&FIFOCurrentSize);   // block if FIFO empty

  data = *FIFOGetPt;          // get data

  // Adjust get pointer
  FIFOGetPt++;
  if (FIFOGetPt == &FIFO[FIFOSize]) {
    FIFOGetPt = &FIFO[0];
  }

  return data;
};

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
int32_t OS_Fifo_Size(void){
  return FIFOCurrentSize.value;     // sempahore value is its current size
};


// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
void OS_MailBox_Init(void){
  OS_InitSemaphore(&MailboxFree, 1);
  //OS_InitSemaphore(&MailboxDataValid, 0);
};

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
//void OS_MailBox_Send(uint32_t data){
void OS_MailBox_Send(uint8_t data[CAN_FRAME_SIZE]){
  OS_bWait(&MailboxFree);         // wait for mailbox to be free

  int32_t status = StartCritical();
  for(int i = 0; i < CAN_FRAME_SIZE; i++){
    MailboxData[i] = data[i];
  }
  MailboxDataValid = 1;
  EndCritical(status);
  //OS_bSignal(&MailboxDataValid);  // signal data in mailbox
};

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
//void OS_MailBox_Recv(uint8_t * data [CAN_FRAME_SIZE]){
void OS_MailBox_Recv(uint8_t* data){
  //uint32_t data;
  //OS_bWait(&MailboxDataValid);    // wait for data in mailbox
  // we dont wait in a timer routine
  if(MailboxDataValid == 1){
    for(int i = 0; i < CAN_FRAME_SIZE; i++){
      //data = MailboxData;
      *(data + i) =  MailboxData[i];
    }
    OS_bSignal(&MailboxFree);       // signal mailbox is free
    MailboxDataValid = 0;
  }

  //return data;
};

// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
uint32_t OS_Time(void){
  return (80000 - 1 - TIMER5_TAV_R);
};

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
uint32_t OS_TimeDifference(uint32_t start, uint32_t stop){
  uint32_t result;
  if (stop < start) {
		result = 80000 + stop - start;
  }
  else if (stop > start) {
    result = stop - start;
  }
  else {
    result = 0;
  }
  return result;
};

// ******** OS_ClearMsTime ************
// Sets the system time to zero (solve for Lab 1), and start a periodic interrupt
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void){
  uint32_t period = 80000000 / 1000;     // period = 1 / clockfrequency
  uint32_t priority = 1;

  OSTimeMs = 0;    // set system time to zero

  // Start periodic interrupt
  Timer5A_Init(&OS_IncrementMsTime, period, priority);
};

// ******** OS_MsTime ************
// Reads the current time in msec (solve for Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// For Labs 2 and beyond, it is ok to make the resolution to match the first call to OS_AddPeriodicThread
uint32_t OS_MsTime(void){
  return OSTimeMs;
};

// ******** OS_IncrementMsTime ************
// Increments the system time by 1 msec
// Inputs:  none
// Outputs: none
void OS_IncrementMsTime(void){
  OSTimeMs++;
}

//************** I/O Redirection *************** 
// redirect terminal I/O to UART or file (Lab 4)

int StreamToDevice=0;                // 0=UART, 1=stream to file (Lab 4)

int fputc (int ch, FILE *f) { 
  if(StreamToDevice==1){  // Lab 4
    if(eFile_Write(ch)){          // close file on error
       OS_EndRedirectToFile(); // cannot write to file
       return 1;                  // failure
    }
    return 0; // success writing
  }
  
  // default UART output
  UART_OutChar(ch);
  return ch; 
}

int fgetc (FILE *f){
  char ch = UART_InChar();  // receive from keyboard
  UART_OutChar(ch);         // echo
  return ch;
}

int OS_RedirectToFile(const char *name){  // Lab 4
  eFile_Create(name);              // ignore error if file already exists
  if(eFile_WOpen(name)) return 1;  // cannot open file
  StreamToDevice = 1;
  return 0;
}

int OS_EndRedirectToFile(void){  // Lab 4
  StreamToDevice = 0;
  if(eFile_WClose()) return 1;    // cannot close file
  return 0;
}

int OS_RedirectToUART(void){
  StreamToDevice = 0;
  return 0;
}

int OS_RedirectToST7735(void){
  
  return 1;
}

// ******** OS_JitterSample ************
// Calculate jitter for periodic tasks.
// Inputs:  work    to ignore first sample
//          period  to calculate jitter
//          id      two separate jitters can be calculated
// Outputs: none
void OS_JitterSample(int work, uint32_t period, int id) {
  unsigned static long LastTime1, LastTimeMs1;  // time at previous ADC sample
  unsigned static long LastTime2, LastTimeMs2;  // time at previous ADC sample
  long jitter;                     // time between measured and expected, in us
  uint32_t thisTime = OS_Time();   // current time, 12.5 ns
  uint32_t thisTimeMs = OS_MsTime();  // current time, 1 ms

  if(work >= 1) {    // ignore timing of first interrupt
    if (id == 1) {          // Jitter1
      uint32_t diffMs = thisTimeMs - LastTimeMs1;
      uint32_t diff = 80000 * diffMs + thisTime - LastTime1;
      // if (diffMs >= 1) {
      //   diff = 80000 * diffMs + OS_TimeDifference(LastTime1, thisTime);
      // }
      // else {
      //   diff = OS_TimeDifference(LastTime1, thisTime);
      // }
      if(diff>period){
        jitter = (diff-period+4)/8;  // in 0.1 usec
      }else{
        jitter = (period-diff+4)/8;  // in 0.1 usec
      }
      if(jitter > MaxJitter1){
        MaxJitter1 = jitter; // in usec
      }       // jitter should be 0
      if(jitter >= JitterSize1){
        jitter = JitterSize1-1;
      }
      JitterHistogram1[jitter]++;
    }
    else if (id == 2) {     // Jitter2
      uint32_t diffMs = thisTimeMs - LastTimeMs2;
      uint32_t diff = 80000 * diffMs + thisTime - LastTime2;
      // if (diffMs >= 1) {
      //   diff = 80000 * diffMs + OS_TimeDifference(LastTime2, thisTime);
      // }
      // else {
      //   diff = OS_TimeDifference(LastTime2, thisTime);
      // }
      if(diff>period){
        jitter = (diff-period+4)/8;  // in 0.1 usec
      }else{
        jitter = (period-diff+4)/8;  // in 0.1 usec
      }
      if(jitter > MaxJitter2){
        MaxJitter2 = jitter; // in usec
      }       // jitter should be 0
      if(jitter >= JitterSize2){
        jitter = JitterSize2-1;
      }
      JitterHistogram2[jitter]++;
    }
  }

  // Update time for next call
  if (id == 1) {        // Jitter1
    LastTime1 = thisTime;
    LastTimeMs1 = thisTimeMs;
  }
  else if (id == 2) {   // Jitter2
    LastTime2 = thisTime;
    LastTimeMs2 = thisTimeMs;
  }
};

/*
Return the pcb given the pcb_id
*/
pcbType* OS_get_pcb(int pcb_id){
  if(pcb_id >= NUMPROCESS)
    return NULL;

  return &pcbs[pcb_id];
}

// return -1 on failure
int OS_get_pcb_id(pcbType* pcb){
  for(int pcb_id = 0; 
        pcb_id < NUMPROCESS; 
        pcb_id++){

    if((pcbs + pcb_id) == pcb)
      return pcb_id;
  }

  return -1;
}



// initilize the pcb_walker to the first process in the list
void OS_pcb_init_walker(pcbType** pcb_walker_ptr){

  OS_Wait(&pcb_sem);

  pcbType* pcb_ptr;

  for(pcb_curr_traverse_id = 0; 
      pcb_curr_traverse_id < NUMPROCESS; 
      pcb_curr_traverse_id++){

    pcb_ptr = &pcbs[pcb_curr_traverse_id];

    if(pcb_ptr->threads != 0){
      *pcb_walker_ptr = pcb_ptr;
      goto ret;
    }
  }

  *pcb_walker_ptr = NULL;
  pcb_curr_traverse_id = 0;
  OS_Signal(&pcb_sem);

ret:
  return;
}


// go to the next valid pcb and set the passed pointer to it
void OS_pcb_walk(pcbType** pcb_walker_ptr){

  //OS_Wait(&pcb_sem);

  pcbType* pcb_ptr;

  for(pcb_curr_traverse_id++; // not incremented when getting out of prev itr
      pcb_curr_traverse_id < NUMPROCESS; 
      pcb_curr_traverse_id++){

    pcb_ptr = &pcbs[ pcb_curr_traverse_id ];

    if(pcb_ptr->threads != 0){
      *pcb_walker_ptr = pcb_ptr;
      goto ret;
    }
  }

  *pcb_walker_ptr = NULL;
  pcb_curr_traverse_id= 0;
  OS_Signal(&pcb_sem);

ret:
  return;
}

pcbType* OS_get_pcb_by_fl_name(char fl_name[OS_PCB_FL_NAME_LEN]){

  pcbType* pcb_ptr;
  int pcb_trv = 0;
  for(; 
      pcb_trv < NUMPROCESS; 
      pcb_trv++){
    
    pcb_ptr = &pcbs[pcb_trv];

    if(!strcmp(fl_name, pcb_ptr->fl_name))
      return pcb_ptr;
  }


  return NULL;
}

// initilize the tcb_walker to the first process in the list
void OS_tcb_init_walker(tcbType** tcb_walker_ptr, int pcb_id){

  OS_Wait(&tcb_sem);

  tcbType* tcb_ptr;

  for(tcb_curr_traverse_id = 0; 
      tcb_curr_traverse_id < NUMTHREADS; 
      tcb_curr_traverse_id++){

    tcb_ptr = &tcbs[tcb_curr_traverse_id];

    if(tcb_ptr->pcb->id == pcb_id){
      *tcb_walker_ptr = tcb_ptr;
      goto ret;
    }
  }

  *tcb_walker_ptr = NULL;
  tcb_curr_traverse_id = 0;
  OS_Signal(&tcb_sem);

ret:
  return;
}


// go to the next valid tcb and set the passed pointer to it
void OS_tcb_walk(tcbType** tcb_walker_ptr, int pcb_id){

  tcbType* tcb_ptr;

  for(tcb_curr_traverse_id++; // not incremented when getting out of prev itr
      tcb_curr_traverse_id < NUMTHREADS; 
      tcb_curr_traverse_id++){

    tcb_ptr = &tcbs[ tcb_curr_traverse_id ];

    if(tcb_ptr->pcb->id == pcb_id){
      *tcb_walker_ptr = tcb_ptr;
      goto ret;
    }
  }

  *tcb_walker_ptr = NULL;
  tcb_curr_traverse_id= 0;
  OS_Signal(&tcb_sem);

ret:
  return;
}

int OS_mark_for_migration(int pcb_id){
  int i;

  int32_t status = StartCritical();
  for(i =0; 
      i < NUMTHREADS; 
      i++){

    if(tcbs[i].pcb->id == pcb_id){
      if(tcbs[i].blocked != NULL)
        goto unblock;


      tcbs[i].is_migrating = 1;
    }
  }

  EndCritical(status);
  return 0;

unblock:
  for(; 
      i >= 0; 
      i--){
    if(tcbs[i].pcb->id == pcb_id){
      tcbs[i].is_migrating = 0;
    }
  }

  EndCritical(status);
  return 1;
}

// ******** OS_mark_for_migration ************
// marks all the threads of a process to migrating state.
// the scheduler ignores these process while scheduling.
// Outputs: 0 for success
/*
int OS_mark_for_migration(int pcb_id){ 
  tcbType *pt;

  // loop through the all the tcbs
  pt = RunPt;
  do {
    if(pt->pcb->id == pcb_id){ // thread belogs to the required process
      pt->is_migrating = 1;
    }

    pt = pt->next;
  } while(pt != RunPt);

  return 0;
}

*/


void OS_add_migrated_thread(tcbType* newThreadPt){
  // code copied from OS_Add_Thread

  // don't let this stop in the middle
  int32_t status = StartCritical();

  if (RunPt == 0) {   // first and only thread
    newThreadPt->next = newThreadPt;
    newThreadPt->previous = newThreadPt;
    RunPt = newThreadPt;
  }
  else {
    // Update next pointers
    newThreadPt->next = RunPt->next;
    RunPt->next = newThreadPt;

    // Update previous pointers
    newThreadPt->previous = RunPt;
    newThreadPt->next->previous = newThreadPt;
  }
  EndCritical(status);

  return;
}
