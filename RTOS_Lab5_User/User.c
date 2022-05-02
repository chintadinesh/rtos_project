// User.c
// Runs on LM4F120/TM4C123
// Standalone user-level process example

#include <stdio.h>
#include <stdint.h>
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"

#include "OS.h"
#include "Display.h"

#define PF2     (*((volatile uint32_t *)0x40025010))
#define PF3     (*((volatile uint32_t *)0x40025020))

//unsigned int line = 0;

void thread(void)
{
  unsigned int id;
	
  while(1) {
    id = OS_Id();
    PF3 ^= 0x08;
    //line = ((line + 1) % 4) + 1;
    //Display_Message(0,line, "Thread: ", id);
    OS_Sleep(2000);
    //line = ((line + 1) % 4) + 1;
    //Display_Message(0,line, "Thread dying: ", id);
    PF3 ^= 0x08;
  }

  OS_Kill();
}

int main(void)
{
  unsigned int id;
  unsigned long time;
	
  id = OS_Id();
  PF2 ^= 0x04;
  //Display_Message(0,0, "Hello world: ", id);
  OS_AddThread(thread, 128, 2);
  //time = OS_Time();
  //OS_Sleep(1000);
  //time = (((OS_TimeDifference(time, OS_Time()))/1000ul)*125ul)/10000ul;
  //Display_Message(0,5, "Sleep time: ", time);

  while (1) {
    PF2 ^= 0x04;
    OS_Sleep(1000);
  }
  OS_Kill();
}
