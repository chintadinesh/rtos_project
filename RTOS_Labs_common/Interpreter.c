// *************Interpreter.c**************
// Students implement this as part of EE445M/EE380L.12 Lab 1,2,3,4 
// High-level OS user interface
// 
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano 1/18/20, valvano@mail.utexas.edu
#include <stdint.h>
#include <string.h> 
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <ctype.h>
#include "../inc/ADCT0ATrigger.h"
#include "../inc/ADCSWTrigger.h"
#include "../inc/LED.h"
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/ST7735.h"
#include "../RTOS_Labs_common/UART0int.h"
#include "../RTOS_Labs_common/eDisk.h"
#include "../RTOS_Labs_common/eFile.h"
#include "../RTOS_Labs_common/ADC.h"
#include "../RTOS_Labs_common/Interpreter.h"
#include "../RTOS_Lab5_ProcessLoader/loader.h"

#include "../RTOS_Labs_common/heap.h"

// "help" command message definition
#define HELP_MESSAGE "Available Commands:\r\n \
    exit\r\n \
    help\r\n \
    lcd_top\r\n \
    lcd_ bottom\r\n \
    adc_in\r\n \
    time\r\n \
	led\r\n \
    file\r\n \
    jitter\r\n \
    performance\r\n \
    debug\r\n \
    elf\r\n"


extern uint32_t NumSamples;
extern uint32_t NumCreated;
extern uint32_t DataLost;
extern uint32_t FilterWork;
extern uint32_t PIDWork;
extern uint32_t x[64];
extern uint32_t y[64];
extern int32_t MaxJitter1;
extern int32_t MaxJitter2;
extern uint32_t JitterHistogram1[];	
extern uint32_t JitterHistogram2[];	
extern uint32_t const JitterSize1;	
extern uint32_t const JitterSize2;
extern uint32_t CPUUtil;
static const ELFSymbol_t symtab[] = {
	{ "ST7735_Message", ST7735_Message }
};

// Interpreter message specifications
uint8_t maxMessageSize = 100;
uint8_t maxArgSize = 30;


// ******** Interpreter ************
// Command line interpreter (shell)
// Run the user interface
// Inputs:  none
// Outputs: none
void Interpreter(void) { 
  int status = 0;
  char inputString[maxMessageSize];
  int i = 0;

  // run interpreter
	UART_OutString("\r\nStarting Interpreter...\r\n");
  do {
		strcpy(inputString, "");	// Clear input string
    	UART_OutString("Interpreter>");
   		UART_InString(inputString, maxMessageSize);
	    UART_OutString("\r\n");
		status = Interpreter_ParseCommand(inputString);
		UART_OutString("\r\n");
  } while(!status);    // exit based on status code
	
	UART_OutString("Exiting Interpreter...\r\n");
	for (i = 0; i < 80000000 / 1000; i++);		// small delay to finish UART printing
}

// ******** Interpreter_ParseCommand ************
// Parse and interpret a command
// Inputs: command to interpret
// Outputs: status to continue/exit interpreter
int Interpreter_ParseCommand(const char message[]) {
	char buf[maxMessageSize];
	char *argv[maxArgSize];
	size_t argc;
	char c;
	ELFEnv_t env = { symtab, 1};
	
	// Split message into arguments
	strcpy(buf, message);
	argc = Split_Message(buf, argv, maxArgSize);

	// Interpret command and perform respective function
	if 		(strcmp(argv[0], "exit") == 0) {				// Exit interpreter
		return 1;
	}
	else if (strcmp(argv[0], "help") == 0) {				// Print help message
		UART_OutString(HELP_MESSAGE);
	}
	else if (strcmp(argv[0], "lcd_top") == 0) {				// Print to the top-half of ST7735 display
		if (argc == 3 || argc == 4) {
			ST7735_Message(0, atoi(argv[1]), argv[2], atoi(argv[3]));
		}
		else {
			UART_OutString("Error: Wrong Usage\r\n");
		}
	}
	else if (strcmp(argv[0], "lcd_bottom") == 0) {			// Print to the bottom-half of ST7735 display
		if (argc == 3 || argc == 4) {
			ST7735_Message(1, atoi(argv[1]), argv[2], atoi(argv[3]));
		}
		else {
			UART_OutString("Error: Wrong Usage\r\n");
		}
	}
	else if (strcmp(argv[0], "adc_in") == 0) {				// Read in value from ADC module
		UART_OutString("ADC: ");
		UART_OutUDec(ADC_In());
		UART_OutString("\r\n");
	}
	else if (strcmp(argv[0], "time") == 0) {				// Print system runtime
		UART_OutString("System Runtime: ");
		UART_OutUDec(OS_MsTime());
		UART_OutString("ms\r\n");
	}
	else if (strcmp(argv[0], "led") == 0) {					// Toggle red, green, or blue led
		if (strcmp(argv[1], "red") == 0) {
			LED_RedToggle();
		}
		else if (strcmp(argv[1], "green") == 0) {
			LED_GreenToggle();
		}
		else if (strcmp(argv[1], "blue") == 0) {
			LED_BlueToggle();
		}
		else {
			UART_OutString("Error: Wrong Usage\r\n");
		}
	}
	else if (strcmp(argv[0], "file") == 0) {				// Interact with file system
		if (strcmp(argv[1], "init") == 0) {			// Initialize file
			if(!eFile_Init()) {
				UART_OutString("Initialization Completed!\n\r");
			}
			else {
				UART_OutString("Error\n\r");
			}
		}
		else if (strcmp(argv[1], "format") == 0) {	// Format disk
			if(!eFile_Format()) {
				UART_OutString("Formatting Completed!\n\r");
			}
			else {
				UART_OutString("Error\n\r");
			}
		}
		else if (strcmp(argv[1], "mount") == 0) {	// Mount file system
			if(!eFile_Mount()) {
				UART_OutString("Mount Completed!\n\r");
			}
			else {
				UART_OutString("Error\n\r");
			}
		}
		else if (strcmp(argv[1], "unmount") == 0) {	// Unmount file system
			if(!eFile_Unmount()) {
				UART_OutString("Unmount Completed!\n\r");
			}
			else {
				UART_OutString("Error\n\r");
			}
		}
		if (strcmp(argv[1], "ls") == 0) {			// Print directory contents
			if(eFile_DOpen("")){
				UART_OutString("ERROR opening directory\r\n");
			}
			else{
				char* name[7];
				unsigned long size;

				while(eFile_DirNext(name, &size) == 0){
					UART_OutString(*name);
					UART_OutString(" ");
					UART_OutUDec(size);
					UART_OutString("\r\n");
				}

				if(eFile_DClose()){
					UART_OutString("ERROR closing the directory\r\n");	
				}
			}
		}
		else if (strcmp(argv[1], "create") == 0) {	// Create file
			if (argc == 3) {
				if(!eFile_Create(argv[2])) {
					UART_OutString("New File Created!\n\r");
				}
				else {
					UART_OutString("Error\n\r");
				}
			}
			else {
				UART_OutString("Error: Wrong Usage\r\n");
			}
		}
		else if (strcmp(argv[1], "write") == 0) {	// Write to file
			if (argc == 4) {
				if(!eFile_WOpen(argv[2])) {
					for (int i = 0; i < strlen(argv[3]); i++) {
						if(eFile_Write(argv[3][i])) {
							UART_OutString("Error\n\r");
							break;
						}
					}
					if(eFile_WClose())              UART_OutString("Error\n\r");
					UART_OutString("Write Completed!\n\r");
				}
			}
			else {
				UART_OutString("Error: Wrong Usage\r\n");
			}
		}
		else if (strcmp(argv[1], "read") == 0) {	// Read file
			if (argc == 3) {
				if(!eFile_ROpen(argv[2])) {
					while (!eFile_ReadNext(&c)) {
						UART_OutChar(c);
					}
					UART_OutString("\n\r");
					if(eFile_RClose())              UART_OutString("Error\n\r");
					UART_OutString("Read Completed!\n\r");
				}
			}
			else {
				UART_OutString("Error: Wrong Usage\r\n");
			}
		}
		else if (strcmp(argv[1], "delete") == 0) {	// Delete file
			if (argc == 3) {
				if(!eFile_Delete(argv[2])) {
					UART_OutString("File Deleted!\n\r");
				}
				else {
					UART_OutString("Error\n\r");
				}
			}
			else {
				UART_OutString("Error: Wrong Usage\r\n");
			}
		}
	}
	else if (strcmp(argv[0], "jitter") == 0) {				// Print jitter information
		UART_OutString("---------- Jitter1 -------------\r\n");	
    	Jitter(MaxJitter1, JitterSize1, JitterHistogram1);	
		UART_OutString("---------- Jitter2 -------------\r\n");	
		Jitter(MaxJitter2, JitterSize2, JitterHistogram2);
	}
	else if (strcmp(argv[0], "performance") == 0) {			// Print performance measures
		UART_OutString("Performance Measures\r\n");
		UART_OutString("--------------------\r\n");
		UART_OutString("NumSamples: ");
		UART_OutUDec(NumSamples);
		UART_OutString("\r\n");
		UART_OutString("NumCreated: ");
		UART_OutUDec(NumCreated);
		UART_OutString("\r\n");
		UART_OutString("DataLost: ");
		UART_OutUDec(DataLost);
		UART_OutString("\r\n");
		UART_OutString("FilterWork: ");
		UART_OutUDec(FilterWork);
		UART_OutString("\r\n");
		UART_OutString("PIDWork: ");
		UART_OutUDec(PIDWork);
		UART_OutString("\r\n");
		UART_OutString("CPUUtil: ");
		UART_OutUDec(CPUUtil);
		UART_OutString("\r\n");
	}
	else if (strcmp(argv[0], "debug") == 0) {				// Print debugging parameters
		UART_OutString("Debugging Parameters\r\n");
		UART_OutString("--------------------\r\n");
		UART_OutString("x: ");
		for (int i = 0; i < (sizeof x / sizeof x[0]); i++) {
			UART_OutUDec(x[i]);
			UART_OutString(" ");
		}
		UART_OutString("\r\n");
		UART_OutString("y: ");
		for (int i = 0; i < (sizeof y / sizeof y[0]); i++) {
			UART_OutUDec(y[i]);
			UART_OutString(" ");
		}
		UART_OutString("\r\n");
	}
	else if (strcmp(argv[0], "elf") == 0) {					// Run program from SD Card
		if (argc == 2) {
			if (!exec_elf(argv[1], &env)) {
				UART_OutString("Error\r\n");
			}
		}
		else {
			UART_OutString("Error: Wrong Usage\r\n");
		}
	}
	else if (strcmp(argv[0], "top") == 0) {					


		if (strcmp(argv[1], "pcbs") == 0) {			// Initialize file

      // this functions by walking through the list of processes.
      // Initilize a process data structure and pass the pointer to the
      // function.
      // It returns NULL when the process list ends
      pcbType* pcb_walker;

      int n = 0;
      char buffer[50];

      // Format
      // NO FILE    ID  THREADS IO_SEM 
      // 1. hello.c 32  3       0
      sprintf(buffer, "%s. %s %s %s %s %s %s\n\r", "NO", 
                    "FILE", 
                    "ID",
                    "THREADS",          
                    "IO_SEM",
                    "TXT_LEN",
                    "DT_LEN");

      UART_OutString(buffer);


      for( OS_pcb_init_walker(&pcb_walker);
          (pcb_walker) != NULL;
          OS_pcb_walk(&pcb_walker)){


        sprintf(buffer, "%d. %s\t %d\t\t %d\t %d\t %d\t %d\n\r", 
                      ++n, 
                      pcb_walker->fl_name,
                      pcb_walker->id,
                      pcb_walker->threads, 
                      pcb_walker->io_sema.value,
                      Heap_Mem_Size(pcb_walker->text),
                      Heap_Mem_Size(pcb_walker->data)
                      );

        UART_OutString(buffer);
      }
    }
		else if (strcmp(argv[1], "tcbs") == 0) {//print only the pcb corresponding to the id

      char buffer[50];
      tcbType* tcb_walker;

      int n = 0;

      // Format
      // NO PID  TID PRI MGTG
      // 1. 0    1    2   0
      // 2. 0    1    2   0
      sprintf(buffer, "%s. %s\t %s\t %s\t %s\n\r", 
                    "NO", 
                    "PID",
                    "TID",
                    "PRI",
                    "MGTG"
                    );

      UART_OutString(buffer);

      pcbType* pcb = OS_get_pcb_by_fl_name("root");
      if(pcb == NULL){
        return 0;
      }


      int pcb_id = pcb->id;
      for( OS_tcb_init_walker(&tcb_walker, pcb_id);
          (tcb_walker) != NULL;
          OS_tcb_walk(&tcb_walker, pcb_id)){


        sprintf(buffer, "%d. %d\t\t %d\t\t %d\t %d\n\r", 
                      ++n, 
                      tcb_walker->pcb->id,
                      tcb_walker->id,
                      tcb_walker->priority, 
                      tcb_walker->is_migrating);

        UART_OutString(buffer);
      }

    }
    else {													// Unrecognized commands, print help message
      UART_OutString("Error: Unrecognized SubCommand\r\n");
      UART_OutString(HELP_MESSAGE);
    }
  }
	else if (strcmp(argv[0], "mgrt") == 0) {					
    if(argc == 2){
      int pcb_id = (int)(argv[1][0] - '0');
      if (pcb_id >= NUMPROCESS){
        UART_OutString("Error: Invalid pid\r\n");
        UART_OutString(HELP_MESSAGE);
      }
      else{
        OS_mark_for_migration(pcb_id);
      }
    }
    else{
      UART_OutString("Error: Invalid usage of mgrt\r\n");
      UART_OutString(HELP_MESSAGE);
    }
  }
  else {													// Unrecognized commands, print help message
    UART_OutString("Error: Unrecognized Command\r\n");
    UART_OutString(HELP_MESSAGE);
  }


  return 0;
}

// ******** Split_Message ************
// Split a string into its arguments based on whitespace and quotations
// Inputs: message string to split
// Outputs: number of arguments found
// Original Author: Hill (stackoverflow)
//                  https://stackoverflow.com/questions/9659697/parse-string-into-array-based-on-spaces-or-double-quotes-strings
size_t Split_Message(char *buffer, char *argv[], size_t argv_size) {
    char *p, *start_of_word;
    int c;
    enum states { DULL, IN_WORD, IN_STRING } state = DULL;
    size_t argc = 0;

    for (p = buffer; argc < argv_size && *p != '\0'; p++) {
        c = (unsigned char) *p;
        switch (state) {
        case DULL:
            if (isspace(c)) {
                continue;
            }

            if (c == '"') {
                state = IN_STRING;
                start_of_word = p + 1; 
                continue;
            }
            state = IN_WORD;
            start_of_word = p;
            continue;

        case IN_STRING:
            if (c == '"') {
                *p = 0;
                argv[argc++] = start_of_word;
                state = DULL;
            }
            continue;

        case IN_WORD:
            if (isspace(c)) {
                *p = 0;
                argv[argc++] = start_of_word;
                state = DULL;
            }
            continue;
        }
    }

    if (state != DULL && argc < argv_size)
        argv[argc++] = start_of_word;

    return argc;
}

// ******** Jitter ************
// Print max jitter measurment and histogram
// Inputs:	MaxJitter			maximum jitter size calculated
//			JitterSize			size of histogram
//			JitterHistogram[]	histogram array
// Outpus:	none
void Jitter(int32_t MaxJitter, uint32_t const JitterSize, uint32_t JitterHistogram[]){
  UART_OutString("Jitter Stats\r\n");
  UART_OutString("--------------------\r\n");
  UART_OutString("MaxJitter: ");
  UART_OutSDec(MaxJitter);
  UART_OutString("\r\n");
  UART_OutString("--------------------\r\n");
  UART_OutString("Histogram: ");
  for (int i = 0; i < JitterSize; i++) {
    UART_OutUDec(JitterHistogram[i]);
    UART_OutString(" ");
  }
  UART_OutString("\r\n");
}
