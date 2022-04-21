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
#include "../RTOS_Labs_common/esp8266.h"


// "help" command message definition
#define HELP_MESSAGE "Available Commands:\r\n \
    exit\r\n \
    help\r\n \
    lcd_top\r\n \
    lcd_ bottom\r\n \
    adc_in\r\n \
    time\r\n \
    led\r\n \
    jitter\r\n \
    performance\r\n \
    debug\r\n"


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
extern Sema4Type WebServerSema;

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
	else {													// Unrecognized commands, print help message
		UART_OutString("Error: Unrecognized Command\r\n");
		// UART_OutString(HELP_MESSAGE);
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

// ******** WebInterpreter ************
// Command line interpreter (shell) through WiFi
// Run the user interface
// Inputs:  none
// Outputs: none
void WebInterpreter(void) { 
  int status = 0;
  char inputString[maxMessageSize];
  int i = 0;

  ST7735_DrawString(0,3,"Connected      ",ST7735_YELLOW);

  // run interpreter
	ESP8266_Send("\r\nStarting Interpreter...\r\n");
  do {
		strcpy(inputString, "");	// Clear input string
    	ESP8266_Send("Interpreter>");
   		ESP8266_Receive(inputString, maxMessageSize);	// Requires [Ctrl + Enter] to send
	    // ESP8266_Send("\r\n");
		status = WebInterpreter_ParseCommand(inputString);
		ESP8266_Send("\r\n");
  } while(!status);    // exit based on status code
	
	ESP8266_Send("Exiting Interpreter...\r\n");
	for (i = 0; i < 80000000 / 1000; i++);		// small delay to finish printing

	ST7735_DrawString(0,3,"Diconnected      ",ST7735_RED);
	ESP8266_CloseTCPConnection();
	OS_Signal(&WebServerSema);

    OS_Kill();
}

// ******** WebInterpreter_ParseCommand ************
// Parse and interpret a command through WiFi
// Inputs: command to interpret
// Outputs: status to continue/exit interpreter
int WebInterpreter_ParseCommand(const char message[]) {
	char buf[maxMessageSize];
	char *argv[maxArgSize];
	size_t argc;
	char uint32_t_str[10];
	
	// Split message into arguments
	strcpy(buf, message);
	argc = Split_Message(buf, argv, maxArgSize);

	// Interpret command and perform respective function
	if 		(strcmp(argv[0], "exit") == 0) {				// Exit interpreter
		return 1;
	}
	else if (strcmp(argv[0], "help") == 0) {				// Print help message
		ESP8266_Send(HELP_MESSAGE);
	}
	else if (strcmp(argv[0], "lcd_top") == 0) {				// Print to the top-half of ST7735 display
		if (argc == 3 || argc == 4) {
			ST7735_Message(0, atoi(argv[1]), argv[2], atoi(argv[3]));
		}
		else {
			ESP8266_Send("Error: Wrong Usage\r\n");
		}
	}
	else if (strcmp(argv[0], "lcd_bottom") == 0) {			// Print to the bottom-half of ST7735 display
		if (argc == 3 || argc == 4) {
			ST7735_Message(1, atoi(argv[1]), argv[2], atoi(argv[3]));
		}
		else {
			ESP8266_Send("Error: Wrong Usage\r\n");
		}
	}
	else if (strcmp(argv[0], "adc_in") == 0) {				// Read in value from ADC module
		ESP8266_Send("ADC: ");
		sprintf(uint32_t_str, "%u", ADC_In());
		ESP8266_Send(uint32_t_str);
		ESP8266_Send("\r\n");
	}
	else if (strcmp(argv[0], "time") == 0) {				// Print system runtime
		ESP8266_Send("System Runtime: ");
		sprintf(uint32_t_str, "%u", OS_MsTime());
		ESP8266_Send(uint32_t_str);
		ESP8266_Send("ms\r\n");
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
			ESP8266_Send("Error: Wrong Usage\r\n");
		}
	}
	else if (strcmp(argv[0], "jitter") == 0) {				// Print jitter information
		ESP8266_Send("---------- Jitter1 -------------\r\n");	
		Jitter(MaxJitter1, JitterSize1, JitterHistogram1);	
		ESP8266_Send("---------- Jitter2 -------------\r\n");	
		Jitter(MaxJitter2, JitterSize2, JitterHistogram2);
	}
	else if (strcmp(argv[0], "performance") == 0) {			// Print performance measures
		ESP8266_Send("Performance Measures\r\n");
		ESP8266_Send("--------------------\r\n");
		ESP8266_Send("NumSamples: ");
		sprintf(uint32_t_str, "%u", NumSamples);
		ESP8266_Send(uint32_t_str);
		ESP8266_Send("\r\n");
		ESP8266_Send("NumCreated: ");
		sprintf(uint32_t_str, "%u", NumCreated);
		ESP8266_Send(uint32_t_str);
		ESP8266_Send("\r\n");
		ESP8266_Send("DataLost: ");
		sprintf(uint32_t_str, "%u", DataLost);
		ESP8266_Send(uint32_t_str);
		ESP8266_Send("\r\n");
		ESP8266_Send("FilterWork: ");
		sprintf(uint32_t_str, "%u", FilterWork);
		ESP8266_Send(uint32_t_str);
		ESP8266_Send("\r\n");
		ESP8266_Send("PIDWork: ");
		sprintf(uint32_t_str, "%u", PIDWork);
		ESP8266_Send(uint32_t_str);
		ESP8266_Send("\r\n");
		ESP8266_Send("CPUUtil: ");
		sprintf(uint32_t_str, "%u", CPUUtil);
		ESP8266_Send(uint32_t_str);
		ESP8266_Send("\r\n");
	}
	else if (strcmp(argv[0], "debug") == 0) {				// Print debugging parameters
		ESP8266_Send("Debugging Parameters\r\n");
		ESP8266_Send("--------------------\r\n");
		ESP8266_Send("x: ");
		for (int i = 0; i < (sizeof x / sizeof x[0]); i++) {
			sprintf(uint32_t_str, "%u", x[i]);
			ESP8266_Send(uint32_t_str);
			ESP8266_Send(" ");
		}
		ESP8266_Send("\r\n");
		ESP8266_Send("y: ");
		for (int i = 0; i < (sizeof y / sizeof y[0]); i++) {
			sprintf(uint32_t_str, "%u", y[i]);
			ESP8266_Send(uint32_t_str);
			ESP8266_Send(" ");
		}
		ESP8266_Send("\r\n");
	}
	else {													// Unrecognized commands, print help message
		ESP8266_Send("Error: Unrecognized Command\r\n");
		// ESP8266_Send(HELP_MESSAGE);
	}

  return 0;
}
