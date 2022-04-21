/**
 * @file      Interpreter.h
 * @brief     Real Time Operating System for Labs 2, 3 and 4
 * @details   EE445M/EE380L.6
 * @version   V1.0
 * @author    Valvano
 * @copyright Copyright 2020 by Jonathan W. Valvano, valvano@mail.utexas.edu,
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      Jan 5, 2020

 ******************************************************************************/


#include <stdint.h>
#include <stdlib.h>


// ******** Interpreter ************
// Command line interpreter (shell)
// Run the user interface
// Inputs:  none
// Outputs: none
void Interpreter(void);

// ******** Interpreter_ParseCommand ************
// Parse and interpret a command
// Inputs: command to interpret
// Outputs: status to continue/exit interpreter
int Interpreter_ParseCommand(const char message[]);

// ******** Split_Message ************
// Split a string into its arguments based on whitespace and quotations
// Inputs: message string to split
// Outputs: number of arguments found
// Original Author: Hill (stackoverflow)
//                  https://stackoverflow.com/questions/9659697/parse-string-into-array-based-on-spaces-or-double-quotes-strings
size_t Split_Message(char *buffer, char *argv[], size_t argv_size);

// ******** Jitter ************
// Print max jitter measurment and histogram
// Inputs:	MaxJitter			maximum jitter size calculated
//			JitterSize			size of histogram
//			JitterHistogram[]	histogram array
// Outpus:	none
void Jitter(int32_t MaxJitter, uint32_t const JitterSize, uint32_t JitterHistogram[]);

// ******** WebInterpreter ************
// Command line interpreter (shell) through WiFi
// Run the user interface
// Inputs:  none
// Outputs: none
void WebInterpreter(void);

// ******** WebInterpreter_ParseCommand ************
// Parse and interpret a command through WiFi
// Inputs: command to interpret
// Outputs: status to continue/exit interpreter
int WebInterpreter_ParseCommand(const char message[]);
