// *************ADC.h**************
// EE445M/EE380L.6 Labs 1, 2, Lab 3, and Lab 4 
// mid-level ADC functions
// you are allowed to call functions in the low level ADCSWTrigger driver
// 
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano Jan 5, 2020, valvano@mail.utexas.edu

#include <stdint.h>


// ******** ADC_Init ************
// Initialize ADC0 sequencer 3
// Inputs:  channelNum (0 to 11) specifices pin to sample
// Outputs: returns error 1, if channelNum > 11
int ADC_Init(uint32_t channelNum);

// ******** ADC_In ************
// Software starts sequencer 3 and return 12-bit ADC result
// Inputs:  none
// Outputs: 12-bit ADC sampled value
uint32_t ADC_In(void);
