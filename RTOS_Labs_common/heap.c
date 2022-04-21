// filename *************************heap.c ************************
// Implements memory heap for dynamic memory allocation.
// Follows standard malloc/calloc/realloc/free interface
// for allocating/unallocating memory.

// Jacob Egner 2008-07-31
// modified 8/31/08 Jonathan Valvano for style
// modified 12/16/11 Jonathan Valvano for 32-bit machine
// modified August 10, 2014 for C99 syntax

/* This example accompanies the book
   "Embedded Systems: Real Time Operating Systems for ARM Cortex M Microcontrollers",
   ISBN: 978-1466468863, Jonathan Valvano, copyright (c) 2015

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


#include <string.h>
#include <stdint.h>
#include "../RTOS_Labs_common/heap.h"
#include "../RTOS_Labs_common/OS.h"

/*

*/

// we cant really use a linked list. Because that would required dynamic
// allocation of node elements -- checken-egg requirement.
// Matthew said, Lecture8 has a simpler approximate implementation.

typedef int32_t tag_t;

// number of bytes used for tag. We are using int16_t here
//#define TAG_SIZE sizeof(tag_t)
#define TAG_SIZE 1

#define HEAP_MAX 9
#define HEAP_SIZE (1<<HEAP_MAX)
int32_t Heap[HEAP_SIZE]; // 2048 bytes

// Heap needs to be pretected.
Sema4Type heap_mutex;

// flag to represent that heap is initilized
char _heap_initilized = 0;

//******** Heap_Init *************** 
// Initialize the Heap
// input: none
// output: always 0
// notes: Initializes/resets the heap to a clean state where no memory
//  is allocated.
int32_t Heap_Init(void){

  OS_InitSemaphore(&heap_mutex, 1);
  
  OS_bWait(&heap_mutex);
  // each element in the heap is 1 bytes.
  Heap[0] = -1*HEAP_MAX; 
  Heap[HEAP_SIZE - TAG_SIZE] = -1*HEAP_MAX;

  // set contents to 0
  memset(Heap + 1, 0, (HEAP_SIZE - 2*TAG_SIZE)<< 2); // word alignemnt

  OS_bSignal(&heap_mutex);

  _heap_initilized = 1;

  return 0;
}

uint16_t get_nearest_size(int32_t desiredBytes){
  int32_t num = desiredBytes + 2*4;
  int32_t msb = 0;
  
  char is_pow_2 = num % 2; 

  while(num >>= 1){
    is_pow_2 +=  num %2;
    msb++;
  }

  if(is_pow_2 == 1)  
    return msb - 2;
  else
    return msb + 1 - 2;

}

// Heap is char type. Hence, we need to type cast it to access uint16_t
// integers
// returns the interger at the specified position from a character array.
tag_t get_tag(int pos){
  //return *(tag_t*)(Heap + pos/TAG_SIZE);
  return Heap[pos];
}

void set_tag(int pos, tag_t num){
  //*(tag_t*)(Heap + pos/TAG_SIZE) = num;
  Heap[pos] = num;
}


//******** Heap_Malloc *************** 
// Allocate memory, data not initialized
// input: 
//   desiredBytes: desired number of bytes to allocate
// output: void* pointing to the allocated memory or will return NULL
//   if there isn't sufficient space to satisfy allocation request
void* Heap_Malloc(int32_t desiredBytes){
  if(!_heap_initilized)
    return 0;

  // the tags are negative numbers, 
  // adjust your logic accordingly
  int8_t required_size = -1*get_nearest_size(desiredBytes);

  int8_t current_best = -128;
  uint16_t current_best_position = 0;

  uint16_t start = 0;

  tag_t start_tag;
  OS_bWait(&heap_mutex);

  for(; start < HEAP_SIZE;){
    start_tag = get_tag(start); 

    if(start_tag == required_size){
      current_best =  start_tag;
      current_best_position = start;
      break;
    }
    else if(start_tag < required_size  // larger free node
            && start_tag > current_best // better than current
            ){
      current_best =  start_tag;
      current_best_position = start;
    }

    start += (start_tag > 0) ? (1 << start_tag) : (1 << (-1*start_tag)); //move to next node
  }

  if(current_best == -128) { // reached end of heap
    OS_bSignal(&heap_mutex);
    return 0;
  }

  if(current_best  == required_size){ // found the required size
    //set_tag(current_best_position, -1*required_size);
    //set_tag(current_best_position + ((1 << (-1*required_size)) - TAG_SIZE), -1*required_size);
  }
  else{ // split the current_best
    for(;current_best < required_size;){ //adjust negative logic
      current_best = current_best + 1;

      // mark the other half free
      set_tag(current_best_position + (1 << -1*current_best), current_best);
      set_tag(current_best_position + (1 << (-1*current_best + 1)) - TAG_SIZE, current_best);

      // free space must be remarked
      set_tag(current_best_position, current_best);
      set_tag(current_best_position + (1 << -1*current_best) - TAG_SIZE, current_best);

    }
  }

  set_tag(current_best_position, -1*current_best); // change the current block to +ve
  set_tag(current_best_position + (1 << -1*current_best) - TAG_SIZE, -1*current_best);

  void* ans_pointer = (void*)(Heap + current_best_position + TAG_SIZE);

  // set the contents to zero
  memset(ans_pointer, 0, ((1 << -1*current_best) - 2*TAG_SIZE) << 2);

  OS_bSignal(&heap_mutex);

  // return the start locaiton pointer without the tag memory
  return ans_pointer;
}


//******** Heap_Calloc *************** 
// Allocate memory, data are initialized to 0
// input:
//   desiredBytes: desired number of bytes to allocate
// output: void* pointing to the allocated memory block or will return NULL
//   if there isn't sufficient space to satisfy allocation request
//notes: the allocated memory block will be zeroed out
void* Heap_Calloc(int32_t desiredBytes){  
  if(!_heap_initilized)
    return 0;

  void * ans = Heap_Malloc(desiredBytes);

  OS_bWait(&heap_mutex);
  memset(ans, 0, desiredBytes);
  OS_bSignal(&heap_mutex);

  return ans;
}


//******** Heap_Realloc *************** 
// Reallocate buffer to a new size
//input: 
//  oldBlock: pointer to a block
//  desiredBytes: a desired number of bytes for a new block
// output: void* pointing to the new block or will return NULL
//   if there is any reason the reallocation can't be completed
// notes: the given block may be unallocated and its contents
//   are copied to a new block if growing/shrinking not possible
void* Heap_Realloc(void* oldBlock, int32_t desiredBytes){
  if(!_heap_initilized)
    return 0;

  int8_t required_size = get_nearest_size(desiredBytes);
  int8_t current_size = *((tag_t*)(oldBlock) -1);
  if(required_size == current_size)
    return oldBlock;

  void* new_block = Heap_Calloc(desiredBytes);
  if(!new_block)
    return 0;

  int current_bytes = ((1<<current_size) - 2*TAG_SIZE) << 2;
  int bytes_to_copy = (desiredBytes < current_bytes) ? desiredBytes : current_bytes;

  // copy the old block contents
  OS_bWait(&heap_mutex);
  memcpy(new_block, oldBlock, bytes_to_copy);
  OS_bSignal(&heap_mutex);

  if(Heap_Free(oldBlock))
    return NULL;

  return new_block; 
}


//******** Heap_Free *************** 
// return a block to the heap
// input: pointer to memory to unallocate
// output: 0 if everything is ok, non-zero in case of error (e.g. invalid pointer
//     or trying to unallocate memory that has already been unallocated
int32_t Heap_Free(void* pointer){

  /*
  assumption:
  the pointer is valid. It is the start of a true heap space. In other words
  pointer -1 points to a valid tag.
  */

  if(!_heap_initilized)
    return 0;
  
  OS_bWait(&heap_mutex);

  int8_t current_tag = *((tag_t*)(pointer) - TAG_SIZE);
  // return error if the poitner is not used
  if(current_tag < 0){
    OS_bSignal(&heap_mutex);
    return 1;
  }

  // check if the tags match
  if(current_tag != *((tag_t*)pointer + ( 1 << current_tag) -2*TAG_SIZE)){ 
    OS_bSignal(&heap_mutex);
    return 1;
  }

  // tag must be less the max possible value
  if(current_tag > HEAP_MAX){
    OS_bSignal(&heap_mutex);
    return 1;
  }

  //freeing the current node
  *((tag_t*)pointer - TAG_SIZE) = -1*current_tag;
  *((tag_t*)pointer + (1 << current_tag) -2*TAG_SIZE) = -1*current_tag;

  //memset(pointer, 0, (1 << current_tag) -2*TAG_SIZE);
  memset(pointer, 0, ((1 << current_tag) - 2*TAG_SIZE) << 2);

  void *left_tag_pointer, *right_tag_pointer,   // we merge the left and right
        *middle_tag_pointer, // make this zeros
        *previous_pointer, *next_pointer; // the corresponding previous and left nodes
  void* merge_pointer = pointer; // initilize the loop for merging

  // TODO: boundary checking
  // previous for the first block is invalid
  // next for the last block is invalid.
  // invalid is represented as if it is already occupied
  // first and last blocks are identified by comparing the pointers

  int8_t next_tag, previous_tag;
  uint8_t u_current_tag, u_next_tag, u_previous_tag;
  int8_t is_first = 0, is_last = 0;
  for(; ; ) {

    // extract the tags and pointers to previous nodes
    current_tag = *((tag_t*)merge_pointer - TAG_SIZE);
    u_current_tag = (current_tag < 0) ? -1 * current_tag : current_tag;

    next_pointer = (tag_t*)merge_pointer + (1 << u_current_tag);
    next_tag  = *((tag_t*)next_pointer - TAG_SIZE);
    u_next_tag = (next_tag < 0) ? -1 * next_tag : next_tag;

    previous_tag = *((tag_t*)merge_pointer - 2*TAG_SIZE);// next tag size
    u_previous_tag = (previous_tag < 0) ? -1 * previous_tag : previous_tag;

    previous_pointer = (tag_t*)merge_pointer  - (1 << u_previous_tag); 
    
    is_first = (merge_pointer == (Heap + 1)) ?  1 : 0;
    is_last = (next_pointer > (Heap + HEAP_SIZE - 1)) ? 1 : 0;

    // get the left tag and right tags
    if((current_tag ==  previous_tag ) && !is_first){
      left_tag_pointer = (tag_t*)previous_pointer - TAG_SIZE;
      middle_tag_pointer = (tag_t*)merge_pointer - 2*TAG_SIZE;
      right_tag_pointer = (tag_t*)next_pointer - 2*TAG_SIZE;
    }
    else if ((current_tag == next_tag) && !is_last){
      left_tag_pointer = (tag_t*)merge_pointer - TAG_SIZE;
      middle_tag_pointer = (tag_t*)next_pointer - 2*TAG_SIZE;
      right_tag_pointer = (tag_t*)next_pointer + (1 << u_next_tag) - 2*TAG_SIZE; 
    }
    else { //  when there are no nodes to merge with
      break;
    }

    // negative increment the tags
    *(tag_t*)left_tag_pointer = *(tag_t*)left_tag_pointer - 1; // negative increment by 1
    *(tag_t*)right_tag_pointer = *(tag_t*)right_tag_pointer - 1;
    *(tag_t*)middle_tag_pointer = 0;    // just replace the values with zeros
    *((tag_t*)middle_tag_pointer + TAG_SIZE) = 0;

    merge_pointer = (tag_t*)left_tag_pointer + TAG_SIZE; // update the pointer
  }

  OS_bSignal(&heap_mutex);
  return 0;   // replace
}


//******** Heap_Stats *************** 
// return the current status of the heap
// input: reference to a heap_stats_t that returns the current usage of the heap
// output: 0 in case of success, non-zeror in case of error (e.g. corrupted heap)
int32_t Heap_Stats(heap_stats_t *stats){
  if(!_heap_initilized)
    return 1;

  // initilize stats
  stats->used = 0;
  stats->free = 0;
  stats->size = 0;

  uint16_t start = 0;

  OS_bWait(&heap_mutex);
  for(; start < HEAP_SIZE;){
    tag_t start_tag = get_tag(start); 

    // tag should never be 0. Since we always have atleast 2 bytes for the
    // tags.

    if(start_tag > 0){ // used node

      // 2 bytes are used for the tag
      stats->used += ((1 << start_tag) << 2) - 2*4;
      stats->size += 2*4;
      start += (1 << start_tag);
    }
    else if(start_tag < 0) { // free node

      // 2 bytes are used for the tag
      stats->free += ((1 << (-1*start_tag)) << 2) - 2*4;
      stats->size += 2*4;
      start += (1 << (-1 * start_tag));
    }
  }

  stats->size += stats->used + stats->free;

  OS_bSignal(&heap_mutex);

  return 0;   // replace
}
