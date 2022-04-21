// filename ************** eFile.c *****************************
// High-level routines to implement a solid-state disk 
// Students implement these functions in Lab 4
// Jonathan W. Valvano 1/12/20
#include <stdint.h>
#include <string.h>
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/eDisk.h"
#include "../RTOS_Labs_common/eFile.h"
#include <stdio.h>


// File System component definitions
#define DISKDRIVE         0     // 0 for SDCard
#define DIRECTORYBLOCKS   0     // blocks 0 to 1 in disk, Directory = NUMBLOCKS * 12 bytes (max)
#define FATBLOCKS         2     // blocks 2 to 9 in disk, FAT = NUMBLOCKS * 2 bytes
#define DISKBLOCKS        10    // blocks 10 to NUMBLOCKS
#define MAXBLOCKS         512   // make bigger to access more memory in disk
#define BUFFERSIZE        512   // 1 block in disk is 512 bytes


// File System component declarations
fileType Directory[MAXBLOCKS];  // Directory in memory, sector 0 in disk
blockType FAT[MAXBLOCKS];       // File Allocation Table in memory, sector 1 in disk
uint8_t InitFlag = 0;
uint8_t ReadWriteFlag = 0;      // file is open, 1 for write, 2 for read, 3 for directory, 0 for none
fileType *File;                 // currently open file
int16_t Block = -1;             // block in RAM
int16_t BlockIndex = 0;
BYTE Buff[BUFFERSIZE];          // buffer in RAM, 512 bytes
uint32_t BuffIndex = 0;         // position in buffer
extern Sema4Type LCDFree;


//---------- eFile_Init-----------------
// Activate the file system, without formating
// Input: none
// Output: 0 if successful and 1 on failure (already initialized)
int eFile_Init(void){ // initialize file system
  // File system previously initialized
  if (InitFlag != 0) {
    return 1;
  }

  // Initialize local Directory to empty
  for (int i = 0; i < MAXBLOCKS; i++) {
    strcpy(Directory[i].name, "");
    Directory[i].startBlock = -1;
    Directory[i].size = 0;
  }

  // Initialize local FAT to empty
  for (int i = 0; i < DISKBLOCKS; i++) {
    FAT[i].nextBlock = 0;
  }
  for (int i = DISKBLOCKS; i < MAXBLOCKS; i++) {
    FAT[i].nextBlock = -1;
  }

  eDisk_Init(DISKDRIVE);    // Initialize disk driver

  // File system initialized
  InitFlag = 1;
  OS_InitSemaphore(&LCDFree, 1);

  return 0;
}

//---------- eFile_Format-----------------
// Erase all files, create blank directory, initialize free space manager
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Format(void){ // erase disk, add format
  unsigned long status;

  // File system not initialized
  if (!InitFlag)    return 1;

  OS_bWait(&LCDFree);

  // Initialize local Directory to empty
  for (int i = 0; i < MAXBLOCKS; i++) {
    strcpy(Directory[i].name, "");
    Directory[i].startBlock = -1;
    Directory[i].size = 0;
  }

  // Initialize local FAT to empty
  for (int i = 0; i < DISKBLOCKS; i++) {
    FAT[i].nextBlock = 0;
  }
  for (int i = DISKBLOCKS; i < MAXBLOCKS; i++) {
    FAT[i].nextBlock = -1;
  }

  // Write Directory and FAT to disk
  status = OS_LockScheduler();
  if (eDisk_WriteBlocks((BYTE*)&Directory, DIRECTORYBLOCKS, 2)) {
    OS_bSignal(&LCDFree);
    return 1;    // error
  }
  if (eDisk_WriteBlocks((BYTE*)&FAT, FATBLOCKS, 8)) {
    OS_bSignal(&LCDFree);
    return 1;    // error
  }
  OS_UnLockScheduler(status);

  OS_bSignal(&LCDFree);

  return 0;
}

//---------- eFile_Mount-----------------
// Mount the file system, without formating
// Input: none
// Output: 0 if successful and 1 on failure
int eFile_Mount(void){ // initialize file system
  unsigned long status;

  // File system not initialized
  if (!InitFlag)    return 1;

  OS_bWait(&LCDFree);

  // Read Directory and FAT from disk
  status = OS_LockScheduler();
  if (eDisk_ReadBlocks((BYTE*)&Directory, DIRECTORYBLOCKS, 2)) {
    OS_bSignal(&LCDFree);
    return 1;    // error
  }
  if (eDisk_ReadBlocks((BYTE*)&FAT, FATBLOCKS, 8)) {
    OS_bSignal(&LCDFree);
    return 1;    // error
  }
  OS_UnLockScheduler(status);

  OS_bSignal(&LCDFree);

  return 0;
}

//---------- eFile_Create-----------------
// Create a new, empty file with one allocated block
// Input: file name is an ASCII string up to seven characters 
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Create(const char name[]){  // create new file, make it empty
  int32_t block = -1;
  int i;

  // File system not initialized
  if (!InitFlag)    return 1;

  OS_bWait(&LCDFree);

  // Find free block
  for (i = DISKBLOCKS; i < MAXBLOCKS; i++) {
    if (FAT[i].nextBlock == -1) {       // free block
      block = i;
      FAT[i].nextBlock = 0;
      break;
    }
  }
  if (i == MAXBLOCKS) {   // no free blocks available
    OS_bSignal(&LCDFree);
    return 1;
  }

  // Find free file
  for (int i = 0; i < MAXBLOCKS; i++) {
    if (Directory[i].startBlock == -1) {      // free file
      strcpy(Directory[i].name, name);
      Directory[i].startBlock = block;
      Directory[i].size = 0;
      break;
    }
  }

  OS_bSignal(&LCDFree);

  return 0;
}

//---------- eFile_WOpen-----------------
// Open the file, read into RAM last block
// Input: file name is an ASCII string up to seven characters
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_WOpen(const char name[]){      // open a file for writing
  int32_t block = -1;
  int i;

  // File system not initialized
  if (!InitFlag)    return 1;

  OS_bWait(&LCDFree);

  // Check for open file
  if (ReadWriteFlag) {
    OS_bSignal(&LCDFree);
    return 1;
  }

  // Find file
  for (i = 0; i < MAXBLOCKS; i++) {
    if (strcmp(Directory[i].name, name) == 0) {
      File = &Directory[i];
      block = Directory[i].startBlock;
      break;
    }
  }
  if (i == MAXBLOCKS) {   // file not found
    OS_bSignal(&LCDFree);
    return 1;
  }

  // Move to last block
  while (FAT[block].nextBlock > 0) {
    block = FAT[block].nextBlock;
  }
  Block = block;

  // Read data
  if (eDisk_ReadBlock(Buff, Block)) {
    File = NULL;
    Block = -1;
    // Buff = NULL;
    BuffIndex = 0;
    OS_bSignal(&LCDFree);
    return 1;    // error
  }
  BuffIndex = File->size % BUFFERSIZE;

  ReadWriteFlag = 1;    // file open fro writing

  OS_bSignal(&LCDFree);

  return 0;
}

//---------- eFile_Write-----------------
// save at end of the open file
// Input: data to be saved
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Write(const char data){
  int32_t block = -1;
  int i;

  // File system not initialized
  if (!InitFlag)    return 1;

  OS_bWait(&LCDFree);

  // Check for open file for writing
  if (ReadWriteFlag != 1) {
    OS_bSignal(&LCDFree);
    return 1;
  }

  // End of current block reached, move onto next
  if (BuffIndex % BUFFERSIZE == 0) {
    // Write data
    if (eDisk_WriteBlock(Buff, Block)) {
    OS_bSignal(&LCDFree);
    return 1;    // error
    }

    // Find free block
    for (i = DISKBLOCKS; i < MAXBLOCKS; i++) {
      if (FAT[i].nextBlock == -1) {       // free block
        block = i;
        FAT[i].nextBlock = 0;
        break;
      }
    }
    if (i == MAXBLOCKS) {   // no free blocks available
      OS_bSignal(&LCDFree);
      return 1;
    }
    
    // Append new block to file
    FAT[Block].nextBlock = block;
    Block = block;

    // Read data
    if (eDisk_ReadBlock(Buff, Block)) {
      File = NULL;
      Block = -1;
      // Buff = NULL;
      BuffIndex = 0;
      OS_bSignal(&LCDFree);
      return 1;    // error
    }
    BuffIndex = 0;
  }

  // Append data to the end of the buffer
  Buff[BuffIndex] = data;
  BuffIndex++;
  File->size++;

  OS_bSignal(&LCDFree);

  return 0;
}

//---------- eFile_WClose-----------------
// close the file, left disk in a state power can be removed
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_WClose(void){ // close the file for writing
  // File system not initialized
  if (!InitFlag)    return 1;

  OS_bWait(&LCDFree);

  // Check for open file for writing
  if (ReadWriteFlag != 1) {
    OS_bSignal(&LCDFree);
    return 1;
  }

  // Write data
  if (eDisk_WriteBlock(Buff, Block)) {
    OS_bSignal(&LCDFree);
    return 1;    // error
  }

  // Close file
  File = NULL;
  Block = -1;
  // Buff = NULL;
  BuffIndex = 0;
  ReadWriteFlag = 0;

  OS_bSignal(&LCDFree);

  return 0;
}

//---------- eFile_ROpen-----------------
// Open the file, read first block into RAM 
// Input: file name is an ASCII string up to seven characters
// Output: 0 if successful and 1 on failure (e.g., trouble read to flash)
int eFile_ROpen(const char name[]){      // open a file for reading 
  int32_t block = -1;
  int i;

  // File system not initialized
  if (!InitFlag)    return 1;

  OS_bWait(&LCDFree);

  // Check for open file
  if (ReadWriteFlag) {
    OS_bSignal(&LCDFree);
    return 1;
  }

  // Find file
  for (i = 0; i < MAXBLOCKS; i++) {
    if (strcmp(Directory[i].name, name) == 0) {
      File = &Directory[i];
      block = Directory[i].startBlock;
      break;
    }
  }
  if (i == MAXBLOCKS) {   // file not found
    OS_bSignal(&LCDFree);
    return 1;
  }

  // Move to first block
  Block = block;

  // Read data
  if (eDisk_ReadBlock(Buff, Block)) {
    File = NULL;
    Block = -1;
    // Buff = NULL;
    BuffIndex = 0;
    OS_bSignal(&LCDFree);
    return 1;    // error
  }
  BlockIndex = 0;
  BuffIndex = 0;

  ReadWriteFlag = 2;      // file open for reading

  OS_bSignal(&LCDFree);

  return 0;
}
 
//---------- eFile_ReadNext-----------------
// retrieve data from open file
// Input: none
// Output: return by reference data
//         0 if successful and 1 on failure (e.g., end of file)
int eFile_ReadNext(char *pt){       // get next byte
  // File system not initialized
  if (!InitFlag)    return 1;

  OS_bWait(&LCDFree);

  // Check for open file for reading
  if (ReadWriteFlag != 2) {
    OS_bSignal(&LCDFree);
    return 1;
  }

  // End of file reached
  if (BlockIndex * BUFFERSIZE + BuffIndex == File->size) {
    OS_bSignal(&LCDFree);
    return 1;
  }

  // End of block reached, move onto next
  if (BuffIndex % BUFFERSIZE == 0) {
    Block = FAT[Block].nextBlock;
    if (eDisk_ReadBlock(Buff, Block)) {
      File = NULL;
      Block = -1;
      // Buff = NULL;
      BuffIndex = 0;
      OS_bSignal(&LCDFree);
      return 1;    // error
    }
    BuffIndex = 0;
  }

  // *pt = *(Buff + BuffPos * sizeof(BYTE));
  *pt = Buff[BuffIndex];
  BuffIndex++;

  OS_bSignal(&LCDFree);

  return 0;
}

//---------- eFile_RClose-----------------
// close the reading file
// Input: none
// Output: 0 if successful and 1 on failure (e.g., wasn't open)
int eFile_RClose(void){ // close the file for writing
  // File system not initialized
  if (!InitFlag)    return 1;

  OS_bWait(&LCDFree);

  // Check for open file for reading
  if (ReadWriteFlag != 2) {
    OS_bSignal(&LCDFree);
    return 1;
  }

  // Close file
  File = NULL;
  Block = -1;
  // Buff = NULL;
  BuffIndex = 0;
  ReadWriteFlag = 0;

  OS_bSignal(&LCDFree);

  return 0;
}

//---------- eFile_Delete-----------------
// delete this file
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Delete(const char name[]){  // remove this file
  int32_t block = -1;
  int32_t temp = -1;
  int i;

  // File system not initialized
  if (!InitFlag)    return 1;

  OS_bWait(&LCDFree);

  // Find and delete file
  for (i = 0; i < MAXBLOCKS; i++) {
    if (strcmp(Directory[i].name, name) == 0) {   // delete this entry
      block = Directory[i].startBlock;
      strcpy(Directory[i].name, "");
      Directory[i].startBlock = -1;
      Directory[i].size = 0;
      break;
    }
  }
  if (i == MAXBLOCKS) {   // file not found
    OS_bSignal(&LCDFree);
    return 1;
  }

  // Delete blocks
  while (block > 0) {
    temp = FAT[block].nextBlock;
    FAT[block].nextBlock = -1;      // delete block
    block = temp;
  }

  OS_bSignal(&LCDFree);

  return 0;
}

int _dir_block = -1;
//---------- eFile_DOpen-----------------
// Open a (sub)directory, read into RAM
// Input: directory name is an ASCII string up to seven characters
//        (empty/NULL for root directory)
// Output: 0 if successful and 1 on failure (e.g., trouble reading from flash)
int eFile_DOpen(const char name[]){ // open directory
  // File system not initialized
  if (!InitFlag)    return 1;

  OS_bWait(&LCDFree);

  // Check for open file
  if (ReadWriteFlag) {
    OS_bSignal(&LCDFree);
    return 1;
  }

  _dir_block = 0;
  for (; _dir_block < MAXBLOCKS;) {
    if (Directory[_dir_block].startBlock != -1) {
        break;
    }
    _dir_block++; //increment only when there is no match
  }

  ReadWriteFlag = 3;

  if(_dir_block == MAXBLOCKS){
    _dir_block = -1;
    OS_bSignal(&LCDFree);
    return 0;
  }

  OS_bSignal(&LCDFree);

  return 0;

}

//---------- eFile_DirNext-----------------
// Retreive directory entry from open directory
// Input: none
// Output: return file name and size by reference
//         0 if successful and 1 on failure (e.g., end of directory)
int eFile_DirNext(char *name[], unsigned long *size){  // get next entry 
  // File system not initialized
  if (!InitFlag)    return 1;

  OS_bWait(&LCDFree);

  // Check for open directory
  if (ReadWriteFlag != 3) {
    OS_bSignal(&LCDFree);
    return 1;
  }

  if (_dir_block == -1) {
    OS_bSignal(&LCDFree);
    return 1;
  }

  for (; _dir_block < MAXBLOCKS;) {
    if (Directory[_dir_block].startBlock != -1) {
      *name = Directory[_dir_block].name;
      *size = Directory[_dir_block].size;
      _dir_block++;
      break;
    }
    _dir_block++;
  }
  
  if(_dir_block == MAXBLOCKS){
    OS_bSignal(&LCDFree);
    return 1;
  }

  OS_bSignal(&LCDFree);
  
  return 0;
}

//---------- eFile_DClose-----------------
// Close the directory
// Input: none
// Output: 0 if successful and 1 on failure (e.g., wasn't open)
int eFile_DClose(void){ // close the directory
  // File system not initialized
  if (!InitFlag)    return 1;

  OS_bWait(&LCDFree);

  // Check for open directory
  if (ReadWriteFlag != 3) {
    OS_bSignal(&LCDFree);
    return 1;
  }

  _dir_block = -1;
  ReadWriteFlag = 0;

  OS_bSignal(&LCDFree);

  return 0;

}

//---------- eFile_Unmount-----------------
// Unmount and deactivate the file system
// Input: none
// Output: 0 if successful and 1 on failure (not currently mounted)
int eFile_Unmount(void){
  unsigned long status;

  // File system not initialized
  if (!InitFlag)    return 1;

  OS_bWait(&LCDFree);

  // Write Directory and FAT to disk
  status = OS_LockScheduler();
  if (eDisk_WriteBlocks((BYTE*)&Directory, DIRECTORYBLOCKS, 2)) {
    OS_bSignal(&LCDFree);
    return 1;    // error
  }
  if (eDisk_WriteBlocks((BYTE*)&FAT, FATBLOCKS, 8)) {
    OS_bSignal(&LCDFree);
    return 1;    // error
  }
  OS_UnLockScheduler(status);

  OS_bSignal(&LCDFree);

  return 0;
}
