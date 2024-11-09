#pragma once
#include <wyconfig.h>

#include <Display_ST7789.h> // Required as creates the SPI object for SD CS Chip select
#include <SD.h>
//include <FS.h>
//#include <cstring>

extern uint16_t SDCard_Size;
extern uint16_t Flash_Size;

bool SD_Init();
void Flash_test();

bool File_Search(const char* directory, const char* fileName);
uint16_t Folder_retrieval(const char* directory, const char* fileExtension, char File_Name[][100],uint16_t maxFiles);
void remove_file_extension(char *file_name);