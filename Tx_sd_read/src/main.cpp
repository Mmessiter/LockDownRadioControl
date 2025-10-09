// allows access to the SD card in Windows or Linux but not on macOS

#include <SD.h>
#include <MTP_Teensy.h>
#define CS_SD BUILTIN_SDCARD // Works on T_3.6 and T_4.1

void setup()
{
  MTP.begin();
  SD.begin(CS_SD);
  MTP.addFilesystem(SD, "Transmitter's SD card");
}

void loop()
{
  MTP.loop(); 
}
