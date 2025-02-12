#ifndef INISIASISERIAL_H
#define INISIASISERIAL_H
#include <termios.h>

class Serial 
{
   public:
      int getindexportSTM32(const char *port);
      int errorattributeterminal(void);
      void initserial(void);
      int readserial(unsigned char *hasil);
      int writeSerial(unsigned char*, int);
   private:
      int index;
      termios toptions;

};
#endif