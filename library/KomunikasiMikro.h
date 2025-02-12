#ifndef KOMUNIKASIMIKRO_H
#define KOMUNIKASIMIKRO_H
#include"InisiasiSerial.h"
#include"TypeData.h"

namespace R2CKomunikasiSTM32{
    void STM32init(Serial&);
    int sendData(Serial, pcData);
    bool parseSTM32Data(STM32Data&,const unsigned char*);
}

#endif