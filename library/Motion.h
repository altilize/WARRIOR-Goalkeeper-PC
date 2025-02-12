#ifndef MOTION_H
#define MOTION_H
#include"TypeData.h"

namespace R2CMotion {
    bool pindahgrid(int, int, STM32Data&, pcData&,int,int);
    int  Motion(pcData&, int, int, int, int, int);
    bool setgrid(pcData&,int,int,int,int);
    bool putarrobotdarikompas(pcData&, STM32Data&,int,int);
    bool tendangkicker(pcData&,int,bool);
    bool hindarimusuh(pcData&,STM32Data&);
    bool movingrotation(float, pcData&, STM32Data&);
    bool hadap_teman(pcData&, STM32Data&, int, float, bool,bool,bool);
    bool putar_gawang(pcData&,STM32Data&);
    bool mengitariobjek(float, int, pcData&, STM32Data&);
    bool sampingkiper(pcData&, STM32Data&);
    bool pindahgrid2(int, int, STM32Data&, pcData&,int,int,int);

}

#endif