#ifndef UDP_LISTENER_H
#define UDP_LISTENER_H

namespace UDPListen{
    void openUDP(void);
    int receiveBasestation(unsigned char* dataUDP);
}
#endif