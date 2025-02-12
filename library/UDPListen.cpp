#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h> // untuk sleep
#include <string.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include "UDPListen.h"

#define MULTICAST_ADDR "224.16.32.110" //ip multicast
#define PORT 12478 //port 

int sockfd; 
struct sockaddr_in multicast_addr;
struct ip_mreq multicast_request;

void UDPListen::openUDP(){

    // Membuat soket UDP
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("Gagal membuat soket");
        exit(1);
    }

    // Inisialisasi struktur sockaddr_in
    memset(&multicast_addr, 0, sizeof(multicast_addr));
    multicast_addr.sin_family = AF_INET;
    multicast_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    multicast_addr.sin_port = htons(PORT);

    // Mengikat soket ke alamat multicast
    if (bind(sockfd, (struct sockaddr *)&multicast_addr, sizeof(multicast_addr)) < 0) {
        perror("Gagal mengikat soket");
        close(sockfd);
        exit(1);
    }

    // Mengatur alamat multicast yang akan digunakan
    inet_pton(AF_INET, MULTICAST_ADDR, &(multicast_request.imr_multiaddr));
    multicast_request.imr_interface.s_addr = htonl(INADDR_ANY);

    // Bergabung dengan grup multicast
    if (setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (void *)&multicast_request, sizeof(multicast_request)) < 0) {
        perror("Gagal mengatur soket");
        close(sockfd);
        exit(1);
    }

    printf("Menerima data dari %s:%d...\n", MULTICAST_ADDR, PORT);
}

int UDPListen::receiveBasestation(unsigned char* dataUDP){
    int bytes_received = recvfrom(sockfd, dataUDP, 256, 0, NULL, 0);
    if (bytes_received < 0) {
        perror("Gagal menerima data");
        close(sockfd);
        exit(1);
    }
    else if(bytes_received==6 || bytes_received==3 || bytes_received==4){
        return *dataUDP;
    }
    return false;
}