#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h> // untuk sleep
#include <string.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include "UDPSend.h"

#define MAX_BUF_SIZE 1024 //maksimal buffer
#define MULTICAST_ADDR "224.16.32.110" //ip multicast
#define PORT 12478 //port

void UDPSend::sendBasestation(unsigned char* data) {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("socket");
        exit(1);
    }

    struct sockaddr_in multicast_addr;
    memset(&multicast_addr, 0, sizeof(multicast_addr));
    multicast_addr.sin_family = AF_INET;
    multicast_addr.sin_addr.s_addr = inet_addr(MULTICAST_ADDR); // Multicast IP address
    multicast_addr.sin_port = htons(PORT);

    if(sendto(sockfd, data , 17, 0, (struct sockaddr *) &multicast_addr, sizeof(multicast_addr)) < 0){
        perror("sendto");
        close(sockfd); // Close the socket on error
        exit(1);
    }
    close(sockfd);
}
