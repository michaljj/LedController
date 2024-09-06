#ifndef MDNSHANDLER_H
#define MDNSHANDLER_H

#define MDNS_HOSTNAME "ledcontroller"
#define MDNS_INSTANCENAME "My LED Controller"

void mDNSHandler_StartMdnsService(uint16_t port);


#endif //MDNSHANDLER_H