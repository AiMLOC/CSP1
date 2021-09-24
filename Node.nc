/*
 * ANDES Lab - University of California, Merced
 * This class provides the basic functions of a network node.
 *
 * @author UCM ANDES Lab
 * @date   2013/09/03
 *Kshav ayer
 */

typedef struct Neigbor{

   uint16_t srcNode;
    uint16_t Age;

} Neighbor;

#include <Timer.h>
#include "includes/command.h"
#include "includes/packet.h"
#include "includes/CommandMsg.h"
#include "includes/sendInfo.h"
#include "includes/channels.h"
//new 
#include "includes/socket.h"

module Node{
   uses interface Boot;

   uses interface SplitControl as AMControl;
   uses interface Receive;

   uses interface SimpleSend as Sender;

   uses interface CommandHandler;


   //new 
   uses interface List<pack> as PacketList; 

   uses interface List<Neighbor> as NeighborList;

}

implementation{
   pack sendPackage;

   // Prototypes
   void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t Protocol, uint16_t seq, uint8_t *payload, uint8_t length);
   //New PT
   void findNeighbors();
   void addPack(pack Package);
   bool checkForPack(pack *Package);
   bool isNeighbor(uint16_t src);

   event void Boot.booted(){
      call AMControl.start();

      dbg(GENERAL_CHANNEL, "Booted\n");
   }

   event void AMControl.startDone(error_t err){
      if(err == SUCCESS){
         dbg(GENERAL_CHANNEL, "Radio On\n");
      }else{
         //Retry until successful
         call AMControl.start();
      }
   }

   event void AMControl.stopDone(error_t err){}

   event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){
      dbg(GENERAL_CHANNEL, "Packet Received\n");
      if(len==sizeof(pack)){
         pack* myMsg=(pack*) payload;
         dbg(GENERAL_CHANNEL, "Package Payload: %s\n", myMsg->payload);
         return msg;
      }
      dbg(GENERAL_CHANNEL, "Unknown Packet Type %d\n", len);
      return msg;
   }


   event void CommandHandler.ping(uint16_t destination, uint8_t *payload){
      dbg(GENERAL_CHANNEL, "PING EVENT \n");
      makePack(&sendPackage, TOS_NODE_ID, destination, 0, 0, 0, payload, PACKET_MAX_PAYLOAD_SIZE);
      call Sender.send(sendPackage, destination);
   }

   event void CommandHandler.printNeighbors(){}

   event void CommandHandler.printRouteTable(){}

   event void CommandHandler.printLinkState(){}

   event void CommandHandler.printDistanceVector(){}

   event void CommandHandler.setTestServer(){}

   event void CommandHandler.setTestClient(){}

   event void CommandHandler.setAppServer(){}

   event void CommandHandler.setAppClient(){}

   void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t protocol, uint16_t seq, uint8_t* payload, uint8_t length){
      Package->src = src;
      Package->dest = dest;
      Package->TTL = TTL;
      Package->seq = seq;
      Package->protocol = protocol;
      memcpy(Package->payload, payload, length);

   }

   //Added FLOODING + NEIGHBOR DISCOVERY
      void addPack(pack Package){
      call PacketList.pushback(Package);     // Add the packet to the front of the packet list
   }

   bool checkForPack(pack* Package){
      pack Temp;
      uint16_t i;
      for(i = 0; i < call PacketList.size(); i++){
         Temp = call PacketList.get(i);
         if(Temp.src == Package->src && Temp.seq == Package->seq && Temp.dest == Package->dest){    // If the source/dest and sequence numbers are equal, we've seen this packet before
            return TRUE;
         }

      }

      return FALSE;                                                         // If the packet isn't found, return false
   }

   bool isNeighbor(uint16_t src){
      if(!call NeighborList.isEmpty()){                                     // Check to see if the neighbor exists in the neighbor list
         uint16_t i, size = call NeighborList.size();
         Neighbor neighbor;
         for(i = 0; i < size; i++){
             neighbor = call NeighborList.get(i);
             if(neighbor.srcNode == src){
                 neighbor.Age = 0;
                 return TRUE;
             }
         }

      }

      return FALSE;
   }

   void findNeighbors(){
        char * message;
        Neighbor neighbor, temp;
        uint16_t i, size = call NeighborList.size();
        for(i = 0; i < size; i++){                           // Age each neighbor on each neighbor discovery
            neighbor = call NeighborList.get(i);             // Really messy, but this can't be done with pointers since nesC only allows static allocation
            neighbor.Age++;                                  // Since we have to call a function to retrieve each neighbor, we can't directly change the age value
            call NeighborList.remove(i);                     // i.e. call NeighborList.get(i).Age++;
            call NeighborList.pushback(neighbor);
        }
        for(i = 0; i < size; i++){
            temp = call NeighborList.get(i);
            if(temp.Age > 3){                                // If neighbor is missing for 3 consecutive calls, drop them
                call NeighborList.remove(i);
                size--;
                i--;
            }
        }
        // dbg(NEIGHBOR_CHANNEL, "Sending discovery packets to neighbors...\n");
        message = "foobar";
        makePack(&sendPackage, TOS_NODE_ID, AM_BROADCAST_ADDR, 2, PROTOCOL_PING, 1,     // Send the neighbor discovery packet(dest = AM_BROADCAST_ADDR) 
                (uint8_t *)message,(uint8_t)sizeof(message));

        addPack(sendPackage);
        call Sender.send(sendPackage, AM_BROADCAST_ADDR);
    }

}
