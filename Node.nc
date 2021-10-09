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
   uses interface Timer<TMilli> as RandomDiscoverNTimer;
   
   uses interface List<pack> as PacketList; 

   uses interface List<Neighbor> as NeighborList;
   

}

implementation{
   pack sendPackage;

   // Prototypes
   void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t Protocol, uint16_t seq, uint8_t *payload, uint8_t length);
   //New PT
   void discoverNeighbors();
   void addPacket(pack Package);
   bool packageCheck(pack *Package);
   bool checkNeighbor(uint16_t src);

   event void Boot.booted(){
      call AMControl.start();

      dbg(GENERAL_CHANNEL, "Booted\n");
   }

   event void AMControl.startDone(error_t err){
      if(err == SUCCESS){
         dbg(GENERAL_CHANNEL, "Radio On\n");
         call RandomDiscoverNTimer.startPeriodic(1000);      //NEW Random* Qing T for discovering neighbors
      }else{
         //Retry until successful
         call AMControl.start();
      }
   }

   event void RandomDiscoverNTimer.fired(){
      discoverNeighbors();                                //NEW FUNC For every Q update list + drops
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

   //NEW Added FLOODING + NEIGHBOR DISCOVERY
      void addPacket(pack Package){
      call PacketList.pushback(Package);     // Push packet to front of PacketList
   }

   bool packageCheck(pack* Package){
      pack tmppcktls;
      uint16_t i;
      for(i = 0; i < call PacketList.size(); i++){
         tmppcktls = call PacketList.get(i);
         if(tmppcktls.src == Package->src && tmppcktls.seq == Package->seq && tmppcktls.dest == Package->dest){    // If packet indentifiers and equal then same packet
            return TRUE;
         }

      }

      return FALSE;                                                         // No packet = False
   }

   bool checkNeighbor(uint16_t src){
      if(!call NeighborList.isEmpty()){                                     // Neighbor Check
         uint16_t i, size = call NeighborList.size();
         Neighbor neighbor;
         for(i = 0; i < size; i++){
             neighbor = call NeighborList.get(i);
             if(neighbor.srcNode == src){
                 neighbor.Age = 0;
                 return TRUE;                                              //If neighbor exist return true
             }
         }

      }

      return FALSE;                                                         
   }

   void discoverNeighbors(){
        char * message;
        Neighbor neighbor, tmppcktls;
        uint16_t i, size = call NeighborList.size();
        for(i = 0; i < size; i++){                           // Use .age interface to number each neighbor when discovered
            neighbor = call NeighborList.get(i);             
            neighbor.Age++;                                  // Change age value through .age function
            call NeighborList.remove(i);                     // .remove interafce added manually (not included in skeleton)
            call NeighborList.pushback(neighbor);              //once remove push neighbor list back
        }
        for(i = 0; i < size; i++){
            tmppcktls = call NeighborList.get(i);
            if(tmppcktls.Age > 4){                                // ping 4 times and no response remove node
                call NeighborList.remove(i);
                size--;
                i--;
            }
        }
        dbg(NEIGHBOR_CHANNEL, "DEBUG: Discovery ->\n");//DEBUG
        
        message = "swag";  //Packet message 
        makePack(&sendPackage, TOS_NODE_ID, AM_BROADCAST_ADDR, 2, PROTOCOL_PING, 1,     //Run makePack with vars to send discovery packet to AM ADDR
                (uint8_t *)message,(uint8_t)sizeof(message));

        addPacket(sendPackage);
        call Sender.send(sendPackage, AM_BROADCAST_ADDR);   //send
    }

}
