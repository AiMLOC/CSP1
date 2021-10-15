/*
 * ANDES Lab - University of California, Merced
 * This class provides the basic functions of a network node.
 *
 * @author UCM ANDES Lab
 * @date   2013/09/03
 *Kshav ayer
 */

#define INFINITY      9999
#define MAX           20

typedef struct Neighbor{

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

   //new2
   uses interface Hashmap<uint16_t> as RoutingTable;
   

}

implementation{
   pack sendPackage;
   //NEWPT2
   uint16_t seqNum = 0;           // Increment the seqNum for each new pakcet
   uint16_t LSTable[MAX][MAX];
   // Prototypes
   void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t Protocol, uint16_t seq, uint8_t *payload, uint8_t length);
   //New PT
   void discoverNeighbors();
   void addPacket(pack Package);
   bool packageCheck(pack *Package);
   bool checkNeighbor(uint16_t src);
   //New PT2
   void initLSTable(); 
   void sendLSPacket();
   void updateLSTable(uint8_t * payload, uint16_t source);
   void printLSTable();
   void dijkstra();
   uint16_t minDist(uint16_t dist[], bool sptSet[]);
   void printRoutingTable();



   event void Boot.booted(){
      call AMControl.start();

      dbg(GENERAL_CHANNEL, "Booted\n");
   }

   event void AMControl.startDone(error_t err){
      if(err == SUCCESS){
         dbg(GENERAL_CHANNEL, "Radio On\n");
         initLSTable(); //Link State Table
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

//NEW
      event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){
      
      if(len==sizeof(pack)){
         pack* myMsg=(pack*) payload;
         if(packageCheck(myMsg) || myMsg->TTL == 0){ 
             // dbg(GENERAL_CHANNEL, "Dropping the packet\n");
         }else if(myMsg->dest == TOS_NODE_ID){  
  
             //dbg(FLOODING_CHANNEL, "Package Payload: %s :: Package Source: Node %d\n", myMsg->payload, myMsg->src);     // If the packet arrived at the correct destination output the payload   
              
             switch(myMsg->protocol){  

                case PROTOCOL_PING:
                    makePack(&sendPackage, TOS_NODE_ID, myMsg->src, MAX_TTL, PROTOCOL_PINGREPLY, seqNum, 
                            (uint8_t *) myMsg->payload, sizeof(myMsg->payload));
                    seqNum++;

                    addPacket(sendPackage);
                    if(call RoutingTable.get(myMsg->src)){
                        dbg(ROUTING_CHANNEL, "Path found, sending reply to next hop %d\n", call RoutingTable.get(myMsg->src));
                        call Sender.send(sendPackage, call RoutingTable.get(myMsg->src));
                    }                  
                    else
                        dbg(ROUTING_CHANNEL, "Path not found, cancelling reply\n");

                    break;                                                                                              // Send a ping reply to the source using flooding                           
            
                case PROTOCOL_PINGREPLY:
                    dbg(FLOODING_CHANNEL, "Received the ping reply from %d\n", myMsg->src);
                    break;                                                                                              // Output if the ping reply was received  

            }

         }else if(myMsg->dest == AM_BROADCAST_ADDR){

             switch(myMsg->protocol){
                Neighbor neighbor;                                                                                      // Broadcast packets will help us find neighbors    
                case PROTOCOL_PING:
                    // dbg(NEIGHBOR_CHANNEL, "Received ping from neighbor Node %d\n", myMsg->src);                   
                    makePack(&sendPackage, TOS_NODE_ID, AM_BROADCAST_ADDR, MAX_TTL, PROTOCOL_PINGREPLY, 
                            myMsg->seq, (uint8_t *)myMsg->payload, sizeof(myMsg->payload));                     

                    addPacket(sendPackage);
                    call Sender.send(sendPackage, myMsg->src);
                    break;

                 case PROTOCOL_PINGREPLY:
                    if(!checkNeighbor(myMsg->src)){
                        neighbor.srcNode = myMsg->src;
                        neighbor.Age = 0;                                                                             // If the neighbor is not on our list put them on it
                        call NeighborList.pushback(neighbor);
                        LSTable[TOS_NODE_ID - 1][myMsg->src - 1] = 1;
                        sendLSPacket();
                    }
                    break;

                 case PROTOCOL_LINKSTATE:
                    updateLSTable((uint8_t *)myMsg->payload, myMsg->src);                                           // If it's a link state pack update the link state table                                   
                    makePack(&sendPackage, myMsg->src, myMsg->dest, myMsg->TTL, myMsg->protocol,
                            myMsg->seq, (uint8_t *)myMsg->payload, sizeof(myMsg->payload));

                    addPacket(sendPackage);
                    call Sender.send(sendPackage, AM_BROADCAST_ADDR);
                    break;
            }
    
         }else{     
            makePack(&sendPackage, myMsg->src, myMsg->dest, myMsg->TTL, myMsg->protocol, myMsg->seq, 
                    (uint8_t *)myMsg->payload, sizeof(myMsg->payload));

            addPacket(sendPackage);

            if(call RoutingTable.get(myMsg->dest)){
                // dbg(ROUTING_CHANNEL, "Route found, forwarding to %d\n", call RoutingTable.get(myMsg->dest));
                call Sender.send(sendPackage, call RoutingTable.get(myMsg->dest));
            }else{
                dbg(ROUTING_CHANNEL, "Route not found...\n");
            }
        
         }

             return msg;
     }
         dbg(GENERAL_CHANNEL, "Unknown Packet Type %d\n", len);
         return msg;
   }  


   event void CommandHandler.ping(uint16_t destination, uint8_t *payload){
      dbg(GENERAL_CHANNEL, "PING EVENT \n");
      makePack(&sendPackage, TOS_NODE_ID, destination, 0, 0, 0, payload, PACKET_MAX_PAYLOAD_SIZE);
      //call Sender.send(sendPackage, destination); Moved to routingtabbelow

      //added
      seqNum++; 
      
      if(call RoutingTable.get(destination)){
         dbg(ROUTING_CHANNEL, "Sending to next hop %d\n", call RoutingTable.get(destination));
         call Sender.send(sendPackage, call RoutingTable.get(destination));
      }
      else{
         dbg(ROUTING_CHANNEL, "Route to destination not found...\n");
      }                                                                     

   }

   event void CommandHandler.printNeighbors(){}

   event void CommandHandler.printRouteTable(){
      printRoutingTable(); //added
   }

   event void CommandHandler.printLinkState(){
      printLSTable(); //added
   }

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
        dbg(NEIGHBOR_CHANNEL, "Discovery Packets -> Neighbor\n");//DEBUG
        
        message = "swag";  //Packet message 
        makePack(&sendPackage, TOS_NODE_ID, AM_BROADCAST_ADDR, 2, PROTOCOL_PING, 1,     //Run makePack with vars to send discovery packet to AM ADDR
                (uint8_t *)message,(uint8_t)sizeof(message));

        addPacket(sendPackage);
        call Sender.send(sendPackage, AM_BROADCAST_ADDR);   //send
    }



   //NEW Added Table Neighbors + Link-state flooding + Shortest-path Dijkstra + Forwarding   
    void initLSTable(){
        uint16_t i, j;
        for(i = 0; i < MAX; i++){
            for(j = 0; j < MAX; j++){
                    LSTable[i][j] = INFINITY;                           // Initialize all link state table values to infinity(20)
            }
        }
    }

    void sendLSPacket(){
        char payload[255];
        char tempC[127];
        uint16_t i, size = call NeighborList.size();            // Construct the link state packet by concatenating the neighborlist to the payload
        Neighbor neighbor;      
        for(i = 0; i < size; i++){
            neighbor = call NeighborList.get(i);
            sprintf(tempC, "%d", neighbor.srcNode);
            strcat(payload, tempC);
            strcat(payload, ",");
        }
        
        makePack(&sendPackage, TOS_NODE_ID, AM_BROADCAST_ADDR, 50, PROTOCOL_LINKSTATE, seqNum,
                (uint8_t *) payload, (uint8_t)sizeof(payload));

        seqNum++;
        addPacket(sendPackage);
        call Sender.send(sendPackage, AM_BROADCAST_ADDR);
    }

    void updateLSTable(uint8_t * payload, uint16_t source){
        uint8_t * temp = payload;
        uint16_t length = strlen((char *)payload);              // Update the link state table neighbor pairs upon receiving a link state packet
        uint16_t i = 0;
        char buffer[5];
        while (i < length){
            if(*(temp + 1) == ','){
                memcpy(buffer, temp, 1);
                temp += 2;
                i += 2;
                buffer[1] = '\0';
            }else if(*(temp + 2) == ','){
               memcpy(buffer, temp, 2);
                temp += 3;
                i += 3;
                buffer[2] = '\0';
            }
            
                LSTable[source - 1][atoi(buffer) - 1] = 1;
        }

        dijkstra();
    }

    void printLSTable(){
        uint16_t i;                                            // Print out the neighbor pairs in the local link state table
        uint16_t j;
        for(i = 0; i < 20; i++){
            for(j = 0; j < 20; j++){
                if(LSTable[i][j] == 1)
                    dbg(ROUTING_CHANNEL, "Neighbors: %d and %d\n", i + 1, j + 1);
            }
        }
    }

    uint16_t minDist(uint16_t dist[], bool sptSet[]){
        uint16_t min = INFINITY, minIndex = 18, i;
        for(i = 0; i < MAX; i++){
            if(sptSet[i] == FALSE && dist[i] < min)
                min = dist[i], minIndex = i;
        }
        return minIndex;
    }

    void dijkstra(){
        uint16_t myID = TOS_NODE_ID - 1, i, count, v, u;
        uint16_t dist[MAX];
        bool sptSet[MAX];
        int parent[MAX];
        int temp;

        for(i = 0; i < MAX; i++){
            dist[i] = INFINITY;
            sptSet[i] = FALSE;
            parent[i] = -1;   
        }

        dist[myID] = 0;

        for(count = 0; count < MAX - 1; count++){
            u = minDist(dist, sptSet);
            sptSet[u] = TRUE;

            for(v = 0; v < MAX; v++){
                if(!sptSet[v] && LSTable[u][v] != INFINITY && dist[u] + LSTable[u][v] < dist[v]){
                    parent[v] = u;
                    dist[v] = dist[u] + LSTable[u][v];
                }
            }           
        }

        for(i = 0; i < MAX; i++){
            temp = i;
            while(parent[temp] != -1  && parent[temp] != myID && temp < MAX){
                temp = parent[temp];
            }
            if(parent[temp] != myID){
                call RoutingTable.insert(i + 1, 0);
            }
            else
                call RoutingTable.insert(i + 1, temp + 1);
        }
    }

    void printRoutingTable(){
        
        uint16_t size = call RoutingTable.size(), i, output;
        for(i = 0; i < size; i++){
            output = call RoutingTable.get((uint32_t) i);
            dbg(ROUTING_CHANNEL, "Key: %d\t Next Hop: %d\n", i, output);
        }

        dbg(ROUTING_CHANNEL, "\n");
    }
    }