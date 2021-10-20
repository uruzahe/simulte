//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

/**
 * Mode4App is a new application developed to be used with Mode 4 based simulation
 * Author: Brian McCarthy
 * Email: b.mccarthy@cs.ucc.ie
 */

#ifndef _LTE_MODE4APP_H_
#define _LTE_MODE4APP_H_

// My Code, Begin.
#include "veins_inet/veins_inet.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins_inet/VeinsInetMobility.h"
#include "stack/rlc/um/entity/UmTxEntity.h"

#include "apps/cpm/VeinsCarlaCpm_m.h"
#include "apps/cpm/VeinsCarlaPacket_m.h"
#include "apps/mode4App/CarlaVeinsUtil.h"


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <vector>

using veins::TraCIMobility;

// My Code, End.

#include "apps/mode4App/Mode4BaseApp.h"
#include "apps/alert/AlertPacket_m.h"
#include "corenetwork/binder/LteBinder.h"


class Mode4App : public Mode4BaseApp {

public:
    ~Mode4App() override;

protected:
    //sender
    int size_;
    int nextSno_;
    int priority_;
    int duration_;
    simtime_t period_;

    simsignal_t sentMsg_;
    simsignal_t delay_;
    simsignal_t rcvdMsg_;
    simsignal_t cbr_;

    cMessage *selfSender_;

    LteBinder* binder_;
    MacNodeId nodeId_;

   int numInitStages() const { return inet::NUM_INIT_STAGES; }

   /**
    * Grabs NED parameters, initializes gates
    * and the TTI self message
    */
   void initialize(int stage);

   void handleLowerMessage(cMessage* msg);


   /**
    * Statistics recording
    */
   void finish();

   /**
    * Main loop of the Mac level, calls the scheduler
    * and every other function every TTI : must be reimplemented
    * by derivate classes
    */
   void handleSelfMessage(cMessage* msg);

   /**
    * sendLowerPackets() is used
    * to send packets to lower layer
    *
    * @param pkt Packet to send
    */
   void sendLowerPackets(cPacket* pkt);

   //-----  My Code, Begin -----
   bool isSduQueueEmpty();
   void SendPacket(std::string payload, std::string type, int payload_byte_size, int duration, json pdu_info);
   void SdusHandler(std::vector<json> sdus, double send_time, double recv_time);
   // void SduHandler(json sdu);
   void myHandleLowerMessage(std::string payload, std::string type, double send_time, double recv_time);
   virtual void loadCarlaVeinsData(bool read_only);
   virtual void syncCarlaVeinsData(cMessage* msg);

   veins::VeinsInetMobility* mobility;
   UmTxEntity* UmTxEntityPtr;

   std::string carlaVeinsDataDir;
   bool is_dynamic_simulation;
   bool sendCPM;
   bool sendBeacons;
   cMessage* sendCPMEvt;
   double carlaTimeStep;
   std::string sumo_id;
   std::vector<std::string> obtainedCPMs;
   std::vector<std::string> reservedCPMs;

   // AppQueue, VirtualTxSduQueue, and VirtualRxSduQueue are used to control SDU queue.
   // In default codes of OpenCV2X, sdu fragments are removed if new packets are generated in the application layer (See enque method of UmTxEntity.cc).
   // Therefore, we have to generate packets when SDU queue is empty.
   std::vector<std::string> appQueue;
   VirtualTxSduQueue* _sdu_tx_ptr;
   VirtualRxSduQueue* _sdu_rx_ptr;
   simtime_t _pdu_interval;
   cMessage* _pdu_sender;

   std::string veinsLockFile;
   std::string veinsTxtFile;

   uint32_t generatedCPMs;
   uint32_t receivedCPMs;

   int sensor_num;
   int max_cpm_size;

   CAMs*  _cams_ptr;
   PerceivedObjectes* _pos_ptr;

   CAMSendHandler* _cams_send_ptr;
   POSendHandler* _pos_send_ptr;

   CAMRecvHandler* _cams_recv_ptr;
   PORecvHandler* _pos_recv_ptr;

   // ----- My Code, End -----

};

#endif
