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

#include "apps/mode4App/Mode4App.h"
#include "common/LteControlInfo.h"
#include "stack/phy/packet/cbr_m.h"

// ----- Begin My Code -----
#include <nlohmann/json.hpp>

using json = nlohmann::json;
// ----- End My Code -----

Define_Module(Mode4App);

void Mode4App::initialize(int stage)
{
    Mode4BaseApp::initialize(stage);
    if (stage==inet::INITSTAGE_LOCAL){
        // Register the node with the binder
        // Issue primarily is how do we set the link layer address

        // Get the binder
        binder_ = getBinder();

        // Get our UE
        cModule *ue = getParentModule();

        //Register with the binder
        nodeId_ = binder_->registerNode(ue, UE, 0);

        // Register the nodeId_ with the binder.
        binder_->setMacNodeId(nodeId_, nodeId_);

        // ----- Begin My Code -----
        if (veins::FindModule<veins::VeinsInetMobility*>::findSubModule(getParentModule())) {
            mobility = veins::FindModule<veins::VeinsInetMobility*>::findSubModule(getParentModule());
        }
        else {
            mobility = nullptr;
        }

        UmTxEntityPtr = nullptr;
        // ----- End My Code -----

    } else if (stage==inet::INITSTAGE_APPLICATION_LAYER) {
        selfSender_ = NULL;
        nextSno_ = 0;

        selfSender_ = new cMessage("selfSender");

        size_ = par("packetSize");
        period_ = par("period");
        priority_ = par("priority");
        duration_ = par("duration");

        sentMsg_ = registerSignal("sentMsg");
        delay_ = registerSignal("delay");
        rcvdMsg_ = registerSignal("rcvdMsg");
        cbr_ = registerSignal("cbr");

        // My Code, Begin.
        EV_TRACE << "My Code" << std::endl;
        carlaVeinsDataDir = par("carlaVeinsDataDir").stringValue();
        sendCPM = par("sendCPM").boolValue();
        sendBeacons = par("sendBeacons").boolValue();
        is_dynamic_simulation = par("is_dynamic_simulation").boolValue();
        carlaTimeStep = par("carlaTimeStep").doubleValue();

        sumo_id = mobility->external_id;
        appQueue = {};

        // ----- old codes. -----
        // obtainedCPMs = {};
        // if (is_dynamic_simulation) {
        //   reservedCPMs = {} ;
        // } else {
        //   reservedCPMs = get_cpm_payloads_from_carla(sumo_id, carlaVeinsDataDir, true);
        // }

        _cams_ptr = new CAMs;
        _perceived_objects_ptr = new PerceivedObjectes;

        if (!is_dynamic_simulation) {
          loadCarlaVeinsData(true);
        }

        generatedCPMs = 0;
        receivedCPMs = 0;
        // My Code, End.

        double delay = 0.001 * intuniform(0, 1000, 0);
        scheduleAt((simTime() + delay).trunc(SIMTIME_MS), selfSender_);
    }
}

void Mode4App::handleLowerMessage(cMessage* msg)
{
    if (msg->isName("CBR")) {
        Cbr* cbrPkt = check_and_cast<Cbr*>(msg);
        double channel_load = cbrPkt->getCbr();
        emit(cbr_, channel_load);
        delete cbrPkt;
    } else {
        // ----- Begin My Code -----
        if (veins::VeinsCarlaCpm* cpm = dynamic_cast<veins::VeinsCarlaCpm*>(msg)) {
            receivedCPMs++;
            obtainedCPMs.push_back((std::string) cpm->getPayload());

            // emit statistics
            simtime_t delay = simTime() - cpm->getTimestamp();
            emit(delay_, delay);
            emit(rcvdMsg_, (long)1);
            // std::cout << sumo_id << " received cpm messages" << std::endl;
            // std::cout << "payloads: " << cpm->getPayload() << std::endl;
            EV << "Mode4App::handleMessage - CPM Packet received: SeqNo[" << cpm->getSno() << "] Delay[" << delay << "]" << endl;
        } // ----- End My Code -----
        else {
            AlertPacket* pkt = check_and_cast<AlertPacket*>(msg);

            if (pkt == 0) {
                throw cRuntimeError("Mode4App::handleMessage - FATAL! Error when casting to AlertPacket");
            }

            // emit statistics
            simtime_t delay = simTime() - pkt->getTimestamp();
            emit(delay_, delay);
            emit(rcvdMsg_, (long)1);

            EV << "Mode4App::handleMessage - Packet received: SeqNo[" << pkt->getSno() << "] Delay[" << delay << "]" << endl;
        }

        delete msg;
    }
}

void Mode4App::handleSelfMessage(cMessage* msg)
{
    if (!strcmp(msg->getName(), "selfSender")){
        if (sendBeacons) {
            std::cout << simTime() << std::endl;
            // Replace method
            AlertPacket* packet = new AlertPacket("Alert");
            packet->setTimestamp(simTime());
            packet->setByteLength(size_);
            packet->setSno(nextSno_);

            nextSno_++;

            auto lteControlInfo = new FlowControlInfoNonIp();

            lteControlInfo->setSrcAddr(nodeId_);
            lteControlInfo->setDirection(D2D_MULTI);
            lteControlInfo->setPriority(priority_);
            lteControlInfo->setDuration(duration_);
            lteControlInfo->setCreationTime(simTime());

            packet->setControlInfo(lteControlInfo);

            Mode4BaseApp::sendLowerPackets(packet);
        } else {
            syncCarlaVeinsData(msg);
        }


        emit(sentMsg_, (long)1);

        scheduleAt(simTime() + period_, selfSender_);
    }
    else
        throw cRuntimeError("Mode4App::handleMessage - Unrecognized self message");
}

// ----- Begin My Code -----
void Mode4App::loadCarlaVeinsData(bool read_only){

  _cams_ptr->load_json_strs(
    file2string_vector(
      cams_json_file_path(carlaVeinsDataDir, sumo_id),
      read_only
    )
  );

  _perceived_objects_ptr->load_json_strs(
    file2string_vector(
      objects_json_file_path(carlaVeinsDataDir, sumo_id),
      read_only
    )
  );

}

void Mode4App::syncCarlaVeinsData(cMessage* msg)
{

  if (is_dynamic_simulation) {
    loadCarlaVeinsData(false);
  }
  // ----- old code -----
  //
  //   std::vector<std::string> targetCPMs;
  //   double next_time_step = carlaTimeStep;
  //
  //   // ----- make targetCPMs -----
  //   if (is_dynamic_simulation) {
  //     // save received cpms
  //     set_cpm_payloads_for_carla(sumo_id, carlaVeinsDataDir, obtainedCPMs);
  //     obtainedCPMs.clear();
  //     obtainedCPMs.shrink_to_fit();
  //
  //     // send CPMs
  //     std::vector<std::string> new_payloads = get_cpm_payloads_from_carla(sumo_id, carlaVeinsDataDir, false);
  //
  //     for (auto payload = new_payloads.begin(); payload != new_payloads.end(); payload++) {
  //       targetCPMs.push_back(*payload);
  //     }
  //
  //   } else {
  //     auto payload = reservedCPMs.begin();
  //
  //     while (payload != reservedCPMs.end()) {
  //       try {
  //           json payload_json = json::parse(*payload);
  //
  //           double timestamp = payload_json["timestamp"].get<double>();
  //           double simtime = simTime().dbl();
  //           // std::cout << "sumo_id" << sumo_id << "simTime: " << simtime << " timestamp: " << timestamp << std::endl;
  //
  //           if (timestamp <= simtime - carlaTimeStep) {
  //             // std::cout << "The packet is too old, so erase it." << std::endl;
  //             reservedCPMs.erase(payload);
  //           } else if (simtime - carlaTimeStep < timestamp && timestamp <= simtime) {
  //             // std::cout << "The packet is created now, so send it." << std::endl;
  //             targetCPMs.push_back(*payload);
  //             reservedCPMs.erase(payload);
  //           } else {
  //             // std::cout << "The packet should be sent in the next timestamp, so break" << std::endl;
  //             break;
  //           }
  //       } catch (...) {
  //           std::cout << "reservedCPMs error: "<< (*payload).c_str() << "." << std::endl;
  //           continue;
  //       }
  //     }
  //   }
  //
  //   // ----- extend targetCPMs to AppQueue -----
  //   for (auto payload = targetCPMs.begin(); payload != targetCPMs.end(); payload++) {
  //     appQueue.push_back(*payload);
  //   }
  //
  //   if (UmTxEntityPtr == nullptr && veins::FindModule<UmTxEntity*>::findSubModule(getParentModule())) {
  //     UmTxEntityPtr = veins::FindModule<UmTxEntity*>::findSubModule(getParentModule());
  //   }
  //
  //   if (UmTxEntityPtr != nullptr) {
  //     std::cout << "Empty: " << UmTxEntityPtr->isSduQueueEmpty() << std::endl;
  //   }
  //
  //   // ----- send packet from targetCPMs -----
  //   if (UmTxEntityPtr == nullptr || UmTxEntityPtr->isSduQueueEmpty()) {
  //     auto payload = appQueue.begin();
  //
  //     while (payload != appQueue.end()) {
  //       try {
  //           json payload_json = json::parse(*payload);
  //
  //           veins::VeinsCarlaCpm* packet = new veins::VeinsCarlaCpm();
  //
  //           packet->setPayload((*payload).c_str());
  //           packet->setBitLength(payload_json["option"]["size"].get<int>() * 8);
  //
  //           // ----- Begin Population -----
  //           packet->setTimestamp(simTime());
  // //          packet->setByteLength(size_);
  //           packet->setSno(nextSno_);
  //           nextSno_++;
  //
  //           nextSno_++;
  //
  //           auto lteControlInfo = new FlowControlInfoNonIp();
  //
  //           lteControlInfo->setSrcAddr(nodeId_);
  //           lteControlInfo->setDirection(D2D_MULTI);
  //           lteControlInfo->setPriority(priority_);
  //           lteControlInfo->setDuration(duration_);
  //           lteControlInfo->setCreationTime(simTime());
  //
  //           packet->setControlInfo(lteControlInfo);
  //
  // //          std::cout << "payload: " << *payload << std::endl;
  //           Mode4BaseApp::sendLowerPackets(packet);
  //
  //           appQueue.erase(payload);
  //           break;
  //           // ----- End Population -----
  //       } catch (...) {
  //           std::cout << "appQueue error: "<< (*payload).c_str() << "." << std::endl;
  //           appQueue.erase(payload);
  //           continue;
  //       }
  //     }
  //   }
}
// ----- End My Code -----

void Mode4App::finish()
{
    cancelAndDelete(selfSender_);
}

Mode4App::~Mode4App()
{
    binder_->unregisterNode(nodeId_);
}
