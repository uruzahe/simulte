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

        // ----- My Code, Begin. -----
        EV_TRACE << "My Code" << std::endl;

        max_cpm_size = par("max_cpm_size").intValue();
        carlaVeinsDataDir = par("carlaVeinsDataDir").stringValue();
        sensor_num = par("sensor_num").intValue();
        sendCPM = par("sendCPM").boolValue();
        sendBeacons = par("sendBeacons").boolValue();
        is_dynamic_simulation = par("is_dynamic_simulation").boolValue();
        carlaTimeStep = par("carlaTimeStep").doubleValue();

        sumo_id = mobility->external_id;
        std::cout << "sumo_id: " << sumo_id << " is loaded. " << std::endl;
        appQueue = {};

        _cams_ptr = new CAMs;
        _cams_send_ptr = new CAMSendHandler;
        _cams_recv_ptr = new CAMRecvHandler;

        _pos_ptr = new PerceivedObjectes;
        _pos_send_ptr = new POSendHandler;
        _pos_recv_ptr = new PORecvHandler;


        if (!sendBeacons && !is_dynamic_simulation) {
          loadCarlaVeinsData(true);
        }

        if (!sendBeacons) {
            touch(cams_recv_json_file_path(carlaVeinsDataDir, sumo_id));
            touch(objects_recv_json_file_path(carlaVeinsDataDir, sumo_id));
        }

        generatedCPMs = 0;
        receivedCPMs = 0;
        // ----- My Code, End. -----

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
        if (veins::VeinsCarlaPacket* vc_pkt = dynamic_cast<veins::VeinsCarlaPacket*>(msg)) {
            receivedCPMs++;
            obtainedCPMs.push_back((std::string) vc_pkt->getPayload());

            // emit statistics
            simtime_t delay = simTime() - vc_pkt->getTimestamp();
            emit(delay_, delay);
            emit(rcvdMsg_, (long)1);
//            std::cout << sumo_id << " received cpm messages" << std::endl;
//            std::cout << "payloads: " << vc_pkt->getPayload() << std::endl;
            EV << "Mode4App::handleMessage - CPM Packet received: SeqNo[" << vc_pkt->getSno() << "] Delay[" << delay << "]" << endl;

            if ((std::string) vc_pkt->getType() == "cam") {
              string_vector2file(cams_recv_json_file_path(carlaVeinsDataDir, sumo_id), { (std::string) vc_pkt->getPayload() });

            } else if ((std::string) vc_pkt->getType() == "cpm") {
              json payload = json::parse(vc_pkt->getPayload());
              string_vector2file(objects_recv_json_file_path(carlaVeinsDataDir, sumo_id), payload.get<std::vector<std::string>>());
            }

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

  _pos_ptr->load_json_strs(
    file2string_vector(
      objects_json_file_path(carlaVeinsDataDir, sumo_id),
      read_only
    )
  );

}

bool Mode4App::isSduQueueEmpty()
{
    if (UmTxEntityPtr == nullptr && veins::FindModule<UmTxEntity*>::findSubModule(getParentModule())) {
      UmTxEntityPtr = veins::FindModule<UmTxEntity*>::findSubModule(getParentModule());
    }

    // ----- send packet from targetCPMs -----
    if (UmTxEntityPtr == nullptr || UmTxEntityPtr->isSduQueueEmpty()) {
      return true;
    } else {
      return false;
    }
}

void Mode4App::SendPacket(std::string payload, std::string type, int payload_byte_size)
{
  try {
      veins::VeinsCarlaPacket* packet = new veins::VeinsCarlaPacket();

      packet->setPayload(payload.c_str());
      packet->setBitLength(payload_byte_size * 8);
      packet->setType(type.c_str());

      // ----- Begin Population -----
      packet->setTimestamp(simTime());
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
      // ----- End Population -----
  } catch (...) {
      std::cout << "appQueue error: "<< payload.c_str() << "." << std::endl;
  }
}

void Mode4App::syncCarlaVeinsData(cMessage* msg)
{
  double target_start_time = simTime().dbl() - period_.dbl();
  double target_end_time = simTime().dbl();

  if (is_dynamic_simulation) {
    loadCarlaVeinsData(false);
  }

  // ----- send packets if SduQueue is empty. -----
  // ----- This is because the Sduqueue cannot store multiple packets in default codes. -----
  if (!isSduQueueEmpty()) {
    return;
  }

  std::vector<json> target_cams = _cams_send_ptr->filter_cams_by_etsi(_cams_ptr->data_between_time(target_start_time, target_end_time));

  if (!target_cams.empty()) {
    json packet = _cams_send_ptr->convert_payload_and_size(target_cams[0], size_);

    SendPacket(packet["payload"].get<std::string>(), "cam", packet["size"].get<int>());

    return;
  }

  std::vector<json> target_pos = _pos_send_ptr->filter_pos_by_etsi(_pos_ptr->data_between_time(target_start_time, target_end_time));

  if (!target_pos.empty()) {
    json packet = _pos_send_ptr->convert_payload_and_size(target_pos, sensor_num, max_cpm_size);
    // std::cout << packet["payload"].get<std::string>() << std::endl;
    if (0 < packet["size"].get<int>()) { // size 0 means that there are no enough size to contain perceived_objects.
      SendPacket(packet["payload"].get<std::string>(), "cpm", packet["size"].get<int>());
    }

    return;
  }
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
