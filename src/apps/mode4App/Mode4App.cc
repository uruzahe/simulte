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
#include "apps/cpm/PduMakeInfo_m.h"
// ----- End My Code -----


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
        std::cout << __func__ << ", sumo_id: " << sumo_id << " is loaded. " << std::endl;
        appQueue = {};

        // ----- App Layer -----
        _cams_ptr = new CAMs;
        _cams_send_ptr = new CAMSendHandler;
        _cams_recv_ptr = new CAMRecvHandler;

        _pos_ptr = new PerceivedObjectes;
        _pos_send_ptr = new POSendHandler;
        _pos_recv_ptr = new PORecvHandler;

        // ----- Virtual Network Layer -----
        _network_ptr = new VirtualGeoNetwork;

        // ----- Virtual Access Layer Queue -----
        _sdu_tx_ptr = new VirtualTxSduQueue;
        _sdu_rx_ptr = new VirtualRxSduQueue;
        _pdu_sender = new cMessage("_pdu_sender");
        _pdu_interval = 0.020;

        // ----- initialize -----
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

        double delay = TTI * intuniform(0, 1.0 / TTI, 0);
        scheduleAt((simTime() + delay).trunc(SIMTIME_US), selfSender_);
        scheduleAt((simTime() + delay).trunc(SIMTIME_US), _pdu_sender);
    }
}


void Mode4App::SdusHandler(std::vector<json> sdus, double send_time, double recv_time)
{
  // std::cout << "begin: " << __func__ << std::endl;

  int status_flag = 0;

  for (auto itr = sdus.begin(); itr != sdus.end(); itr++) {
    status_flag = (*itr)["status_flag"].get<int>();

    if (status_flag == 11) {
      myHandleLowerMessage((*itr)["payload"].get<std::string>(), (*itr)["type"].get<std::string>(), send_time, recv_time);

    } else {
      json packet = _sdu_rx_ptr->enque_and_decode((*itr));

      if (packet != NULL) {
        myHandleLowerMessage(packet["payload"].get<std::string>(), packet["type"].get<std::string>(), send_time, recv_time);
      }
    }
  }

  // std::cout << "end: " << __func__ << std::endl;
}


void Mode4App::myHandleLowerMessage(std::string payload, std::string type, double send_time, double recv_time)
{
  // std::cout << "begin: " << __func__ << std::endl;

  json recv_data;
  recv_data["payload"] = payload;
  recv_data["send_time"] = send_time;
  recv_data["recv_time"] = recv_time;

  // std::cout << "payload: " << payload << ", type: " << type << std::endl;

  if (type == "cam") {
    string_vector2file(cams_recv_json_file_path(carlaVeinsDataDir, sumo_id), { recv_data.dump() });

  } else if (type == "cpm") {
    string_vector2file(objects_recv_json_file_path(carlaVeinsDataDir, sumo_id), { recv_data.dump() });

  } else if (type == "pdu") {
    SdusHandler(json::parse(payload)["sdus"].get<std::vector<json>>(), send_time, recv_time);

  } else {
    assert(type == "cam" || type == "cpm" || type == "pdu");

  }

  // std::cout << "end: " << __func__ << std::endl;
}

void Mode4App::handleLowerMessage(cMessage* msg)
{
    // std::cout << "received msg: " << msg->getName() << std::endl;
    if (msg->isName("CBR")) {
        Cbr* cbrPkt = check_and_cast<Cbr*>(msg);
        double channel_load = cbrPkt->getCbr();
        std::cout << __func__ << ", cbr: " << channel_load << std::endl;
        emit(cbr_, channel_load);
        delete cbrPkt;

    } else if (msg->isName("PduMakeInfo")){
      PduMakeInfo* pdu_make_info_pkt = check_and_cast<PduMakeInfo*>(msg);
      double start_time = pdu_make_info_pkt->getStartTime();
      double rri = pdu_make_info_pkt->getRri();

      // std::cout << "start_time: " << start_time << ", rri: " << rri << std::endl;
      cancelEvent(selfSender_);
      cancelEvent(_pdu_sender);

      _pdu_interval = ((double) rri) / 1000.0;

      scheduleAt(start_time, selfSender_);
      scheduleAt(start_time, _pdu_sender);
    } else {
        // ----- Begin My Code -----
        if (veins::VeinsCarlaPacket* vc_pkt = dynamic_cast<veins::VeinsCarlaPacket*>(msg)) {
          // std::cout << "vc_pkt" << std::endl;
          myHandleLowerMessage((std::string) vc_pkt->getPayload(), (std::string) vc_pkt->getType(), vc_pkt->getTimestamp().dbl(), simTime().dbl());

        } // ----- End My Code -----
        else {
          // std::cout << "not vc_pkt" << std::endl;
            AlertPacket* pkt = check_and_cast<AlertPacket*>(msg);
            if (pkt == 0) { throw cRuntimeError("Mode4App::handleMessage - FATAL! Error when casting to AlertPacket"); }
        }

        simtime_t delay = simTime() - msg->getTimestamp();
        emit(delay_, delay);
        emit(rcvdMsg_, (long)1);
        // EV << "Mode4App::handleMessage - Packet received: SeqNo[" << msg->getSno() << "] Delay[" << delay << "]" << endl;

        delete msg;
    }
}

void Mode4App::handleSelfMessage(cMessage* msg)
{
    if (!strcmp(msg->getName(), "selfSender")){
        if (sendBeacons) {
            // std::cout << simTime() << std::endl;
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

    } else if (!strcmp(msg->getName(), "_pdu_sender")) {
      if (isSduQueueEmpty()) {
        double minimum_Bps = _sdu_tx_ptr->minimum_Bps(simTime().dbl());
        if (0 < minimum_Bps) {
          json pdu_info = _sdu_tx_ptr->Bps2packet_size_and_rri(minimum_Bps);
          std::cout << pdu_info << std::endl;
          json pdu = _sdu_tx_ptr->generate_PDU(pdu_info["size"].get<int>(), simTime().dbl());

          SendPacket(pdu.dump(), "pdu", pdu["size"].get<int>(), pdu["duration"].get<int>(), pdu_info);
        }
      }

      scheduleAt(simTime() + _pdu_interval, _pdu_sender);

    } else {
      throw cRuntimeError("Mode4App::handleMessage - Unrecognized self message");
    }
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

void Mode4App::SendPacket(std::string payload, std::string type, int payload_byte_size, int duration_ms, json pdu_info={})
{
  if (type == "pdu") {

    try {
      // std::cout << "payload: " << payload << "\n, size: " << payload_byte_size << "\n, duration_ms: " << duration_ms << std::endl;
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
      lteControlInfo->setDuration(duration_ms);
      lteControlInfo->setCreationTime(simTime());
      lteControlInfo->setMyChannelNum(pdu_info["ch"].get<int>());
      lteControlInfo->setMyRri(pdu_info["rri"].get<double>() * 10);

      packet->setControlInfo(lteControlInfo);

      Mode4BaseApp::sendLowerPackets(packet);
      // ----- End Population -----
    } catch (...) {
        // std::cout << "appQueue error: "<< payload.c_str() << "." << std::endl;
    }

  } else if (type == "cam" || type == "cpm") {
    json packet;
    packet["payload"] = payload;
    packet["type"] = type;
    packet["size"] = payload_byte_size;
    packet["size"] = 750 * 2;
    packet["sender_id"] = sumo_id;
    packet["max_duration"] = 100;
    packet["expired_time"] = simTime().dbl() + 0.1;

    if (type == "cam") {
      packet["priority"] = 2;
    } else if (type == "cpm") {
      packet["priority"] = 3;
    } else {
      assert(type == "cam" || type == "cpm");
    }

    _sdu_tx_ptr->enque(packet);

    // ----
  } else {
    assert(type == "cam" || type == "cpm" || type == "pdu");
  }
}

void Mode4App::syncCarlaVeinsData(cMessage* msg)
{
  double target_start_time = simTime().dbl() - period_.dbl();
  double target_end_time = simTime().dbl();

  if (is_dynamic_simulation) {
    loadCarlaVeinsData(false);
  }

  std::vector<json> target_cams = _cams_send_ptr->filter_cams_by_etsi(_cams_ptr->data_between_time(target_start_time, target_end_time));

  if (!target_cams.empty()) {
    json packet = _cams_send_ptr->convert_payload_and_size(target_cams[0], size_);

    SendPacket(packet["payload"].get<std::string>(), "cam", packet["size"].get<int>(), duration_);

    // return;
  }

  std::vector<json> target_pos = _pos_send_ptr->filter_pos_by_etsi(_pos_ptr->data_between_time(target_start_time, target_end_time));

  if (!target_pos.empty()) {
    json packet = _pos_send_ptr->convert_payload_and_size(target_pos, sensor_num, max_cpm_size);
    // // std::cout << packet["payload"].get<std::string>() << std::endl;
    if (0 < packet["size"].get<int>()) { // size 0 means that there are no enough size to contain perceived_objects.
      SendPacket(packet["payload"].get<std::string>(), "cpm", packet["size"].get<int>(), duration_);
    }

    // return;
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
