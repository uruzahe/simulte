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
        _max_hop = par("max_hop").intValue();

        sumo_id = mobility->external_id;
        std::cout << ", sumo_id: " << sumo_id << " is loaded. " << std::endl;
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
        _cbf_resend = new cMessage("_cbf_resend");

        // ----- Virtual Access Layer Queue -----
        _sdu_tx_ptr = new VirtualTxSduQueue;
        _sdu_rx_ptr = new VirtualRxSduQueue;
        _pdu_sender = new cMessage("_pdu_sender");

        _pdu_interval = 0.020;

        _removeDataFromQueue = new cMessage("_removeDataFromQueue");
        // _removeDataFromQueue->setSchedulingPriority(10);

        _resource_selection = new cMessage("_resource_selection");
        _resource_selection->setSchedulingPriority(10);


        // ----- initialize -----
        if (!sendBeacons && !is_dynamic_simulation) {
          loadCarlaVeinsData(true);
        }

        if (!sendBeacons) {
            touch(grants_file_path(carlaVeinsDataDir, sumo_id));
            touch(cams_recv_json_file_path(carlaVeinsDataDir, sumo_id));
            touch(objects_recv_json_file_path(carlaVeinsDataDir, sumo_id));
            touch(dup_count_file_path(carlaVeinsDataDir, sumo_id));
            touch(cbr_file_path(carlaVeinsDataDir, sumo_id));
        }

        generatedCPMs = 0;
        receivedCPMs = 0;
        // ----- My Code, End. -----

        double delay = TTI * intuniform(0, 1.0 / TTI, 0);
        scheduleAt((simTime() + delay).trunc(SIMTIME_US), selfSender_);
        scheduleAt((simTime() + delay).trunc(SIMTIME_US), _pdu_sender);
    }
}


void Mode4App::StachSendPDU() {
  if (isSduQueueEmpty() && _sdu_tx_ptr->is_empty(simTime().dbl()) == false) {
    int pdu_size = _sdu_tx_ptr->_ch2size[_current_ch];
    json pdu = _sdu_tx_ptr->generate_PDU(pdu_size, simTime().dbl());
    json pdu_info = {{"ch", _current_ch}, {"rri", _current_rri}};

    // std::cout << __func__ << ", pdu: " << pdu << ", pdu_info: " << pdu_info << std::endl;
    // std::cout << __func__ << ", now: " << simTime() << ", pdu_time: " << _pdu_sender->getArrivalTime() << std::endl;
    SendPacket(pdu.dump(), "pdu", pdu["size"].get<int>(), pdu["duration"].get<double>(), pdu_info);
  }
}

json Mode4App::attach_headers(json packet, json geocast_header={}, json rlc_header={}) {
  // std::cout << packet << std::endl;
  // std::cout << geocast_header << std::endl;
  // std::cout << rlc_header << std::endl;

  packet["geocast"] = geocast_header["geocast"];
  packet["rlc"] = rlc_header["rlc"];
  // packet.merge_patch(geocast_header);
  // packet.merge_patch(rlc_header);

  return packet;
}

json Mode4App::FacilityHandler (std::string cmd, json packet={}) {
  json result = {};

  if (cmd == "FromGeocast") {
    return packet;

  } else if (cmd == "ToGeocast") {
    return this->GeoNetworkHandler("FromApp", packet);

  } else {
    throw cRuntimeError("Unknown cmd");
  }

  return result;
}


json Mode4App::GeoNetworkHandler (std::string cmd, json packet={}) {
  json result = {};

  if (cmd == "FromApp") {
    return this->RlcHandler("FromGeocast", _network_ptr->enque(packet, simTime().dbl()));

  } else if (cmd == "FromRlc") {
    _network_ptr->enque(packet, simTime().dbl());

    double resend_time = (int) (_network_ptr->CBF_resend_time(packet, mobility->getCurrentPosition(), simTime().dbl()) / TTI) * TTI;
    // std::cout << __func__ << ", resend_time: " << resend_time << ", packet: " << packet << std::endl;

    if (simTime().dbl() < resend_time) {
      // !!!!! Tips, change priority !!!!!
      packet["rlc"]["priority"] = 4;
      // !!!!! Tips, end !!!!!

      _network_ptr->resend_enque(resend_time, packet);
      _network_ptr->_resend_times.push_back(resend_time);

      this->GeoNetworkHandler("ReSendSchedule", {});
    }

    return this->FacilityHandler("FromGeocast", packet);

  } else if (cmd == "ReSend") {
    std::vector<json> resend_packets = _network_ptr->resend_deque(simTime().dbl());

    for (auto itr = resend_packets.begin(); itr != resend_packets.end(); itr++) {
      this->RlcHandler(
        "FromGeocast",
        _network_ptr->enque(
          _network_ptr->update_header(
            *itr,
            mobility->getCurrentPosition()
          ),
          simTime().dbl()
        )
      );
    }

    this->GeoNetworkHandler("ReSendSchedule", {});

  } else if (cmd == "ReSendSchedule") {
    if (0 < _network_ptr->_resend_times.size()) {
      std::sort(_network_ptr->_resend_times.begin(), _network_ptr->_resend_times.end(), std::greater<double>{});
      std::unique(_network_ptr->_resend_times.begin(), _network_ptr->_resend_times.end());

      cancelEvent(_cbf_resend);
      scheduleAt(_network_ptr->_resend_times.back(), _cbf_resend);

      _network_ptr->_resend_times.pop_back();
    }

  } else {
    throw cRuntimeError("Unknown cmd");

  }

  return result;
}

json Mode4App::RlcHandler (std::string cmd, json packet={}) {
  json result = {};

  if (cmd == "FromGeocast") {
    if (_sdu_tx_ptr->enque(_sdu_tx_ptr->update_header(packet, this->sumo_id), simTime().dbl())) {
      if (simTime() <= _pdu_sender->getArrivalTime() && _pdu_sender->getArrivalTime() <= simTime() + 0.001) {
        // ----- do nothing -----
      } else {
        cancelEvent(_resource_selection);
        scheduleAt(simTime(), _resource_selection);
      }
    }

  } else if (cmd == "ToGeocast") {
    return this->GeoNetworkHandler("FromRlc", packet);

  } else if (cmd == "FromPhy") {

  } else if (cmd == "ToPhy") {
    // std::cout << __func__ << simTime() << "ToPhy" << std::endl;
    this->StachSendPDU();

  } else {
    throw cRuntimeError("Unknown cmd");
  }

  return result;
}

void Mode4App::SdusHandler(std::vector<json> sdus, double send_time, double recv_time)
{
  // std::cout << "begin: " << __func__ << std::endl;

  int status_flag = 0;

  for (auto itr = sdus.begin(); itr != sdus.end(); itr++) {
    status_flag = (*itr)["status_flag"].get<int>();

    if (status_flag == 11) {
      myHandleLowerMessage((*itr)["payload"].get<std::string>(), (*itr)["type"].get<std::string>(), send_time, recv_time);
      this->RlcHandler("ToGeocast", (*itr));

    } else {
      json packet = _sdu_rx_ptr->enque_and_decode((*itr), simTime().dbl());

      if (packet != NULL) {
        myHandleLowerMessage(packet["payload"].get<std::string>(), packet["type"].get<std::string>(), send_time, recv_time);
        this->RlcHandler("ToGeocast", packet);

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
    ASSERT(type == "cam" || type == "cpm" || type == "pdu");

  }

  // std::cout << "end: " << __func__ << std::endl;
}

void Mode4App::handleLowerMessage(cMessage* msg)
{
    // std::cout << "received msg: " << msg->getName() << std::endl;
    if (msg->isName("CBR")) {
        Cbr* cbrPkt = check_and_cast<Cbr*>(msg);
        double channel_load = cbrPkt->getCbr();
        json log = { {"time", simTime().dbl() }, {"cbr", channel_load} };
        std::cout << __func__ << ", " << simTime() << ", cbr: " << channel_load << std::endl;
        this->_cbr_logs.push_back(log.dump());
        emit(cbr_, channel_load);
        delete cbrPkt;

    } else if (msg->isName("PduMakeInfo")){
      PduMakeInfo* pdu_make_info_pkt = check_and_cast<PduMakeInfo*>(msg);
      double start_time = pdu_make_info_pkt->getStartTime();
      double rri = pdu_make_info_pkt->getRri();


      json log = {
        {"time", simTime().dbl()},
        {"_current_ch", _current_ch},
        {"_current_rri", _current_rri},
        {"_new_ch", pdu_make_info_pkt->getCh()},
        {"_new_rri", pdu_make_info_pkt->getRri()},
        {"_new_start_time", start_time},
        {"_current_start_time", _pdu_sender->getArrivalTime().dbl()}
      };
      this->_grant_logs.push_back(log.dump());

      cancelEvent(_pdu_sender);

      _pdu_interval = rri / 1000.0;
      _current_ch = pdu_make_info_pkt->getCh();
      _current_rri = _pdu_interval.dbl();

      std::cout << __func__ << ", start_time: " << start_time << ", rri: " << _current_rri << ", ch: " << _current_ch << std::endl;
      // _pdu_sender->setSchedulingPriority(0);
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
          // cancelEvent(_resource_selection);
          // scheduleAt(simTime(), _resource_selection);
        }

        emit(sentMsg_, (long)1);
        scheduleAt(simTime() + period_, selfSender_);

    } else if (!strcmp(msg->getName(), "_pdu_sender")) {
      // std::cout << __func__ << simTime() << "_pdu_sender" << std::endl;
      this->RlcHandler("ToPhy");
      // _pdu_sender->setSchedulingPriority(0);
      scheduleAt(simTime() + _pdu_interval, _pdu_sender);

    }
    else if (!strcmp(msg->getName(), "_removeDataFromQueue")) {
      removeDataFromQueue();

    } else if (!strcmp(msg->getName(), "_cbf_resend")) {
      this->GeoNetworkHandler("ReSend", {});

    } else if (!strcmp(msg->getName(), "_resource_selection")) {
      resource_selection();

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

void Mode4App::removeDataFromQueue()
{
  // !!!!! important, so dont remove these codes !!!!!
  simtime_t pre_time = _pdu_sender->getArrivalTime();
  cancelEvent(_pdu_sender);
  scheduleAt(pre_time, _pdu_sender);
  // !!!!! important, so dont remove these codes !!!!!

  if (UmTxEntityPtr == nullptr && veins::FindModule<UmTxEntity*>::findSubModule(getParentModule())) {
    UmTxEntityPtr = veins::FindModule<UmTxEntity*>::findSubModule(getParentModule());
  }

  if (UmTxEntityPtr != nullptr) {
    UmTxEntityPtr->removeDataFromQueue();
  }
}

void Mode4App::SendPacket(std::string payload, std::string type, int payload_byte_size, int duration_ms, json pdu_info={})
{
  // std::cout << __func__ << ", " << simTime() << ", pdu_info: " << pdu_info << std::endl;
  if (type == "pdu" || type == "reselection") {

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
      // lteControlInfo->setMyChannelNum(5);
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
    // packet["size"] = 700;
    packet["sender_id"] = sumo_id;
    packet["max_duration"] = 100;
    packet["min_duration"] = 20; // For ensuring grant in Mac layer.
    packet["expired_time"] = simTime().dbl() + 0.1;

    if (type == "cam") {
      packet["priority"] = 2;
    } else if (type == "cpm") {
      packet["priority"] = 3;
    } else {
      ASSERT(type == "cam" || type == "cpm");
    }

    // this->StachSendEnque(packet);
    inet::Coord CurrentPos = mobility->getCurrentPosition();
    double target_dist = 1000;
    if (CurrentPos.x <= 0) {
      // do nothing.
    } else {
      target_dist = - target_dist;
    }

    this->FacilityHandler(
      "ToGeocast",
      this->attach_headers(
        packet,
        _network_ptr->header(
          CurrentPos.x,
          CurrentPos.y,
          CurrentPos.x + target_dist,
          CurrentPos.y,
          _max_hop,
          packet["expired_time"].get<double>(),
          this->sumo_id
        ),
        _sdu_tx_ptr->header(
          this->sumo_id,
          packet["priority"].get<int>(),
          packet["expired_time"].get<double>()
        )
      )
    );

    // ----
  } else {
    ASSERT(type == "cam" || type == "cpm" || type == "pdu");
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


void Mode4App::resource_selection() {
  // std::cout << __func__ << ", " << simTime() << ", Begin." << std::endl;

  json pdu_info = {};
  bool is_required_more_cr = false;
  bool is_short_duration = false;
  if (isSduQueueEmpty() && _sdu_tx_ptr->is_empty(simTime().dbl()) == false && simTime() + _sdu_tx_ptr->_min_rri / 2.0 < _pdu_sender->getArrivalTime()) {
    pdu_info = _sdu_tx_ptr->get_duration_size_rri(
      simTime().dbl(),
      _sdu_tx_ptr->maximum_duration(simTime().dbl())
    );

    std::cout << __func__ << ", " << simTime() << ", sumo_id: " << sumo_id << ", pdu_info: " << pdu_info <<  ", res time: " << _pdu_sender->getArrivalTime() << std::endl;
    is_required_more_cr = (_current_ch / _current_rri < pdu_info["ch"].get<int>() / pdu_info["rri"].get<double>());
    is_short_duration = pdu_info["duration"].get<double>() < _pdu_sender->getArrivalTime().dbl() - simTime().dbl();

  } else {
    cancelEvent(_resource_selection);
    scheduleAt(_pdu_sender->getArrivalTime() + 2 * TTI, _resource_selection);
  }

  if (is_required_more_cr || is_short_duration) {
    SendPacket("", "reselection", 0, pdu_info["duration"].get<double>() * 1000, pdu_info);
    scheduleAt(simTime(), _removeDataFromQueue);
  }
}
// ----- End My Code -----

void Mode4App::finish()
{
    cancelAndDelete(selfSender_);
    string_vector2file(grants_file_path(carlaVeinsDataDir, sumo_id), this->_grant_logs);
    string_vector2file(dup_count_file_path(carlaVeinsDataDir, sumo_id), _network_ptr->duplication_packets_count());
    string_vector2file(cbr_file_path(carlaVeinsDataDir, sumo_id), _cbr_logs);
}

Mode4App::~Mode4App()
{
    binder_->unregisterNode(nodeId_);
}
