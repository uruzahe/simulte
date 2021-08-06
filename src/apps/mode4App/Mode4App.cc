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

int existFile(const char* path)
{
    FILE* fp = fopen(path, "r");
    if (fp == NULL) {
        return 0;
    }

    fclose(fp);
    return 1;
}


int carla_lock_wait(std::string data_sync_dir) {
  std::string carla_lock_file_path = data_sync_dir + "carla.lock";

  if (existFile(carla_lock_file_path.c_str())) {
    return carla_lock_wait(data_sync_dir);
  } else {
    return 1;
  }
}


void lock(const char *oldpath, const char *newpath) {
  while (symlink(oldpath, newpath) == -1) {
    continue;
  }
}


void set_cpm_payloads_for_carla(std::string sumo_id, std::string data_sync_dir, std::vector<std::string> payloads) {
    std::string packet_data_file_name = sumo_id + "_packet.json";
    std::string packet_lock_file_name = sumo_id + "_packet.json.lock";

//    lock((data_sync_dir + packet_data_file_name).c_str(), (data_sync_dir + packet_lock_file_name).c_str());
    std::ofstream ofs(data_sync_dir + packet_data_file_name, std::ios::in | std::ios::ate);
    if (ofs.is_open()) {
        for (auto payload = payloads.begin(); payload != payloads.end(); payload++) {
            ofs << *payload << std::endl;
        }
    }
    ofs.close();
//    unlink((data_sync_dir + packet_lock_file_name).c_str());
}


std::vector<std::string> get_cpm_payloads_from_carla(std::string sumo_id, std::string data_sync_dir, bool read_only) {
    std::string sensor_data_file_name = sumo_id + "_sensor.json";
    std::string sensor_lock_file_name = sumo_id + "_sensor.json.lock";

    std::vector<std::string> payloads = {};
    std::string payload;

//    lock((data_sync_dir + sensor_data_file_name).c_str(), (data_sync_dir + sensor_lock_file_name).c_str());
    std::ifstream ifs(data_sync_dir + sensor_data_file_name);
    if (ifs.is_open()) {
        while (!ifs.eof()) {
          std::getline(ifs, payload);
          if (payload != "") {
            payloads.push_back(payload);
          } else {
            continue;
          }
        }
    }
    ifs.close();

    if (!read_only) {
      std::ofstream ofs(data_sync_dir + sensor_data_file_name);
      ofs.close();
    }

//    unlink((data_sync_dir + sensor_lock_file_name).c_str());
    return payloads;
}
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

        sendCPMEvt = new cMessage("cpm");
        sumo_id = mobility->external_id;
        obtainedCPMs = {};
        if (is_dynamic_simulation) {
          reservedCPMs = {} ;
        } else {
          reservedCPMs = get_cpm_payloads_from_carla(sumo_id, carlaVeinsDataDir, true);
        }
        veinsLockFile = sumo_id + "_veins.lock";
        veinsTxtFile = sumo_id + "_veins.txt";

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
//            std::cout << sumo_id << " received cpm messages" << std::endl;
//            std::cout << "payloads: " << cpm->getPayload() << std::endl;
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

        nextSno_++;

        emit(sentMsg_, (long)1);

        scheduleAt(simTime() + period_, selfSender_);
    }
    else
        throw cRuntimeError("Mode4App::handleMessage - Unrecognized self message");
}

// ----- Begin My Code -----
void Mode4App::syncCarlaVeinsData(cMessage* msg)
{
    std::vector<std::string> targetCPMs;
    double next_time_step = carlaTimeStep;

    if (is_dynamic_simulation) {
      // save received cpms
      set_cpm_payloads_for_carla(sumo_id, carlaVeinsDataDir, obtainedCPMs);
      obtainedCPMs.clear();
      obtainedCPMs.shrink_to_fit();

      // send CPMs
      std::vector<std::string> new_payloads = get_cpm_payloads_from_carla(sumo_id, carlaVeinsDataDir, false);

      for (auto payload = new_payloads.begin(); payload != new_payloads.end(); payload++) {
        targetCPMs.push_back(*payload);
      }

    } else {
      auto payload = reservedCPMs.begin();

      while (payload != reservedCPMs.end()) {
        try {
            json payload_json = json::parse(*payload);

            double timestamp = payload_json["timestamp"].get<double>();
            double simtime = simTime().dbl();
            // std::cout << "sumo_id" << sumo_id << "simTime: " << simtime << " timestamp: " << timestamp << std::endl;

            if (timestamp <= simtime - carlaTimeStep) {
              // std::cout << "The packet is too old, so erase it." << std::endl;
              reservedCPMs.erase(payload);
            } else if (simtime - carlaTimeStep < timestamp && timestamp <= simtime) {
              // std::cout << "The packet is created now, so send it." << std::endl;
              targetCPMs.push_back(*payload);
              reservedCPMs.erase(payload);
            } else {
              // std::cout << "The packet should be sent in the next timestamp, so break" << std::endl;
              break;
            }
        } catch (...) {
            std::cout << "reservedCPMd error: "<< (*payload).c_str() << "." << std::endl;
            continue;
        }
      }
    }

    for (auto payload = targetCPMs.begin(); payload != targetCPMs.end(); payload++) {
      try {
          json payload_json = json::parse(*payload);

          veins::VeinsCarlaCpm* packet = new veins::VeinsCarlaCpm();

          packet->setPayload((*payload).c_str());
          packet->setBitLength(payload_json["option"]["size"].get<int>() * 8);

          // ----- Begin Population -----
          packet->setTimestamp(simTime());
//          packet->setByteLength(size_);
          packet->setSno(nextSno_);

          auto lteControlInfo = new FlowControlInfoNonIp();

          lteControlInfo->setSrcAddr(nodeId_);
          lteControlInfo->setDirection(D2D_MULTI);
          lteControlInfo->setPriority(priority_);
          lteControlInfo->setDuration(duration_);
          lteControlInfo->setCreationTime(simTime());

          packet->setControlInfo(lteControlInfo);

//          std::cout << "payload: " << *payload << std::endl;
          Mode4BaseApp::sendLowerPackets(packet);
          // ----- End Population -----
      } catch (...) {
          std::cout << "targetCPMd error: "<< (*payload).c_str() << "." << std::endl;
          continue;
      }
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
