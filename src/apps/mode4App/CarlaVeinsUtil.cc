#include "apps/mode4App/CarlaVeinsUtil.h"

using json = nlohmann::json;

// ----- Begin: JsonDataStore-----
JsonDataStore::JsonDataStore() {
  _data = {};
}

bool JsonDataStore::is_empty(){
  return _data.empty();
}

void JsonDataStore::load_json_str(std::string json_str){
  try {
    // // std::cout << json_str << std::endl;
    _data.push_back(json::parse(json_str));

  } catch (...) {
    // ignore the json_str.
  }
}

void JsonDataStore::load_json_strs(std::vector<std::string> json_strs){
  for (auto data_pointer = json_strs.begin(); data_pointer != json_strs.end(); data_pointer++) {
    load_json_str(*data_pointer);
  }

  // ----- For unloding too many data -----
  _data = this->data_between_time(149, 170);
}


std::vector<json> JsonDataStore::pop_all(){
  std::vector<json> pop_data = _data;
  _data = {};

  return pop_data;
}

json JsonDataStore::pop_back(){
  json result = _data.back();

  _data.pop_back();

  return result;
}


void JsonDataStore::push_back(json data){
  _data.push_back(data);
}

std::vector<json> JsonDataStore::data_between_time(double begin_time, double end_time){
  std::vector<json> results;

  auto ptr = _data.begin();

  while (ptr != _data.end()){
    json d = *ptr;
    double timestamp = d["timestamp"].get<double>();

    if (timestamp < begin_time) {
      // // std::cout << "The packet is too old, so erase it." << std::endl;
      _data.erase(ptr);

    } else if (begin_time <= timestamp && timestamp < end_time) {
      // // std::cout << "The packet is created now, so send it." << std::endl;
      results.push_back(d);
      _data.erase(ptr);

    } else {
      // // std::cout << "The packet should be sent in the next timestamp, so break" << std::endl;
      break;

    }
  }

  return results;
}
// ----- End: JsonDataStore-----

// ----- Begin: CAMHandler
std::vector<json> CAMHandler::filter_cams_by_etsi(std::vector<json> cams)
{
  if (cams.empty()) { return {}; }

  json target_cam = cams.back();
  if (_data.empty()) { return {target_cam}; }

  json latest_cam = _data.back();
  double dT = target_cam["timestamp"].get<double>() - latest_cam["timestamp"].get<double>();

  double dlx = target_cam["HF_Container"]["location"]["x"].get<double>() - latest_cam["HF_Container"]["location"]["x"].get<double>();
  double dly = target_cam["HF_Container"]["location"]["y"].get<double>() - latest_cam["HF_Container"]["location"]["y"].get<double>();
  double dl = sqrt(dlx * dlx + dly * dly);

  double dsx = target_cam["HF_Container"]["speed"]["x"].get<double>() - latest_cam["HF_Container"]["speed"]["x"].get<double>();
  double dsy = target_cam["HF_Container"]["speed"]["y"].get<double>() - latest_cam["HF_Container"]["speed"]["y"].get<double>();
  double ds = sqrt(dsx * dsx + dsy * dsy);

  double dy = target_cam["HF_Container"]["yaw"].get<double>() - latest_cam["HF_Container"]["yaw"].get<double>();

  // ----- ETSI standards -----
  if (0.1 <= dT && (1 <= dT || 4 <= dl || 0.5 <= ds || 4 <= dy)) {
    return {target_cam};
  } else {
    return {};
  }
}

json CAMHandler::convert_payload_and_size(json cam, int max_payload_byte)
{
  json result;

  result["payload"] = cam.dump();
  result["type"] = "cam";
  result["size"] = max_payload_byte;

  _data = {cam};

  return result;
}
// ----- End: CAMHandler -----

// ----- Begin: POHandler -----
POHandler::POHandler()
{
  _pseu2po = {};
}

std::vector<json> POHandler::filter_pos_by_etsi(std::vector<json> perceived_objects)
{
  std::unordered_map<std::string, json> result = {};

  for (auto ptr = perceived_objects.begin(); ptr != perceived_objects.end(); ptr++ ) {
    json po = (*ptr);
    std::string pseu = po["pseudonym"].get<std::string>();

    if (_pseu2po.find(pseu) == _pseu2po.end()) {
      result[pseu] = po;

    } else {
      json latest_po = _pseu2po[pseu];

      double dT = po["timestamp"].get<double>() - latest_po["timestamp"].get<double>();

      double dlx = po["location"]["x"].get<double>() - latest_po["location"]["x"].get<double>();
      double dly = po["location"]["y"].get<double>() - latest_po["location"]["y"].get<double>();
      double dl = sqrt(dlx * dlx + dly * dly);

      double dsx = po["speed"]["x"].get<double>() - latest_po["speed"]["x"].get<double>();
      double dsy = po["speed"]["y"].get<double>() - latest_po["speed"]["y"].get<double>();
      double ds = sqrt(dsx * dsx + dsy * dsy);

      // ----- ETSI standards -----
      if (0.1 <= dT && (1 <= dT || 4 <= dl || 0.5 <= ds)) {
        result[pseu] = po;
      }
    }
  }

  std::vector<json> result_vec;
  for (auto ptr = result.begin(); ptr != result.end(); ptr++) {
    result_vec.push_back(ptr->second);
  }

  return result_vec;
}

json POHandler::convert_payload_and_size(std::vector<json> perceived_objects, int sensor_num, int max_payload_byte)
{
  std::vector<std::string> include_pos = {};
  int size = 121 + 35 * sensor_num;

  for (auto ptr = perceived_objects.begin(); ptr != perceived_objects.end(); ptr++) {
    if (35 <= max_payload_byte - size) {
      size = size + 35;
      _pseu2po[(*ptr)["pseudonym"].get<std::string>()] = (*ptr);
      include_pos.push_back((*ptr).dump());

    } else {
      break;

    }
  }

  json result;
  json pos;
  pos = include_pos;
  result["payload"] = pos.dump();
  result["type"] = "cpm";
  if (include_pos.empty()) {
    result["size"] = 0;
  } else {
    result["size"] = size;
  }

  return result;

}
// ----- End: POHandler -----

// ----- Begin: VirtualTxSduQueue -----
VirtualTxSduQueue::VirtualTxSduQueue()
{
  _past_fragments = {};
  _delete_expired_time = 0;
  _fragment = NULL;
  _packet_id = 0;
  // As the priority index is lower, more the priority becomes high.
  // The index 0 indicates the most important packet.

  _priority2packets[0] = {};
  _priority2packets[1] = {};
  _priority2packets[2] = {};
  _priority2packets[3] = {};
}

void VirtualTxSduQueue::delete_expired_fragments(double current_time)
{

  if (_delete_expired_time < current_time) {
    _delete_expired_time = current_time;
  } else {
    return;
  }

  if (_fragment != NULL && _fragment["expired_time"].get<double>() <= current_time) {
    _fragment = NULL;
  }
  // std::cout << "_fragment is NULL" << std::endl;

  for (auto ptr = _priority2packets.begin(); ptr != _priority2packets.end(); ptr++) {
    auto itr = ptr->second.begin();

    while (itr != ptr->second.end()) {
      // std::cout << "in loop: " << (*itr) << std::endl;
      if ((*itr)["expired_time"].get<double>() <= current_time) {
        ptr->second.erase(itr);
      } else {
        itr++;
      }
    }
  }

}

json VirtualTxSduQueue::formatted_fragment(json packet, int lefted_size, int status_flag) {
  json fragment = packet;

  fragment["lefted_size"] = lefted_size;
  fragment["status_flag"] = status_flag; // 01: start flag, 10: end flag, 11: packet, 00: middle frag
  fragment["seq"] = _past_fragments.size();

  _past_fragments.push_back(fragment);

  return fragment;
}

json VirtualTxSduQueue::update_fragment(json fragment, int lefted_size, int status_flag) {
  fragment["lefted_size"] = lefted_size;
  fragment["status_flag"] = status_flag;

  return fragment;
}

json VirtualTxSduQueue::formatted_pdu(int maximum_size, double current_time) {
  json pdu;
  pdu["maximum_size"] = maximum_size - MY_PDCP_HEADER_BYTE - MY_RLC_UM_HEADER_BYTE;
  pdu["timestamp"] = current_time;
  pdu["duration"] = 100;
  pdu["size"] = MY_MAC_HEADER_BYTE;
  pdu["type"] = "pdu";
  pdu["sdus"] = {};

  return pdu;
}

void VirtualTxSduQueue::enque(int priority, json packet)
{
  packet["packet_id"] = _packet_id;
  _priority2packets[priority].push_back(packet);

  _delete_expired_time = 0;
  _packet_id++;
}

json VirtualTxSduQueue::update_pdu_by_fragment(json pdu, double current_time) {
  int lefted_size = pdu["maximum_size"].get<int>() - pdu["size"].get<int>();

  if (_fragment != NULL && 0 < lefted_size) {
    if (_fragment["lefted_size"].get<int>() <= lefted_size) {
      pdu = this->add_fragment_into_pdu(
          pdu,
          this->update_fragment(
            _fragment,
            _fragment["lefted_size"].get<int>(),
            _fragment["status_flag"].get<int>() + 10
          ),
          current_time
      );

      _fragment = NULL;

    } else {
      pdu = this->add_fragment_into_pdu(
          pdu,
          this->update_fragment(
            _fragment,
            lefted_size,
            _fragment["status_flag"].get<int>()
          ),
          current_time
      );

      _fragment = formatted_fragment(_fragment, _fragment["lefted_size"].get<int>() - lefted_size, 0);
    }
  }

  return pdu;
}

json VirtualTxSduQueue::add_fragment_into_pdu(json pdu, json send_fragment, double current_time)
{
  std::vector<json> sdus;
  if (pdu.find("sdu") != pdu.end()) {
    // std::cout << "find sdu" << std::endl;
    sdus = pdu["sdus"].get<std::vector<json>>();
  }
  // std::cout << "end find sdu" << std::endl;

  double tmp_duration = (send_fragment["expired_time"].get<double>() - current_time) * 1000.0;
  // std::cout << "tmp_duration: " << tmp_duration << "duration: " << int(tmp_duration) << std::endl;
  if (sdus.size() == 0 || tmp_duration < pdu["duration"].get<int>()) {
    pdu["duration"] = (int)(tmp_duration + 1);
  }

  sdus.push_back(send_fragment);
  pdu["sdus"] = sdus;

  pdu["size"] = pdu["size"].get<int>() + MY_PDCP_HEADER_BYTE + MY_RLC_UM_HEADER_BYTE + send_fragment["lefted_size"].get<int>();

  return pdu;
}

json VirtualTxSduQueue::generate_PDU(int maximum_byte, double current_time)
{
  this->delete_expired_fragments(current_time);

  json pdu = this->update_pdu_by_fragment(
    this->formatted_pdu(maximum_byte, current_time),
    current_time
  );

  if (_fragment != NULL || pdu["maximum_size"].get<int>() <= pdu["size"].get<int>()) {
    return pdu;
  }

  for (auto ptr = _priority2packets.begin(); ptr != _priority2packets.end(); ptr++) {
    auto itr = ptr->second.begin();

    while (itr != ptr->second.end()) {
      _fragment = this->formatted_fragment(*itr, (*itr)["size"].get<int>(), 1);
      // std::cout << "update_pdu_by_fragment" << std::endl;
      pdu = this->update_pdu_by_fragment(pdu, current_time);
      // std::cout << "end: update_pdu_by_fragment" << std::endl;

      // std::cout << "erase" << std::endl;
      ptr->second.erase(itr);
      // std::cout << "end: erase" << std::endl;

      if (_fragment != NULL || pdu["maximum_size"].get<int>() <= pdu["size"].get<int>()) { break; }
    }

    if (_fragment != NULL || pdu["maximum_size"].get<int>() <= pdu["size"].get<int>()) { break; }
  }

  // ----- substruct (MAC_HEADER + RLC_HEADER_UM + PDCP_HEADER_UM) because these value will be added in low layer in OpenCV2X. -----
  pdu["size"] = pdu["size"].get<int>() - (MAC_HEADER + RLC_HEADER_UM + PDCP_HEADER_UM);

  return pdu;
}



json VirtualTxSduQueue::minimum_Bps(double current_time)
{
  // std::cout << "delete_expired_fragments" << std::endl;
  this->delete_expired_fragments(current_time);
  // std::cout << "end: delete_expired_fragments" << std::endl;

  double total_byte = 0;
  double Bps = 0;
  double tmp_Bps = 0;

  if (_fragment != NULL) {
    total_byte = _fragment["lefted_size"].get<double>();
    Bps = (_fragment["expired_time"].get<double>() - current_time);
  }

  for (auto ptr = _priority2packets.begin(); ptr != _priority2packets.end(); ptr++) {
    for (auto itr = ptr->second.begin(); itr != ptr->second.end(); itr++) {
      total_byte += (*itr)["size"].get<double>();
      tmp_Bps = total_byte / ((*itr)["expired_time"].get<double>() - current_time);

      if (Bps < tmp_Bps) { Bps = tmp_Bps; }
    }
  }

  // std::cout << "end: minimum_Bps" << std::endl;
  return Bps;
}
// ----- End: VirtualTxSduQueue -----


// ----- Begin: VirtualRxSduQueue -----
json VirtualRxSduQueue::enque_and_decode(json sdu)
{
  // std::cout << "begin: " << __func__ << std::endl;
  json result = NULL;

  std::string sender_id = sdu["sender_id"].get<std::string>();
  int packet_id = sdu["packet_id"].get<int>();

  // std::cout << __func__ << ": push_back" << std::endl;
  _sender2packet_id2sdus[sender_id][packet_id].push_back(sdu);

  // ----- decode sdus -----
  int packet_size = sdu["size"].get<int>();
  int sdu_total_size = 0;
  for (auto sdu_ptr = _sender2packet_id2sdus[sender_id][packet_id].begin(); sdu_ptr != _sender2packet_id2sdus[sender_id][packet_id].end(); sdu_ptr++) {
    // std::cout << __func__ << ": get leftted size" << std::endl;
    sdu_total_size += (*sdu_ptr)["lefted_size"].get<int>();

    if (sdu_total_size < packet_size) {
      // std::cout << __func__ << ": continue" << std::endl;
      continue;

    } else if (sdu_total_size == packet_size) {
      // std::cout << __func__ << ": decode" << std::endl;
      result = (*sdu_ptr);
      break;

    } else {
      assert(sdu_total_size <= packet_size);

    }
  }

  if (result != NULL) {
    // std::cout << __func__ << ": clear" << std::endl;
    _sender2packet_id2sdus[sender_id][packet_id].clear();
  }

  // std::cout << "end: " << __func__ << std::endl;
  return result;
}
// ----- End: VirtualRxSduQueue -----


// ----- Begin: function -----
std::string cams_json_file_path(std::string data_sync_dir, std::string sumo_id)
{
  return data_sync_dir + sumo_id + "_cams.json";
}

std::string cams_recv_json_file_path(std::string data_sync_dir, std::string sumo_id)
{
  return data_sync_dir + sumo_id + "_cams_recv.json";
}

std::string cpms_json_file_path(std::string data_sync_dir, std::string sumo_id)
{
  return data_sync_dir + sumo_id + "_cpms.json";

}

std::string objects_json_file_path(std::string data_sync_dir, std::string sumo_id)
{
  return data_sync_dir + sumo_id + "_objects.json";
}

std::string objects_recv_json_file_path(std::string data_sync_dir, std::string sumo_id)
{
  return data_sync_dir + sumo_id + "_objects_recv.json";
}

std::string packet_json_file_path(std::string data_sync_dir, std::string sumo_id)
{
  return data_sync_dir + sumo_id + "_packet.json";
}

std::string sensor_json_file_path(std::string data_sync_dir, std::string sumo_id)
{
  return data_sync_dir + sumo_id + "_sensor.json";
}


int existFile(const char* path)
{
    FILE* fp = fopen(path, "r");
    if (fp == NULL) {
        return 0;
    }

    fclose(fp);
    return 1;
}


std::vector<std::string> file2string_vector(std::string file_path, bool read_only){

  std::vector<std::string> payloads = {};
  std::string payload;

  std::ifstream ifs(file_path);
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
    std::ofstream ofs(file_path);
    ofs.close();
  }

  return payloads;
}

void touch(std::string file_path)
{
//  if (!existFile(file_path.c_str())) {
      FILE *fp;
      fp = fopen(file_path.c_str(), "w");
      fclose(fp);
//  }
}

void string_vector2file(std::string file_path, std::vector<std::string> str_vec) {
    std::ofstream ofs(file_path, std::ios::app);

    if (ofs.is_open()) {
        for (auto payload = str_vec.begin(); payload != str_vec.end(); payload++) {
            // // std::cout << "---: "<< *payload << "---" << std::endl;
            ofs << *payload << "\n";
        }
    }

    ofs.close();
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

json add_time_attribute_to_json(json data, std::string attr_name, double t)
{
  data[attr_name] = t;

  return data;
}


void set_cpm_payloads_for_carla(std::string sumo_id, std::string data_sync_dir, std::vector<std::string> payloads) {
    std::string packet_data_file_name = sumo_id + "_cpms.json";
    std::string packet_lock_file_name = sumo_id + "_cpms.json.lock";

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
    std::string sensor_data_file_name = sumo_id + "_cpms.json";
    std::string sensor_lock_file_name = sumo_id + "_cpms.json.lock";

    std::vector<std::string> payloads = {};
    std::string payload;

    //    lock((data_sync_dir + sensor_data_file_name).c_str(), (data_sync_dir + sensor_lock_file_name).c_str());
    return file2string_vector(data_sync_dir + sensor_data_file_name, read_only);
    //    unlink((data_sync_dir + sensor_lock_file_name).c_str());
}

json Bps2packet_size_and_rri(double minimum_Bps)
{
  json result;
  // result["size"] = 800;
  result["size"] = 300;
  result["rri"] = 1.0;
  result["ch"] = 1;

  std::vector<int> possible_bytes = {300, 450, 600, 800};
  std::vector<double> possible_rris = {0.1, 0.05, 0.02};

  std::unordered_map<int, int> bytes2channel_num_in_MCS7 = {
    {100, 1},
    {300, 2},
    {450, 3},
    {600, 4},
    {800, 5}
  };

  for (auto byte_ptr = possible_bytes.begin(); byte_ptr != possible_bytes.end(); byte_ptr++) {
    for (auto rri_ptr = possible_rris.begin(); rri_ptr != possible_rris.end(); rri_ptr++) {
      if (minimum_Bps <= (*byte_ptr) / (*rri_ptr)) {
        result["size"] = (*byte_ptr);
        result["rri"] = (*rri_ptr);
        result["ch"] = bytes2channel_num_in_MCS7[(*byte_ptr)];

        return result;

      } else {
        continue;

      }
    }
  }

  return result;
}
// ----- End: function -----
