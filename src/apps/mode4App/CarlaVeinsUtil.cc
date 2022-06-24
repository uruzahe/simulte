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
  // return {target_cam};

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

// ----- Begin MCMHandler -----
json MCMHandler::generate_dummy_message(int message_size, std::string message_payload)
{
  json result;

  result["size"] = message_size;
  result["payload"] = message_payload;

  return result;
}

// ----- Begin CC3GPPHandler -----
CC3GPPHandler::CC3GPPHandler()
{
  for (auto ch_ptr = _possible_channels.begin(); ch_ptr != _possible_channels.end(); ch_ptr++) {
    for (auto rri_ptr = _possible_rris.begin(); rri_ptr != _possible_rris.end(); rri_ptr++) {
      _crlimit2resources[((*ch_ptr) / (*rri_ptr)) / (1.0 / TTI)] = {
        {"ch", (*ch_ptr)},
        {"rri", (*rri_ptr)}
      };
    }
  }
}

double CC3GPPHandler::cbr2crlimit(double cbr)
{
  double offset = 0.5;
  cbr = cbr + offset;

  if (cbr <= 0.65) { return 1.0; }
  else if (cbr <= 0.675) { return 1.6 / 1000.0; }
  else if (cbr <= 0.700) { return 1.5 / 1000.0; }
  else if (cbr <= 0.725) { return 1.4 / 1000.0; }
  else if (cbr <= 0.750) { return 1.3 / 1000.0; }
  else if (cbr <= 0.800) { return 1.2 / 1000.0; }
  else if (cbr <= 0.825) { return 1.1 / 1000.0; }
  else if (cbr <= 0.850) { return 1.0 / 1000.0; }
  else if (cbr <= 0.875) { return 0.9 / 1000.0; }
  else { return 0.8 / 1000.0; }
}

double CC3GPPHandler::rrich2occupancy(double rri, int ch)
{
  return (ch / rri) / (1.0 / TTI);
}

json CC3GPPHandler::crlimit2resource(double crlimit, json default_resource)
{
  json result = default_resource;

  for (auto cr_ptr = _crlimit2resources.begin(); cr_ptr != _crlimit2resources.end(); cr_ptr++) {
    if (crlimit <= cr_ptr->first) {
      break;
    } else {
      result = cr_ptr->second;
    }
  }

  return result;
}
// ----- End CC3GPPHandler -----

// ----- Begin: VirtualTxSduQueue -----
VirtualTxSduQueue::VirtualTxSduQueue()
{
  _delete_expired_time = 0;
  _fragment = NULL;
  _packet_id = 0;
  // As the priority index is lower, more the priority becomes high.
  // The index 0 indicates the most important packet.

  // ETSI has 4 priority in Access Layer [1] -----
  // [1]: ETSI EN 302 663 V1.2.1. (2020). Intelligent Transport Systems (ITS); Access layer specification for in the 5 GHz frequency band. 1, 1–24.
  _priority2packets[0] = {}; // High priority DENMs
  _priority2packets[1] = {}; // Normal DENMs
  _priority2packets[2] = {}; // CAMs
  _priority2packets[3] = {}; // Forwarded DENMs and other low priority messages (etc. CPM)
  _priority2packets[4] = {}; // multihop Forwarded message (in convenience)


  for (auto itr = bytes2channel_num_in_MCS7.begin(); itr != bytes2channel_num_in_MCS7.end(); itr++) {
    _ch2size[itr->second] = itr->first;

    for (auto jtr = possible_rris.begin(); jtr != possible_rris.end(); jtr++) {
      cbr2seize_ch_rri[itr->first / (*jtr)] = {
        {"size", itr->first},
        {"ch", itr->second},
        {"rri", (*jtr)}
      };

      cbrs.push_back(itr->first / (*jtr));
    }
  }
  std::sort(cbrs.begin(), cbrs.end());

  _max_size = cbr2seize_ch_rri[cbrs.back()]["size"].get<int>();
  _max_ch =   cbr2seize_ch_rri[cbrs.back()]["ch"].get<int>();
  _max_rri =  cbr2seize_ch_rri[cbrs.back()]["rri"].get<double>();

  _min_rri = *std::min_element(possible_rris.begin(), possible_rris.end());
}


json VirtualTxSduQueue::header(std::string sender_id, int priority, double expired_time, double resource_consider_time) {
  json header;

  header["rlc"] = {
    {"sender_id", sender_id},
    {"priority", priority},
    {"expired_time", expired_time},
    {"resource_consider_time", min(resource_consider_time, expired_time)}
  };

  return header;
}

json VirtualTxSduQueue::update_header(json packet, std::string sender_id, double resource_consider_time) {
  packet["rlc"]["sender_id"] = sender_id;
  packet["rlc"]["resource_consider_time"] = min(resource_consider_time, packet["rlc"]["expired_time"].get<double>());

  return packet;
}

void VirtualTxSduQueue::delete_expired_fragments(double current_time)
{
  // std::cout << __func__ << ", Begin." << std::endl;

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
        // std::cout << "erased: " << std::endl;
        itr = ptr->second.erase(itr);
      } else {
        itr++;
      }
    }
  }

}

json VirtualTxSduQueue::formatted_fragment(json packet, int lefted_size, int status_flag, int start_byte, int end_byte) {
  // std::cout << __func__ << ", Begin." << std::endl;
  ASSERT(start_byte < end_byte);

  json fragment = packet;

  fragment["lefted_size"] = lefted_size;
  fragment["status_flag"] = status_flag; // 01: start flag, 10: end flag, 11: packet, 00: middle frag
  fragment["seq"] = _past_fragment_count++;
  fragment["start_byte"] = start_byte;
  fragment["end_byte"] = end_byte;

  return fragment;
}

json VirtualTxSduQueue::update_fragment(json fragment, int lefted_size, int status_flag, int start_byte, int end_byte) {
  // std::cout << __func__ << ", Begin." << std::endl;
  ASSERT(start_byte < end_byte);

  fragment["lefted_size"] = lefted_size;
  fragment["status_flag"] = status_flag;
  fragment["start_byte"] = start_byte;
  fragment["end_byte"] = end_byte;

  return fragment;
}

json VirtualTxSduQueue::formatted_pdu(int maximum_size, double current_time) {

  // std::cout << __func__ << ", Begin." << std::endl;
  json pdu;
  pdu["maximum_size"] = maximum_size;
  pdu["timestamp"] = current_time;
  pdu["duration"] = 100;
  pdu["size"] = 0;
  pdu["type"] = "pdu";

  std::vector<json> empty_vec = {};
  pdu["sdus"] = empty_vec;

  return pdu;
}

bool VirtualTxSduQueue::enque(json packet, double current_time, double min_duration)
{
  // ----- Since too short expired time frequnetly occurs grant break, we ignore such the packet. -----
  // if (packet["rlc"]["expired_time"].get<double>() < current_time - min_duration) {
  if (packet["rlc"]["expired_time"].get<double>() < current_time) {
    return false;
  }

  // std::cout << __func__ << ", Begin." << std::endl;
  packet["packet_id"] = _packet_id;

  ASSERT(std::find(_priority2packets.begin(), _priority2packets.end(), packet["rlc"]["priority"].get<int>()) != _priority2packets.end());
  _priority2packets[packet["rlc"]["priority"].get<int>()].push_back(packet);

  _delete_expired_time = 0;
  _packet_id++;

  return true;
}


bool VirtualTxSduQueue::enque_temporary(json packet, double current_time, double min_duration)
{
  // ----- Since too short expired time frequnetly occurs grant break, we ignore such the packet. -----
  // if (packet["rlc"]["expired_time"].get<double>() < current_time - min_duration) {
  if (packet["rlc"]["expired_time"].get<double>() < current_time) {
    return false;
  }

  // std::cout << __func__ << ", Begin." << std::endl;
  packet["packet_id"] = _packet_id;

  ASSERT(std::find(_priority2packets.begin(), _priority2packets.end(), packet["rlc"]["priority"].get<int>()) != _priority2packets.end());
  _temporary_priority2packets = _priority2packets;
  _temporary_priority2packets[packet["rlc"]["priority"].get<int>()].push_back(packet);

  return true;
}

json VirtualTxSduQueue::update_pdu_by_fragment(json pdu, double current_time) {

  // std::cout << __func__ << ", Begin." << std::endl;
  int lefted_size = pdu["maximum_size"].get<int>() - pdu["size"].get<int>() - MY_RLC_UM_HEADER_BYTE - MY_MAC_HEADER_BYTE;

  if (_fragment != NULL && 0 < lefted_size) {
    if (_fragment["end_byte"].get<int>() - _fragment["start_byte"].get<int>() <= lefted_size) {
      pdu = this->add_fragment_into_pdu(
          pdu,
          this->update_fragment(
            _fragment,
            _fragment["lefted_size"].get<int>(),
            _fragment["status_flag"].get<int>() + 10,
            _fragment["start_byte"].get<int>(),
            _fragment["end_byte"].get<int>()
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
            _fragment["status_flag"].get<int>(),
            _fragment["start_byte"].get<int>(),
            _fragment["start_byte"].get<int>() + lefted_size
          ),
          current_time
      );

      _fragment = formatted_fragment(
        _fragment,
        _fragment["lefted_size"].get<int>() - lefted_size,
        0,
        _fragment["start_byte"].get<int>() + lefted_size,
        _fragment["end_byte"].get<int>()
      );
    }
  }

  return pdu;
}

json VirtualTxSduQueue::add_fragment_into_pdu(json pdu, json send_fragment, double current_time)
{

  // std::cout << __func__ << ", Begin." << std::endl;
  std::vector<json> sdus;
  if (pdu.find("sdus") != pdu.end()) {
    // std::cout << __func__ << ", Find Sdus: " << sdus << std::endl;
    sdus = pdu["sdus"].get<std::vector<json>>();
  } else {
    // std::cout << __func__ << ", No Sdus." << std::endl;
  }
  // std::cout << __func__ << "sdus:" << sdus << ", Find End." << std::endl;

  double tmp_duration = max(
    send_fragment["min_duration"].get<double>(),
    min(
      (send_fragment["expired_time"].get<double>() - current_time) * 1000.0,
      send_fragment["max_duration"].get<double>()
    )
  );
  if (sdus.size() == 0 || tmp_duration < pdu["duration"].get<int>()) {
    pdu["duration"] = (int)(tmp_duration + 1);
  }

  // std::cout << __func__ << "sdus:" << sdus << ", push_back." << std::endl;
  sdus.push_back(send_fragment);
  pdu["sdus"] = sdus;
  // std::cout << __func__ << "sdus:" << sdus << ", push_back end." << std::endl;

  pdu["size"] = pdu["size"].get<int>() + MY_RLC_UM_HEADER_BYTE + MY_MAC_HEADER_BYTE + send_fragment["lefted_size"].get<int>();

  return pdu;
}

json VirtualTxSduQueue::generate_PDU(int maximum_byte, double current_time)
{

  // std::cout << __func__ << ", Begin." << std::endl;
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

    // std::cout << __func__ << ", " << current_time << ", priority: " << ptr->first << std::endl;
    while (itr != ptr->second.end()) {
      _fragment = this->formatted_fragment(*itr, (*itr)["size"].get<int>(), 1, 0, (*itr)["size"].get<int>());
      // std::cout << __func__ << ", " << current_time << ", _fragment: " << _fragment << std::endl;

      pdu = this->update_pdu_by_fragment(pdu, current_time);

      itr = ptr->second.erase(itr);

      if (_fragment != NULL || pdu["maximum_size"].get<int>() <= pdu["size"].get<int>()) { break; }
    }

    if (_fragment != NULL || pdu["maximum_size"].get<int>() <= pdu["size"].get<int>()) { break; }
  }

  // ----- substruct (MAC_HEADER + RLC_HEADER_UM + PDCP_HEADER_UM) because these value will be added in low layer in OpenCV2X. -----
  // // std::cout << __func__ << ", size: " << pdu["size"].get<int>() << ", in overhead: "  << a
  pdu["size"] = max(1.0, (double) (pdu["size"].get<int>() - (MAC_HEADER + RLC_HEADER_UM + PDCP_HEADER_UM)));


  // std::cout << __func__ << ", " << current_time << ", pdu: " << pdu << std::endl;
  return pdu;
}

int VirtualTxSduQueue::leftted_size_in_PDU(int maximum_byte, double current_time) {
  // std::cout << __func__ << ", Begin." << std::endl;
  this->delete_expired_fragments(current_time);

  int total_size = 0;
  if (_fragment != NULL) {
    total_size = _fragment["end_byte"].get<int>() - _fragment["start_byte"].get<int>() + MY_MAC_HEADER_BYTE + MY_RLC_UM_HEADER_BYTE;
  }

  for (auto ptr = _priority2packets.begin(); ptr != _priority2packets.end(); ptr++) {
    for (auto itr = ptr->second.begin(); itr != ptr->second.end(); itr++) {
      // std::cout << __func__ << ", priority: " << ptr->first << ", maximum_byte: " << maximum_byte << ", total_size: " << total_size << ", " << (*itr) <<  std::endl;
      total_size += (*itr)["size"].get<int>() + MY_MAC_HEADER_BYTE + MY_RLC_UM_HEADER_BYTE;

      if (maximum_byte <= total_size) {
        return 0;

      } else {
        continue;

      }
    }
  }

  if (maximum_byte <= total_size) {
    return 0;

  } else {
    return maximum_byte - total_size;

  }
}




// json VirtualTxSduQueue::minimum_Bps(double current_time)
// {
//
//   // std::cout << __func__ << ", Begin." << std::endl;
//   double total_byte = 0;
//
//   json result;
//   std::vector<double> bps_vec = {0};
//   std::vector<double> rri_vec;
//
//   this->delete_expired_fragments(current_time);
//
//   if (_fragment != NULL) {
//     total_byte += _fragment["lefted_size"].get<double>();
//     rri_vec.push_back(_fragment["expired_time"].get<double>() - current_time);
//     bps_vec.push_back(total_byte / rri_vec.back());
//   }
//
//   for (auto ptr = _priority2packets.begin(); ptr != _priority2packets.end(); ptr++) {
//     for (auto itr = ptr->second.begin(); itr != ptr->second.end(); itr++) {
//       total_byte += (*itr)["size"].get<double>();
//       rri_vec.push_back((*itr)["expired_time"].get<double>() - current_time);
//       bps_vec.push_back(total_byte / rri_vec.back());
//     }
//   }
//
//   return *std::max_element(bps_vec.begin(), bps_vec.end());
// }

json VirtualTxSduQueue::Bps2packet_size_and_rri(double minimum_Bps)
{

  // std::cout << __func__ << ", Begin." << std::endl;
  json result;
  result["size"] = cbr2seize_ch_rri[cbrs.back()]["size"].get<int>();
  result["ch"] =   cbr2seize_ch_rri[cbrs.back()]["ch"].get<int>();
  result["rri"] =  cbr2seize_ch_rri[cbrs.back()]["rri"].get<double>();


  for (auto ltr = cbrs.begin(); ltr != cbrs.end(); ltr++) {
    if (minimum_Bps <= (*ltr)) {
      result["size"] = cbr2seize_ch_rri[(*ltr)]["size"].get<int>();
      result["ch"] =   cbr2seize_ch_rri[(*ltr)]["ch"].get<int>();
      result["rri"] =  cbr2seize_ch_rri[(*ltr)]["rri"].get<double>();
      break;
    }
  }

  return result;
}


double VirtualTxSduQueue::maximum_duration(double current_time) {
  this->delete_expired_fragments(current_time);

  double result = 10;

  if (_fragment != NULL) {
    result = min(_fragment["expired_time"].get<double>() - current_time, result);
  }

  for (auto ptr = _priority2packets.begin(); ptr != _priority2packets.end(); ptr++) {
    for (auto itr = ptr->second.begin(); itr != ptr->second.end(); itr++) {
      result = max(0.001, min((*itr)["expired_time"].get<double>() - current_time, result));
    }
  }

  return result;
}


double VirtualTxSduQueue::maximum_duration_temporary(double current_time) {
  // std::cout << __func__ << ", " << current_time << std::endl;
  this->delete_expired_fragments(current_time);

  double result = 10;

  if (_fragment != NULL) {
    result = min(_fragment["expired_time"].get<double>() - current_time, result);
  }

  for (auto ptr = _temporary_priority2packets.begin(); ptr != _temporary_priority2packets.end(); ptr++) {
    for (auto itr = ptr->second.begin(); itr != ptr->second.end(); itr++) {
      result = max(0.001, min((*itr)["expired_time"].get<double>() - current_time, result));
    }
  }

  return result;
}

double VirtualTxSduQueue::resource_selection_time(double current_time)
{
  double result = current_time;

  for (auto p4itr = _priority2packets[4].begin(); p4itr != _priority2packets[4].end(); p4itr++) {
    if (result <= current_time || (*p4itr)["rlc"]["resource_consider_time"].get<double>() < result) {
      result = (*p4itr)["rlc"]["resource_consider_time"].get<double>();
    }
  }

  return result;
}

bool VirtualTxSduQueue::is_empty(double current_time) {
  this->delete_expired_fragments(current_time);

  bool result = (_fragment == NULL);

  for (auto ptr = _priority2packets.begin(); ptr != _priority2packets.end(); ptr++) {
    result = result && ptr->second.empty();
  }

  return result;
}

json VirtualTxSduQueue::get_duration_size_rri(double current_time, double maximum_duration) {
  // std::cout << __func__ << ", Begin." << std::endl;
  this->delete_expired_fragments(current_time);

  bool be_found = false;
  double total_byte = 0;
  double duration = 0.02;
  double rri = 0.1;
  double size = 300;

  double rri_count = 0;

  // ----- Begin Proposed Method -----
  int RCT_index_in_p4 = 0;
  for (auto p4itr = _priority2packets[4].begin(); p4itr != _priority2packets[4].end(); p4itr++) {
    if ((*p4itr)["rlc"]["resource_consider_time"].get<double>() <= current_time) {
      RCT_index_in_p4 = p4itr - _priority2packets[4].begin();
    }
  }
  // ----- End Proposed Method -----

  json result;
  result["duration"] = maximum_duration;
  result["size"] = cbr2seize_ch_rri[cbrs.back()]["size"].get<int>();
  result["ch"] =   cbr2seize_ch_rri[cbrs.back()]["ch"].get<int>();
  result["rri"] =  cbr2seize_ch_rri[cbrs.back()]["rri"].get<double>();

  for (auto ltr = cbrs.begin(); ltr != cbrs.end(); ltr++) {
    // std::cout << __func__ << ", -----" << std::endl;
    rri =  cbr2seize_ch_rri[(*ltr)]["rri"].get<double>();
    size = cbr2seize_ch_rri[(*ltr)]["size"].get<int>();

    for (duration = maximum_duration; 0 < duration; duration -= rri) {

      be_found = true;
      total_byte = 0;

      if (_fragment != NULL) {

        total_byte += _fragment["lefted_size"].get<double>();
        // std::cout << __func__ << "seg , expired" << _fragment["expired_time"].get<double>() << ", current_time: " << current_time << ", duration: " << duration << std::endl;
        rri_count = (int) ((_fragment["expired_time"].get<double>() - current_time - duration) / rri) + 1;

        // std::cout << __func__ << ", total_byte" << total_byte << ", rri_count: " << rri_count << ", size" << size << ", rri" << rri << std::endl;
        be_found = be_found && (total_byte <= rri_count * size);


        if (be_found == false) { continue; }
      }


      for (auto ptr = _priority2packets.begin(); ptr != _priority2packets.end(); ptr++) {
        // std::cout << __func__ << ", " << (ptr->first) << std::endl;
        for (auto itr = ptr->second.begin(); itr != ptr->second.end(); itr++) {
          // ----- Begin Proposed Method -----
          if (ptr->first == 4 && RCT_index_in_p4 < itr - ptr->second.begin()) {
            // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            continue;
          }
          // ----- End Proposed Method -----

          total_byte += (*itr)["size"].get<double>();
          // std::cout << __func__ << ", expired" << (*itr)["expired_time"].get<double>() << ", current_time: " << current_time << ", duration: " << duration << std::endl;
          rri_count = (int) (((*itr)["expired_time"].get<double>() - current_time - duration) / rri) + 1;

          // std::cout << __func__ << ", total_byte" << total_byte << ", rri_count: " << rri_count << ", size" << size << ", rri" << rri << std::endl;
          be_found = be_found && (total_byte < rri_count * size);


          if (be_found == false) { break; }
        }

        if (be_found == false) { break; }
      }

      if (be_found) {
        result["duration"] = duration;
        result["size"] = cbr2seize_ch_rri[(*ltr)]["size"].get<int>();
        result["ch"] =   cbr2seize_ch_rri[(*ltr)]["ch"].get<int>();
        result["rri"] =  cbr2seize_ch_rri[(*ltr)]["rri"].get<double>();
      } else {
        continue;
      }
    }

    if (be_found) { break; }
  }

  return result;
}


json VirtualTxSduQueue::get_duration_size_rri_temporary(double current_time, double maximum_duration) {
  // std::cout << __func__ << ", Begin." << std::endl;
  this->delete_expired_fragments(current_time);

  bool be_found = false;
  double total_byte = 0;
  double duration = 0.02;
  double rri = 0.1;
  double size = 300;

  double rri_count = 0;

  // ----- Begin Proposed Method -----
  int RCT_index_in_p4 = 0;
  for (auto p4itr = _temporary_priority2packets[4].begin(); p4itr != _temporary_priority2packets[4].end(); p4itr++) {
    if ((*p4itr)["rlc"]["resource_consider_time"].get<double>() <= current_time) {
      RCT_index_in_p4 = p4itr - _temporary_priority2packets[4].begin();
    }
  }
  // ----- End Proposed Method -----

  json result;
  result["duration"] = maximum_duration;
  result["size"] = cbr2seize_ch_rri[cbrs.back()]["size"].get<int>();
  result["ch"] =   cbr2seize_ch_rri[cbrs.back()]["ch"].get<int>();
  result["rri"] =  cbr2seize_ch_rri[cbrs.back()]["rri"].get<double>();

  for (auto ltr = cbrs.begin(); ltr != cbrs.end(); ltr++) {
    // std::cout << __func__ << ", -----" << std::endl;
    rri =  cbr2seize_ch_rri[(*ltr)]["rri"].get<double>();
    size = cbr2seize_ch_rri[(*ltr)]["size"].get<int>();

    for (duration = maximum_duration; 0 < duration; duration -= rri) {

      be_found = true;
      total_byte = 0;

      if (_fragment != NULL) {

        total_byte += _fragment["lefted_size"].get<double>();
        // std::cout << __func__ << "seg , expired" << _fragment["expired_time"].get<double>() << ", current_time: " << current_time << ", duration: " << duration << std::endl;
        rri_count = (int) ((_fragment["expired_time"].get<double>() - current_time - duration) / rri) + 1;

        // std::cout << __func__ << ", total_byte" << total_byte << ", rri_count: " << rri_count << ", size" << size << ", rri" << rri << std::endl;
        be_found = be_found && (total_byte <= rri_count * size);


        if (be_found == false) { continue; }
      }


      for (auto ptr = _temporary_priority2packets.begin(); ptr != _temporary_priority2packets.end(); ptr++) {
        // std::cout << __func__ << ", " << (ptr->first) << std::endl;
        for (auto itr = ptr->second.begin(); itr != ptr->second.end(); itr++) {
          // ----- Begin Proposed Method -----
          if (ptr->first == 4 && RCT_index_in_p4 < itr - ptr->second.begin()) {
            // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            continue;
          }
          // ----- End Proposed Method -----

          total_byte += (*itr)["size"].get<double>();
          // std::cout << __func__ << ", expired" << (*itr)["expired_time"].get<double>() << ", current_time: " << current_time << ", duration: " << duration << std::endl;
          rri_count = (int) (((*itr)["expired_time"].get<double>() - current_time - duration) / rri) + 1;

          // std::cout << __func__ << ", total_byte" << total_byte << ", rri_count: " << rri_count << ", size" << size << ", rri" << rri << std::endl;
          be_found = be_found && (total_byte < rri_count * size);


          if (be_found == false) { break; }
        }

        if (be_found == false) { break; }
      }

      if (be_found) {
        result["duration"] = duration;
        result["size"] = cbr2seize_ch_rri[(*ltr)]["size"].get<int>();
        result["ch"] =   cbr2seize_ch_rri[(*ltr)]["ch"].get<int>();
        result["rri"] =  cbr2seize_ch_rri[(*ltr)]["rri"].get<double>();
      } else {
        continue;
      }
    }

    if (be_found) { break; }
  }

  return result;
}
// ----- End: VirtualTxSduQueue -----


// ----- Begin: VirtualRxSduQueue -----
void VirtualRxSduQueue::logging(std::string sender_id, int packet_id, double recv_time) {
  // return;

  this->_recv_logs.push_back({sender_id, packet_id, recv_time});

  auto itr = _recv_logs.begin();
  while (itr != _recv_logs.end()) {
    if ((*itr).recv_time + 0.2 < recv_time) {
      _sender2packet_id2start_byte2sdu[(*itr).sender_id].erase((*itr).packet_id);
      _recv_logs.erase(itr);

    } else {
      break;
    }
  }
}

json VirtualRxSduQueue::enque_and_decode(json sdu, double current_time)
{

  // std::cout << __func__ << ", Begin." << std::endl;
  json result = NULL;

  std::string sender_id = sdu["rlc"]["sender_id"].get<std::string>();
  int packet_id = sdu["packet_id"].get<int>();

  this->logging(sender_id, packet_id, current_time);

  std::vector<json> sdus = {sdu};
  std::vector<json> tmp_sdus = {};
  for (auto itr = _sender2packet_id2start_byte2sdu[sender_id][packet_id].begin(); itr != _sender2packet_id2start_byte2sdu[sender_id][packet_id].end(); itr++) {
    // std::cout << __func__ << ", itr ----- : " << itr->second << std::endl;


    auto jtr = sdus.begin();
    while (jtr != sdus.end()) {
      // std::cout << __func__ << ", jtr +++++ : " << (*jtr) << std::endl;

      double diff_start_byte = (*jtr)["start_byte"].get<int>() - itr->second["start_byte"].get<int>();
      double diff_end_byte = (*jtr)["end_byte"].get<int>() - itr->second["end_byte"].get<int>();

      if (0 <= diff_start_byte && diff_end_byte <= 0) {
        sdus.erase(jtr);

      } else if (diff_start_byte < 0 && diff_end_byte <= 0) {
        (*jtr)["end_byte"] = itr->second["start_byte"].get<int>();
        jtr++;

      } else if (0 <= diff_start_byte && 0 < diff_end_byte) {
        (*jtr)["start_byte"] = itr->second["end_byte"].get<int>();
        jtr++;

      } else {
        json tmp_1 = (*jtr);
        json tmp_2 = (*jtr);

        tmp_1["end_byte"] = itr->second["start_byte"].get<int>();
        tmp_2["start_byte"] = itr->second["end_byte"].get<int>();

        sdus.erase(jtr);
        tmp_sdus.push_back(tmp_1);
        tmp_sdus.push_back(tmp_2);
      }
    }

    auto ltr = tmp_sdus.begin();
    while (ltr != tmp_sdus.end()) {
      sdus.push_back(*ltr);
      tmp_sdus.erase(ltr);
    }
  }

  if (sdus.size() <= 0) {
    return result;

  } else {
    for (auto jtr = sdus.begin(); jtr != sdus.end(); jtr++) {
      _sender2packet_id2start_byte2sdu[sender_id][packet_id][(*jtr)["start_byte"].get<int>()] = (*jtr);
    }

    if (this->is_decoded(sender_id, packet_id, 0)) {
      return sdu;

    } else {
      return result;

    }
  }

  return result;
}

bool VirtualRxSduQueue::is_decoded(std::string sender_id, int packet_id, int start_byte) {
  if (_sender2packet_id2start_byte2sdu[sender_id][packet_id].find(start_byte) == _sender2packet_id2start_byte2sdu[sender_id][packet_id].end()) {
    return false;

  } else {
    if (_sender2packet_id2start_byte2sdu[sender_id][packet_id][start_byte]["end_byte"].get<int>() == _sender2packet_id2start_byte2sdu[sender_id][packet_id][start_byte]["size"].get<int>()) {
      return true;

    } else {
      return this->is_decoded(sender_id, packet_id, _sender2packet_id2start_byte2sdu[sender_id][packet_id][start_byte]["end_byte"].get<int>());

    }
  }
}
// ----- End: VirtualRxSduQueue -----

// ----- Begin: VirtualGeoNetwork -----

void VirtualGeoNetwork::logging(std::string sender_id, int packet_id, double recv_time) {
  // return;

  this->_recv_logs.push_back({sender_id, packet_id, recv_time});

  auto itr = _recv_logs.begin();
  while (itr != _recv_logs.end()) {
    if ((*itr).recv_time + 0.2 < recv_time) {
      _sender_id2packet_id2packet[(*itr).sender_id].erase((*itr).packet_id);
      // _sender_id2packet_id2packet_count[(*itr).sender_id].erase((*itr).packet_id);
      _recv_logs.erase(itr);

    } else {
      break;
    }
  }

}

json VirtualGeoNetwork::header(double sender_pos_x, double sender_pos_y, double dest_pos_x, double dest_pos_y, double hop_limit, double expired_time, std::string sender_id) {

  // std::cout << __func__ << ", Begin." << std::endl;
  json header;

  header["geocast"] = {
    {"sender_id", sender_id},
    {"packet_id", _packet_id},
    {"sender_pos_x", sender_pos_x},
    {"sender_pos_y", sender_pos_y},
    {"dest_pos_x", dest_pos_x},
    {"dest_pos_y", dest_pos_y},
    {"hop_limit", hop_limit},
    {"expired_time", expired_time},
    {"target_center_x", sender_pos_x},
    {"target_center_y", sender_pos_y},
    {"target_radius", 1000}
  };

  _packet_id++;
  return header;
}

json VirtualGeoNetwork::update_header(json packet, inet::Coord sender_coord) {

  // std::cout << __func__ << ", Begin." << std::endl;
  packet["geocast"]["sender_pos_x"] = sender_coord.x;
  packet["geocast"]["sender_pos_y"] = sender_coord.y;

  return packet;
}


json VirtualGeoNetwork::enque(json packet, double current_time) {

  // std::cout << __func__ << ", Begin." << std::endl;
  packet["geocast"]["hop_limit"] = packet["geocast"]["hop_limit"].get<int>() - 1;

  std::string sender_id = packet["geocast"]["sender_id"].get<std::string>();
  int packet_id = packet["geocast"]["packet_id"].get<int>();

  if (this->is_already_received(packet)) {
    _sender_id2packet_id2packet_count[sender_id][packet_id]++;
  } else {
    _sender_id2packet_id2packet_count[sender_id][packet_id] = 1;
  }

  this->logging(sender_id, packet_id, current_time);

  _sender_id2packet_id2packet[sender_id][packet_id] = packet;

  return packet;
}

// void VirtualGeoNetwork::delete_old_packet(std::string sender_id, double current_time) {
//   // std::cout << __func__ << std::endl;
//
//   auto itr = _sender_id2packet_id2packet[sender_id].begin();
//   while (itr != _sender_id2packet_id2packet[sender_id].end()) {
//     if (itr->second["expired_time"].get<double>() < current_time) {
//       _sender_id2packet_id2packet[sender_id].erase(itr);
//     } else {
//       continue;
//     }
//   }
// }

bool VirtualGeoNetwork::is_already_received(json packet) {

  // std::cout << __func__ << ", Begin." << std::endl;
  std::string sender_id = packet["geocast"]["sender_id"].get<std::string>();
  int packet_id = packet["geocast"]["packet_id"].get<int>();

  if (_sender_id2packet_id2packet.find(sender_id) !=  _sender_id2packet_id2packet.end() && _sender_id2packet_id2packet[sender_id].find(packet_id) != _sender_id2packet_id2packet[sender_id].end()) {
    return true;
  } else {
    return false;
  }
}

double VirtualGeoNetwork::CBF_resend_time(json packet, inet::Coord recver_pos, double current_time) {
  // ----- Conducting GN-DATA.confirm [1].
  // ----- [1] Draft ETSI EN 302 636-4-1 V1.4.0 (2019-05)

  // std::cout << __func__ << ", packet" << std::endl;

  double resend_time = -1;

  // this->delete_old_packet(packet["geocast"]["sender_id"].get<std::string>(), current_time);

  // ----- validation hop limit -----
  // std::cout << __func__ << ", packet" << std::endl;
  if (packet["geocast"]["hop_limit"].get<int>() <= 0) {
    return resend_time;
  }

  // ----- validation duplication receive -----
  // std::cout << __func__ << ", packet" << std::endl;
  if (!this->is_resend(packet)) {
    return resend_time;
  }

  // ----- (4) of GN-DATA.confirm -----
  if (current_time - this->_last_resend_time <= this->_itsGnMinPacketRepetitionInterval) {
    return resend_time;
  }

  // ----- validation angle (60) -----
  // std::cout << __func__ << ", packet" << std::endl;
  double recver_pos_x = recver_pos.x;
  double recver_pos_y = recver_pos.y;
  double sender_pos_x = packet["geocast"]["sender_pos_x"].get<double>();
  double sender_pos_y = packet["geocast"]["sender_pos_y"].get<double>();
  double dest_pos_x = packet["geocast"]["dest_pos_x"].get<double>();
  double dest_pos_y = packet["geocast"]["dest_pos_y"].get<double>();

  double target_center_x = packet["geocast"]["target_center_x"].get<double>();
  double target_center_y = packet["geocast"]["target_center_y"].get<double>();
  double target_radius = packet["geocast"]["target_radius"].get<double>();

  double diff_sender_dest_x = dest_pos_x - sender_pos_x;
  double diff_sender_dest_y = dest_pos_y - sender_pos_y;
  double diff_sender_recver_x = recver_pos_x - sender_pos_x;
  double diff_sender_recver_y = recver_pos_y - sender_pos_y;
  // double diff_recver_dest_x = dest_pos_x - recver_pos_x;
  // double diff_recver_dest_y = dest_pos_y - recver_pos_y;

  double diff_sender_target_x = target_center_x - sender_pos_x;
  double diff_sender_target_y = target_center_y - sender_pos_y;


  double dist_sender_dest = sqrt(diff_sender_dest_x * diff_sender_dest_x + diff_sender_dest_y * diff_sender_dest_y);
  double dist_sender_recver = sqrt(diff_sender_recver_x * diff_sender_recver_x + diff_sender_recver_y * diff_sender_recver_y);
  // double dist_recver_dest = sqrt(diff_recver_dest_x * diff_recver_dest_x + diff_recver_dest_y * diff_recver_dest_y);

  double cos_angle = (diff_sender_dest_x * diff_sender_recver_x + diff_sender_dest_y * diff_sender_recver_y) / (dist_sender_dest * dist_sender_recver);

  // ----- Begin Modification -----
  // ----- New Code -----
  // ----- Validation F(x, y) in ETSI -----
  double F = 1 - ( (diff_sender_target_x * diff_sender_target_x) + (diff_sender_target_y * diff_sender_target_y) ) / (target_radius * target_radius);
  if (F < 0) {
    return resend_time;
  }
  // ----- Old Code -----
  // std::cout << __func__ << ", cos_angle: " << cos_angle << std::endl;
  // if (cos_angle < cos(this->_itsGnBroadcastCBFDefSectorAngle)) {
  //   return resend_time;
  //
  // }
  // if (dist_sender_dest <= dist_sender_recver) {
  //   return resend_time;
  //
  // }

  // if (dist_sender_dest <= _DIST_MAX / 2.0) {
  //   return resend_time;
  // }
  // ----- End Modification -----

  // ----- Proposed Method -----
  if (dist_sender_recver <= _DIST_MIN) {
    return resend_time;
  }

  double TO_CBF_GUC;
  if (dist_sender_recver <= _DIST_MAX) {
    TO_CBF_GUC = _TO_CBF_MAX + (_TO_CBF_MIN - _TO_CBF_MAX) * (dist_sender_recver / _DIST_MAX);
  } else {
    TO_CBF_GUC = _TO_CBF_MIN;
  }

  // ----- validation expired_time -----
  // std::cout << __func__ << ", packet" << std::endl;


  // ----- (3) of GN-DATA.confirm -----
  // ----- We use expired_time instead of itsGnDefaultPacketLifetime.
  // ----- In generally, itsGnDefaultPacketLifetime is between 60 and 600 [1].
  // ----- On the other hand, the expired_time is at most 0.1 sec.
  // ----- Therefore, it becoes difficult to handle such packets.
  // ----- [1] Draft ETSI EN 302 636-4-1 V1.4.0 (2019-05)
  if (packet["geocast"]["expired_time"].get<double>() < current_time + TO_CBF_GUC) {
    return resend_time;
  }

  resend_time = current_time + TO_CBF_GUC;

  return resend_time;
}


void VirtualGeoNetwork::resend_enque(double resend_time, json packet) {

  // std::cout << __func__ << ", Begin." << std::endl;
  if (_resend_time2packets.find(resend_time) == _resend_time2packets.end()) {
    _resend_time2packets[resend_time] = {};
  }

  _resend_time2packets[resend_time].push_back(packet);
}

bool VirtualGeoNetwork::is_resend(json packet) {

  // std::cout << __func__ << ", Begin." << std::endl;
  std::string sender_id = packet["geocast"]["sender_id"].get<std::string>();
  int packet_id = packet["geocast"]["packet_id"].get<int>();

  // std::cout << __func__ << ", Geonet Duplication: " << _sender_id2packet_id2packet_count[sender_id][packet_id] << std::endl;
  if (this->_MAX_CBF_PACKET_COUNT <= _sender_id2packet_id2packet_count[sender_id][packet_id]) {
    return false;
  } else {
    return true;
  }
}


std::vector<json> VirtualGeoNetwork::resend_deque(double resend_time) {
  std::vector<json> packets = {};

  // ----- (4) of GN-DATA.confirm -----
  if (resend_time - this->_last_resend_time <= this->_itsGnMinPacketRepetitionInterval) {
    return packets;
  }

  for (auto itr = _resend_time2packets[resend_time].begin(); itr != _resend_time2packets[resend_time].end(); itr++) {
    if (!this->is_resend(*itr)) {
      continue;
    } else {
      packets.push_back(*itr);

      this->_last_resend_time = resend_time;
      // ----- Considering -----
      break;
    }
  }

  _resend_time2packets.erase(resend_time);

  return packets;
}


std::vector<json> VirtualGeoNetwork::resend_deque_by_resource(double current_time, double duration, int lefted_size, int lowlayer_overhead) {
  std::vector<json> packets = {};
  int total_size = 0;

  // ----- (4) of GN-DATA.confirm -----
  auto itr = _resend_time2packets.begin();
  while (itr != _resend_time2packets.end()) {

    // std::cout << __func__ << ", " << itr->first << ", " << current_time << std::endl;
    if (itr->first < current_time) {
      itr = _resend_time2packets.erase(itr);

    } else {
      auto jtr = itr->second.begin();

      while (jtr != itr->second.end()) {
        // std::cout << (*jtr) << std::endl;
        // std::cout << __func__ << ", duration: " << duration << ", expired_time: " << (*jtr)["geocast"]["expired_time"].get<double>() << ", current_time: " << current_time << ", total_size: " << total_size << ", leftted_size: " << lefted_size << std::endl;

        // if (!this->is_resend(*jtr)) {
        //   jtr++;
        //
        // } else {
        //   (*jtr)["rlc"]["resource_consider_time"] = itr->first;
        //   packets.push_back(*jtr);
        //   jtr = itr->second.erase(jtr);
        //
        // }
        //
        // if (lefted_size <= total_size) {
        //   break;
        // }

        if (total_size + (*jtr)["size"].get<int>() + lowlayer_overhead <= lefted_size && duration <= (*jtr)["geocast"]["expired_time"].get<double>() - current_time && this->is_resend(*jtr)) {
          total_size += (*jtr)["size"].get<int>() + lowlayer_overhead;

          // (*jtr)["rlc"]["resource_consider_time"] = itr->first;
          packets.push_back(*jtr);
          jtr = itr->second.erase(jtr);
        } else {
          jtr++;

        }
      }

      itr++;
    }

    if (lefted_size <= total_size) {
      break;
    }
  }

  return packets;
}

std::vector<std::string> VirtualGeoNetwork::duplication_packets_count() {
  json results = {};

  for (auto itr = _sender_id2packet_id2packet_count.begin(); itr != _sender_id2packet_id2packet_count.end(); itr++) {
    for (auto jtr = itr->second.begin(); jtr != itr->second.end(); jtr++) {
      std::string key = std::to_string(jtr->second);

      if (results.find(key) == results.end()) {
        results[key] = 0;
      }

      results[key] = results[key].get<int>() + 1;

      if (10 <= results[key].get<int>()) {
        // std::cout << __func__ << ", 10 dup packet: " << _sender_id2packet_id2packet[itr->first][jtr->first] << std::endl;
      }
    }
  };

  return {results.dump()};
}
// ----- End: VirtualGeoNetwork -----

// ----- Begin: function -----
std::string grants_file_path(std::string data_sync_dir, std::string sumo_id)
{
  return data_sync_dir + sumo_id + "_grant.json";
}


std::string grants_rec_file_path(std::string data_sync_dir, std::string sumo_id)
{
  return data_sync_dir + sumo_id + "_grant_rec.json";
}

std::string dup_count_file_path(std::string data_sync_dir, std::string sumo_id)
{
  return data_sync_dir + sumo_id + "_geo_dup.json";
}

std::string cbr_file_path(std::string data_sync_dir, std::string sumo_id)
{
  return data_sync_dir + sumo_id + "_cbr.json";
}

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


template <class X> X min(X v1, X v2) {
  if (v1 <= v2) {
    return v1;
  } else {
    return v2;
  }
}

template <class X> X max(X v1, X v2) {
  if (v1 >= v2) {
    return v1;
  } else {
    return v2;
  }
}

// ----- End: function -----
