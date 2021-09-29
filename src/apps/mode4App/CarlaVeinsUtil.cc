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
    // std::cout << json_str << std::endl;
    _data.push_back(json::parse(json_str));

  } catch (...) {
    // ignore the json_str.
  }
}

void JsonDataStore::load_json_strs(std::vector<std::string> json_strs){
  for (auto data_pointer = json_strs.begin(); data_pointer != json_strs.end(); data_pointer++) {
    load_json_str(*data_pointer);
  }
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
      // std::cout << "The packet is too old, so erase it." << std::endl;
      _data.erase(ptr);

    } else if (begin_time <= timestamp && timestamp < end_time) {
      // std::cout << "The packet is created now, so send it." << std::endl;
      results.push_back(d);
      _data.erase(ptr);

    } else {
      // std::cout << "The packet should be sent in the next timestamp, so break" << std::endl;
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
  if (!existFile(file_path.c_str())) {
      FILE *fp;
      fp = fopen(file_path.c_str(), "w");
      fclose(fp);
  }
}

void string_vector2file(std::string file_path, std::vector<std::string> str_vec) {
    std::ofstream ofs(file_path, std::ios::app);

    if (ofs.is_open()) {
        for (auto payload = str_vec.begin(); payload != str_vec.end(); payload++) {
            // std::cout << "---: "<< *payload << "---" << std::endl;
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
// ----- End: function -----
