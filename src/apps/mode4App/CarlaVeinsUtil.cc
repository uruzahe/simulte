#include "apps/mode4App/CarlaVeinsUtil.h"

using json = nlohmann::json;

// ----- Begin: JsonData-----
JsonData::JsonData() {
  _data = {};
}

void JsonData::load_json_str(std::string json_str){
  try {
    _data.push_back(json::parse(json_str));

  } catch (...) {
    // ignore the json_str.
  }
}

void JsonData::load_json_strs(std::vector<std::string> json_strs){
  for (auto data_pointer = json_strs.begin(); i != json_strs.end(); i++) {
    load_json_str(*data_pointer);
  }
}


std::vector<json> JsonData::pop_all(){
  std::vector<json> pop_data = _data;
  _data = {};

  return pop_data;
}

json JsonData::pop_back(){
  return _data.pop_back();
}


void JsonData::push_back(json data){
  _data.push_back(data);
}
// ----- End: JsonData-----

// ----- Begin: function -----
std::string objects_json_file_path(std::string data_sync_dir, std::string sumo_id)
{
  return data_sync_dir + sumo_id + "_objects.json";
}

std::string packet_json_file_path(std::string data_sync_dir, std::string sumo_id);
{
  return data_sync_dir + sumo_id + "_packet.json";
}

std::string sensor_json_file_path(std::string data_sync_dir, std::string sumo_id);
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


std::vector<std::string> file2str_vec(std::string file_path){

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
    return file2str_vec(data_sync_dir + sensor_data_file_name);
    //    unlink((data_sync_dir + sensor_lock_file_name).c_str());
}
// ----- End: function -----
