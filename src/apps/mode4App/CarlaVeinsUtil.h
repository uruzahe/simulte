#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

class JsonDataStore
{
public:
  JsonDataStore();

  std::vector<json> _data;

  bool is_empty();
  void load_json_str(std::string json_str);
  void load_json_strs(std::vector<std::string> json_strs);
  std::vector<json> pop_all();
  json pop_back();
  void push_back(json data);

  std::vector<json> data_between_time(double begin_time, double end_time);
};

class CAMs : public JsonDataStore
{

};

class PerceivedObjectes : public JsonDataStore
{

};

class CAMHandler : JsonDataStore
{
public:
  std::vector<json> filter_cams_by_etsi(std::vector<json> cams);

  json convert_payload_and_size(json cam, int max_payload_byte);

};

class CAMRecvHandler : public CAMHandler
{
};

class CAMSendHandler : public CAMHandler
{
};


class POHandler
{
public:
  POHandler();

  std::unordered_map<std::string, json> _pseu2po;

  std::vector<json> filter_pos_by_etsi(std::vector<json> perceived_objects);

  json convert_payload_and_size(std::vector<json> perceived_objects, int sensor_num, int max_payload_byte);

};

class PORecvHandler : public POHandler
{
public:

};

class POSendHandler : public POHandler
{
};

json add_time_attribute_to_json(json data, std::string attr_name, double t);

std::string cams_json_file_path(std::string data_sync_dir, std::string sumo_id);

std::string cams_recv_json_file_path(std::string data_sync_dir, std::string sumo_id);

std::string cpms_json_file_path(std::string data_sync_dir, std::string sumo_id);

std::string objects_json_file_path(std::string data_sync_dir, std::string sumo_id);

std::string objects_recv_json_file_path(std::string data_sync_dir, std::string sumo_id);

std::string packet_json_file_path(std::string data_sync_dir, std::string sumo_id);

std::string sensor_json_file_path(std::string data_sync_dir, std::string sumo_id);

int existFile(const char* path);

void touch(std::string file_path);

std::vector<std::string> file2string_vector(std::string file_path, bool read_only);

void string_vector2file(std::string file_path, std::vector<std::string> str_vec);

int carla_lock_wait(std::string data_sync_dir);

void lock(const char *oldpath, const char *newpath);

void set_cpm_payloads_for_carla(std::string sumo_id, std::string data_sync_dir, std::vector<std::string> payloads);

std::vector<std::string> get_cpm_payloads_from_carla(std::string sumo_id, std::string data_sync_dir, bool read_only);
