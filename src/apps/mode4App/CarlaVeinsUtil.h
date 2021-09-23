#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <vector>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

class JsonData
{
public:
  JsonData();

  std::vector<json> _data;

  void load_json_str(std::string json_str);
  void load_json_strs(std::vector<std::string> json_strs);
  std::vector<json> pop_all();
  json pop_back();
  void push_back(json data);
};

class CAMs : public JsonData
{

};

class PerceivesObjectes : public JsonData
{

};


class CAMsHandler
{

};

class CPMsHandler
{

};


std::string objects_json_file_path(std::string data_sync_dir, std::string sumo_id);

std::string packet_json_file_path(std::string data_sync_dir, std::string sumo_id);

std::string sensor_json_file_path(std::string data_sync_dir, std::string sumo_id);

int existFile(const char* path);

std::vector<std::string> file2string_vector(const char* path);

int carla_lock_wait(std::string data_sync_dir);

void lock(const char *oldpath, const char *newpath);

void set_cpm_payloads_for_carla(std::string sumo_id, std::string data_sync_dir, std::vector<std::string> payloads);

std::vector<std::string> get_cpm_payloads_from_carla(std::string sumo_id, std::string data_sync_dir, bool read_only);
