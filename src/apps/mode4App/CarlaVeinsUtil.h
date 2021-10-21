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

#include "common/LteCommon.h"

using json = nlohmann::json;

// Cite from: Tuo, X., Wang, F., Zhao, Z., Zhang, Y., & Wang, D. (2017). Packet segmentation for contention-based transmission in 5G. 2017 32nd General Assembly and Scientific Symposium of the International Union of Radio Science, URSI GASS 2017, 2017-Janua(August), 1–4. https://doi.org/10.23919/URSIGASS.2017.8104993
#define MY_MAC_HEADER_BYTE 2
#define MY_RLC_UM_HEADER_BYTE 1
#define MY_PDCP_HEADER_BYTE 1 // PDCP_HEADER is added in OpenCV2X default codes, therefore we don't use MY_PDCP_HEADER_BYTE.
#define MY_GEONETWORK_HEADER 14 // 14 (Byte) = 4 + 8 + 2, V, E. E. N. (2014). Vehicular Communications ; GeoNetworking ; 1, 1–104 ETSI EN 302 636-4-1 V1.2.1.


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

};

class POSendHandler : public POHandler
{
};

class VirtualTxSduQueue
{
public:
  json _fragment;
  std::vector<json> _past_fragments;
  std::unordered_map<int, json> _priority2packets;

  int _packet_id;
  double _delete_expired_time;


  std::vector<double> possible_rris = {0.1, 0.05, 0.02};
  std::map<int, int> bytes2channel_num_in_MCS7 = {
    {150, 1},
    {300, 2},
    {450, 3},
    {600, 4},
    {750, 5}
  };
  std::map<double, json> cbr2seize_ch_rri;
  std::vector<double> cbrs;

  VirtualTxSduQueue();

  json add_fragment_into_pdu(json pdu, json send_fragment, double current_time);
  void delete_expired_fragments(double current_time);
  void enque(json packet);
  // json formatted_packet(std::string payload, std::string type, int payload_byte_size, double current_time, double duration);
  json formatted_fragment(json packet, int leftted_size, int status_flag);
  json formatted_pdu(int maximum_size, double current_time);
  json generate_PDU(int maximum_byte, double current_time);
  json minimum_Bps(double current_time);
  json update_fragment(json fragment, int lefted_size, int status_flag);
  json update_pdu_by_fragment(json pdu, double current_time);
  json Bps2packet_size_and_rri(double minimum_Bps);
};

class VirtualRxSduQueue
{
public:
  // std::unordered_map<std::string, std::unordered_map<std::string, std::vector<json>>> _sender2packet_id2sdus;
  std::unordered_map<std::string, std::unordered_map<int, std::vector<json>>> _sender2packet_id2sdus;

  json enque_and_decode(json fragment);
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

template <class X> X min(X v1, X v2);
