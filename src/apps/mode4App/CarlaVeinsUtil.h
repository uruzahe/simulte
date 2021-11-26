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
#define MY_PDCP_HEADER_BYTE 1
#define MY_SDAP_HEADER_BYTE 1
#define MY_GEONETWORK_HEADER 56
// ----- MY_GEONETWORK_HEADER (GBC header) -----
// 56 (Byte) = 4 + 8 + 24 + 20, V, E. E. N. (2014). Vehicular Communications ; GeoNetworking ; 1, 1–104 ETSI EN 302 636-4-1 V1.2.1.
// 4 byte: Basic Header
// 8 byte: Common Header
// 24 byte: Long Position Vector
// 20 byte: Other information

#define ALL_HEADER_SIZE (MY_MAC_HEADER_BYTE + MY_RLC_UM_HEADER_BYTE + MY_PDCP_HEADER_BYTE + MY_SDAP_HEADER_BYTE + MY_GEONETWORK_HEADER)

#define MY_PACKET_LIFE_TIME 0.1
// ----- data_between_time -----
// This variable is equal to the life time in Transport Layer.


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
  int _past_fragment_count = 0;
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
  std::unordered_map<int, int> _ch2size;
  std::map<double, json> cbr2seize_ch_rri;
  std::vector<double> cbrs;

  double _max_size;
  double _max_rri;
  double _max_ch;

  double _min_rri;

  VirtualTxSduQueue();

  json header(std::string sender_id, int priority, double expired_time, double resource_consider_time);
  json update_header(json packet, std::string sender_id, double resource_consider_time);

  json add_fragment_into_pdu(json pdu, json send_fragment, double current_time);
  void delete_expired_fragments(double current_time);
  bool enque(json packet, double current_time);
  // json formatted_packet(std::string payload, std::string type, int payload_byte_size, double current_time, double duration);
  json formatted_fragment(json packet, int leftted_size, int status_flag, int start_byte, int end_byte);
  json formatted_pdu(int maximum_size, double current_time);
  json generate_PDU(int maximum_byte, double current_time);
  json minimum_Bps(double current_time);
  json update_fragment(json fragment, int lefted_size, int status_flag, int start_byte, int end_byte);
  json update_pdu_by_fragment(json pdu, double current_time);
  json Bps2packet_size_and_rri(double minimum_Bps);

  int leftted_size_in_PDU(int maximum_byte, double current_time);

  bool is_empty(double current_time);
  double maximum_duration(double current_time);
  double resource_selection_time(double current_time);
  json get_duration_size_rri(double current_time, double maximum_duration);
};

struct LogData {
  std::string sender_id;
  int packet_id;
  double recv_time;
};

class VirtualRxSduQueue
{
public:
  // std::unordered_map<std::string, std::unordered_map<int, std::vector<json>>> _sender2packet_id2sdus;
  std::vector<struct LogData> _recv_logs;
  std::unordered_map<std::string, std::unordered_map<int, std::unordered_map<int, json>>> _sender2packet_id2start_byte2sdu;

  void logging(std::string sender_id, int packet_id, double recv_time);
  json enque_and_decode(json fragment, double current_time);
  bool is_decoded(std::string sender_id, int packet_id, int start_byte);
};

class VirtualGeoNetwork
{
public:
  // int _itsGnMaxSduSize;
  // int _itsGnMaxPacketLifetime;
  // int _itsGnMinPacketRepetitionInterval;
  // int _itsGnMaxGeoAreaSize;

  int _packet_id = 0;
  std::vector<struct LogData> _recv_logs;
  std::unordered_map<std::string, std::unordered_map<int, json>> _sender_id2packet_id2packet;
  std::unordered_map<std::string, std::unordered_map<int, int>> _sender_id2packet_id2packet_count;
  std::map<double, std::vector<json>> _resend_time2packets;
  std::vector<double> _resend_times;
  double _last_resend_time = 0;

  double _TO_CBF_MIN = 0.001;
  // double _TO_CBF_MAX = 0.1;
  double _TO_CBF_MAX = 0.010;

  double _DIST_MAX = 1000;
  // double _DIST_MAX = 400;
  // double _DIST_MAX = 0;

  double _PROPOSED_DIST_RATE = 0.0; // PDR(distance=500m) >= 90%
  double _DIST_MIN = _DIST_MAX * _PROPOSED_DIST_RATE;


  double _itsGnBroadcastCBFDefSectorAngle = 30;
  // double _itsGnMinPacketRepetitionInterval = 0.1;
  double _itsGnMinPacketRepetitionInterval = 0;
  // int _itsGnLocationServiceMaxRetrans = 2;
  int _MAX_CBF_PACKET_COUNT = 2;
  // ----- itsGnLocationServiceMaxRetrans -----
  // ----- Default itsGnLocationServiceMaxRetrans is 10 [1]. -----
  // ----- However, we set the value as 1 to mitigate broadcast storm -----
  // ----- [1] Draft ETSI EN 302 636-4-1 V1.4.0 (2019-05)

  void logging(std::string sender_id, int packet_id, double recv_time);
  json header(double sender_pos_x, double sender_pos_y, double dest_pos_x, double dest_pos_y, double hop_limit, double expired_time, std::string sender_id);
  json update_header(json packet, inet::Coord sender_coord);
  void delete_old_packet(std::string sender_id, double current_time);
  bool is_already_received(json packet);
  bool is_resend(json packet);
  json enque(json packet, double current_time);
  double CBF_resend_time(json packet, inet::Coord recver_pos, double current_time);
  void resend_enque(double resend_time, json packet);
  std::vector<json> resend_deque(double resend_time);
  std::vector<json> resend_deque_by_resource(double current_time, double duration, int leftted_size, int lowlayer_overhead);
  std::vector<std::string> duplication_packets_count();

  std::map<double, std::vector<json>> time2resend_packets;
  std::vector<json> deque();
};


json add_time_attribute_to_json(json data, std::string attr_name, double t);

std::string grants_file_path(std::string data_sync_dir, std::string sumo_id);

std::string grants_rec_file_path(std::string data_sync_dir, std::string sumo_id);

std::string cbr_file_path(std::string data_sync_dir, std::string sumo_id);

std::string cams_json_file_path(std::string data_sync_dir, std::string sumo_id);

std::string cams_recv_json_file_path(std::string data_sync_dir, std::string sumo_id);

std::string cpms_json_file_path(std::string data_sync_dir, std::string sumo_id);

std::string dup_count_file_path(std::string data_sync_dir, std::string sumo_id);

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

template <class X> X max(X v1, X v2);
