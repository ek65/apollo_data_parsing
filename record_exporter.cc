/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <assert.h>
#include <fstream>
#include <iostream>
#include <string>

#include "cyber/cyber.h"
#include "cyber/common/file.h"
#include "cyber/message/raw_message.h"
#include "cyber/proto/record.pb.h"
#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "cyber/record/record_writer.h"

#include "modules/control/proto/control_cmd.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

using ::apollo::cyber::record::RecordReader;
using ::apollo::cyber::record::RecordMessage;
// using apollo::cyber::message::RawMessage;

using apollo::cyber::common::GetFileName;

void DisplayUsage(const std::string& binary) {
  std::cerr << "usage: " << binary << " <record_fiename>\n"
            << std::endl;
}

void DumpTextMessage(const google::protobuf::Message& msg) {
  std::string msg_text;

  google::protobuf::TextFormat::PrintToString(msg, &msg_text);

  std::cerr << "[" << msg.GetTypeName() << "]"
            << std::endl << msg_text << std::endl;
}

void ExportRecords(const std::string &readfile) {
  using namespace std;
  RecordReader reader(readfile);
  RecordMessage message;
  
  auto pose_filename = "out/" + GetFileName(readfile) + "-pose.csv";
  auto obst1_filename = "out/" + GetFileName(readfile) + "-obstacle1.csv";
  auto obst2_filename = "out/" + GetFileName(readfile) + "-obstacle2.csv";
  auto obst3_filename = "out/" + GetFileName(readfile) + "-obstacle3.csv";
  auto obst4_filename = "out/" + GetFileName(readfile) + "-obstacle4.csv";
  auto obst5_filename = "out/" + GetFileName(readfile) + "-obstacle5.csv";

  std::ofstream pose_file(pose_filename, std::ios::out);
  // std::ofstream obst1_file(obst1_filename, std::ios::out);
  // std::ofstream obst2_file(obst2_filename, std::ios::out);
  // std::ofstream obst3_file(obst3_filename, std::ios::out);
  // std::ofstream obst4_file(obst4_filename, std::ios::out);
  // std::ofstream obst5_file(obst5_filename, std::ios::out);

  std::vector<std::shared_ptr<ofstream> > files;
  for(int i =0; i < 25; ++i){
    std::shared_ptr<ofstream> out(new std::ofstream);
    auto obst_filename = "out/" + GetFileName(readfile) + "-obstacle"+ to_string(i) +".csv";
    out->open(obst_filename, std::ios::out | std::ofstream::app);
    *out << "timestamp_sec;" << "id;" << "theta;" << "type;"
      << "position_x;" << "position_y;" << "position_z;"
      << "velocity_x;" << "velocity_y;" << "velocity_z;"
      << "acceleration_x;" << "acceleration_y;" << "acceleration_z;"
      << std::endl;
    files.push_back(out);
  }

  if (!pose_file.is_open()) { 
    AERROR << "Can't open output file '" << pose_filename << "'";
    AERROR << "Make sure target folder exists and is writable.";
    return;
  }

  pose_file << "timestamp_sec;"
    << "position_x;" << "position_y;" << "position_z;"
    << "heading;"
    << "qx;" << "qy;" << "qz;" << "qw;"
    << "euler_x;" << "euler_y;" << "euler_z;"
    << "linear_accel_x;" << "linear_accel_y;" << "linear_accel_z;"
    << "angular_vel_x;" << "angular_vel_y;" << "angular_vel_z;" 
    << "linear_vel_x;" << "linear_vel_y;" << "linear_vel_z;" 
    << "heading;"
    << std::endl;
    
  std::cerr << "Write GNSS Pos to " << pose_filename << std::endl;

  // read all messages
  uint64_t i = 0;
  unsigned id_count = 0;
  std::map<int, std::shared_ptr<ofstream>> obs_map;

  for (i = 0; reader.ReadMessage(&message); ++i) {

    if (message.channel_name.compare("/apollo/perception/obstacles") == 0) {
      auto msg = apollo::perception::PerceptionObstacles();
      
      // Parse message content
      if (msg.ParseFromString(message.content)) {
        // DumpTextMessage(msg);
        // dump message to check fields
        if(i==1){
          DumpTextMessage(msg);
        }
        
        // process message data: save some fields to CSV
        const auto& obstacles = msg.perception_obstacle();
        
        if (obstacles.size() >= 1){
          cout << "obstacle size > 1 \n";

          int j = 0;

          for (j = 0; j < obstacles.size(); ++j){
            const auto& obst = obstacles[j];
            const auto& id = obst.id();
            
            const auto& position = obst.position();
            const auto& velocity = obst.velocity();
            const auto& theta = obst.theta();
            const auto& acceleration = obst.acceleration();
            const auto& type = obst.type();

            if(obs_map.find(id) == obs_map.end()){
              obs_map[id] = files[id_count];
              ++id_count;
              cout << "count updated to : " << to_string(id_count) << "\n";
            }

            cout << "writing to the object #: " << to_string(j) << "\n";

            if(obs_map[id]->is_open()){
              *obs_map[id] << std::setprecision(6) << std::fixed 
              << msg.header().timestamp_sec() << ";"
              << id << ";" << theta <<";" << type <<";"
              << position.x() << ";" << position.y() << ";" << position.z() << ";"
              << velocity.x() << ";" << velocity.y() << ";" << velocity.z() << ";"
              << acceleration.x() << ";" << acceleration.y() << ";" << acceleration.z() << ";"
              << std::endl;
            }
          }
          cout << "all obstacles looped through\n";
        }

      } else {
        // AERROR << "Failed to parse protobuf" << std::endl;
      }
    }
    
    if (message.channel_name.compare("/apollo/localization/pose") == 0) {
      // Create message object of appropriate type.
      // Check the output of 'cyber_recorder info' for the exact message types.
      auto msg = apollo::localization::LocalizationEstimate();
      
      // Parse message content
      if (msg.ParseFromString(message.content)) {
        // dump message to check fields
        // DumpTextMessage(msg);
        
        // process message data: save some fields to CSV
        const auto& pose = msg.pose();
        const auto& position = pose.position();
        const auto& orientation = pose.orientation();
        const auto& euler_angles = pose.euler_angles();
        const auto& linear_acceleration = pose.linear_acceleration();
        const auto& angular_vel = pose.angular_velocity();
        const auto& linear_vel = pose.linear_velocity();
        const auto& heading = pose.heading();
        
        // Make sure fields set and order match to the header written above
        
        pose_file << std::setprecision(6) << std::fixed << msg.header().timestamp_sec() << ";"
          << position.x() << ";" << position.y() << ";" << position.z() << ";"
          << pose.heading() << ";"
          << orientation.qx() << ";" << orientation.qy() << ";"<< orientation.qz() << ";"<< orientation.qw() << ";"
          << euler_angles.x() << ";" << euler_angles.y() << ";"<< euler_angles.z() << ";"
          << linear_acceleration.x() << ";" << linear_acceleration.y() << ";" << linear_acceleration.z() << ";" 
          << angular_vel.x() << ";" << angular_vel.y() << ";" << angular_vel.z() << ";"
          << linear_vel.x() << ";" << linear_vel.y() << ";" << linear_vel.z() << ";"
          << heading << ";"
          << std::endl;

      } else {
        AERROR << "Failed to parse protobuf" << std::endl;
      }
    }
  }

  for (int i= 0; i <25 ; ++i){
    files[i]->close();
  }
  cout << "files all closed\n";  
}
// std::cerr << "MSG totalcount: " << i << std::endl;


using namespace std;
int main(int argc, char *argv[]) {
  std::string binary = GetFileName(std::string(argv[0]));
  
  if (argc < 2) {
    DisplayUsage(binary);
    return -1;
  }
  
  apollo::cyber::Init(argv[0]);
  
  sleep(1);
  std::string file_name(argv[1]);
  
  // TODO setup logger to use ADEBUG/AINFO instead of std::cerr

  std::cerr << "Reading " << file_name << std::endl;
  ExportRecords(file_name);
  return 0;
}