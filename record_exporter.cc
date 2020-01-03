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


using namespace std; 
  
// Define Infinite (Using INT_MAX caused overflow problems) 
// Polygon related code copied from : https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
#define INF 10000 
  
struct Point 
{ 
    int x; 
    int y; 
}; 
  
// Given three colinear points p, q, r, the function checks if 
// point q lies on line segment 'pr' 
bool onSegment(Point p, Point q, Point r) 
{ 
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && 
            q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y)) 
        return true; 
    return false; 
} 
  
// To find orientation of ordered triplet (p, q, r). 
// The function returns following values 
// 0 --> p, q and r are colinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise 
int orientation(Point p, Point q, Point r) 
{ 
    int val = (q.y - p.y) * (r.x - q.x) - 
              (q.x - p.x) * (r.y - q.y); 
  
    if (val == 0) return 0;  // colinear 
    return (val > 0)? 1: 2; // clock or counterclock wise 
} 
  
// The function that returns true if line segment 'p1q1' 
// and 'p2q2' intersect. 
bool doIntersect(Point p1, Point q1, Point p2, Point q2) 
{ 
    // Find the four orientations needed for general and 
    // special cases 
    int o1 = orientation(p1, q1, p2); 
    int o2 = orientation(p1, q1, q2); 
    int o3 = orientation(p2, q2, p1); 
    int o4 = orientation(p2, q2, q1); 
  
    // General case 
    if (o1 != o2 && o3 != o4) 
        return true; 
  
    // Special Cases 
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
    if (o1 == 0 && onSegment(p1, p2, q1)) return true; 
  
    // p1, q1 and p2 are colinear and q2 lies on segment p1q1 
    if (o2 == 0 && onSegment(p1, q2, q1)) return true; 
  
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
    if (o3 == 0 && onSegment(p2, p1, q2)) return true; 
  
     // p2, q2 and q1 are colinear and q1 lies on segment p2q2 
    if (o4 == 0 && onSegment(p2, q1, q2)) return true; 
  
    return false; // Doesn't fall in any of the above cases 
} 
  
// Returns true if the point p lies inside the polygon[] with n vertices 
bool isInside(Point polygon[], int n, Point p) 
{ 
    // There must be at least 3 vertices in polygon[] 
    if (n < 3)  return false; 
  
    // Create a point for line segment from p to infinite 
    Point extreme = {INF, p.y}; 
  
    // Count intersections of the above line with sides of polygon 
    int count = 0, i = 0; 
    do
    { 
        int next = (i+1)%n; 
  
        // Check if the line segment from 'p' to 'extreme' intersects 
        // with the line segment from 'polygon[i]' to 'polygon[next]' 
        if (doIntersect(polygon[i], polygon[next], p, extreme)) 
        { 
            // If the point 'p' is colinear with line segment 'i-next', 
            // then check if it lies on segment. If it lies, return true, 
            // otherwise false 
            if (orientation(polygon[i], p, polygon[next]) == 0) 
               return onSegment(polygon[i], p, polygon[next]); 
  
            count++; 
        } 
        i = next; 
    } while (i != 0); 
  
    // Return true if count is odd, false otherwise 
    return count&1;  // Same as (count%2 == 1) 
} 

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
  
  auto pose_filename = "out/" + GetFileName(readfile) + "-ego.csv";
  auto obst1_filename = "out/" + GetFileName(readfile) + "-pedestrian.csv";

  std::ofstream pose_file(pose_filename, std::ios::out);
  std::ofstream obst1_file(obst1_filename, std::ios::out);

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

  obst1_file << "timestamp_sec;" << "id;" << "theta;" << "type;"
    << "position_x;" << "position_y;" << "position_z;"
    << "velocity_x;" << "velocity_y;" << "velocity_z;"
    << "acceleration_x;" << "acceleration_y;" << "acceleration_z;"
    << std::endl;
    
  std::cerr << "Write GNSS Pos to " << pose_filename << std::endl;

  // read all messages
  uint64_t i = 0;
  std::map<int, std::shared_ptr<ofstream>> obs_map;

  Point polygon1[] = {{586610, 4208097}, {586615, 4208098}, {586618, 4208090}, {586613, 4208089}}; 
  int n = (int) (sizeof(polygon1)/sizeof(polygon1[0])); 

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

            cout << "writing to the object #: " << to_string(j) << "\n";
            
            Point p = {(int) position.x(), (int) position.y()}; 
            if(isInside(polygon1, n, p)){
              obst1_file << std::setprecision(6) << std::fixed 
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

  // for (int i= 0; i <25 ; ++i){
  //   files[i]->close();
  // }
  // cout << "files all closed\n";  
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

  // Point polygon1[] = {{0, 0}, {10, 0}, {10, 10}, {0, 10}}; 
  // int n = (int) (sizeof(polygon1)/sizeof(polygon1[0])); 

  // Point p = {20, 20}; 
  // isInside(polygon1, n, p)? cout << "Yes \n": cout << "No \n"; 

  // p = {5, 5}; 
  // isInside(polygon1, n, p)? cout << "Yes \n": cout << "No \n"; 

  // Point polygon2[] = {{0, 0}, {5, 5}, {5, 0}}; 
  // p = {3, 3}; 
  // n = (int) (sizeof(polygon2)/sizeof(polygon2[0])); 
  // isInside(polygon2, n, p)? cout << "Yes \n": cout << "No \n"; 

  // p = {5, 1}; 
  // isInside(polygon2, n, p)? cout << "Yes \n": cout << "No \n"; 

  // p = {8, 1}; 
  // isInside(polygon2, n, p)? cout << "Yes \n": cout << "No \n"; 

  // Point polygon3[] =  {{0, 0}, {10, 0}, {10, 10}, {0, 10}}; 
  // p = {-1,10}; 
  // n = (int) (sizeof(polygon3)/sizeof(polygon3[0])); 
  // isInside(polygon3, n, p)? cout << "Yes \n": cout << "No \n"; 

  return 0;
}