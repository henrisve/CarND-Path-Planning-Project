#include <uWS/uWS.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  vector<double> tmp_map_waypoints_x;
  vector<double> tmp_map_waypoints_y;
  vector<double> tmp_map_waypoints_s;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    tmp_map_waypoints_x.push_back(x);
    tmp_map_waypoints_y.push_back(y);
    tmp_map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  tk::spline sp_sx;
  tk::spline sp_sy;
  sp_sx.set_points(tmp_map_waypoints_s, tmp_map_waypoints_x);
  sp_sy.set_points(tmp_map_waypoints_s, tmp_map_waypoints_y);

  for(int s=1;s<=6944;s+=5){
    map_waypoints_x.push_back(sp_sx(s));
    map_waypoints_y.push_back(sp_sy(s));
    map_waypoints_s.push_back(s);
  }

  /*
  std::ofstream myfile ("example.txt");
  if (myfile.is_open()){
    for(double ss=0;ss< max_s; ss++){
     // double ss = 1;
      vector<double> xydata = getXY(ss, 0, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      myfile << xydata[0]<< "," << xydata[1] << "\n";
    }
    myfile.close();
  }*/
  // start in lane 1 (as in qa video)
  int lane = 1;
  // reference velocity, less than 50
  double ref_vel = 49.5;//0.224;
 // double target_speed=49.5;&target_speed,
  double max_speed=49.5;
  double d_error=0;
  int stay_lane_counter = 0;

  struct cars{
    double v=0;
    double a=0;
    double dist=0;
  };
  //std::map<int, std::pair<float, float>> prev_car_state;
  std::map<int, cars> prev_car_state;
  vector<vector<bool>> map(3, std::vector<bool>(30));  // map of everything within 90 timesteps
  vector<vector<char>> map_s(3, std::vector<char>(90));  // map of everything within 90 meters
  vector<vector<char>> map_2(3, std::vector<char>(30));  // map of everything within 90 meters
  path predicted_path = {};
  //using namespace ;

  std::chrono::high_resolution_clock::time_point t1 =  std::chrono::high_resolution_clock::now();  // remove one second for first itteration.
  // what will happen the first itteration 
  

  h.onMessage([&ref_vel, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &lane, &map, &map_s, &t1,
               &prev_car_state, &map_2,&predicted_path,&max_speed,
               &stay_lane_counter, &d_error](uWS::WebSocket<uWS::SERVER> ws,
                                        char *data, size_t length,
                                        uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          std::chrono::duration<double, std::milli>  td = std::chrono::high_resolution_clock::now() - t1;
          t1 =  std::chrono::high_resolution_clock::now();
          double time_delta = td.count()/1000;
          // j[1] is the data JSON object
          //         std::cerr << "Faoieuieui" << std::endl;
          //      std::cout << "eui to port " << std::endl;
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          int current_lane = round((car_d - 2) / 4);
          d_error = 0.9*d_error + 0.1*((2 + 4 * lane)-car_d);
          //std::cout << car_d << std::endl;
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          //if(stay_lane_counter == 1) std::cout << " ################### now we can change lane again ##########################" << std::endl;
          if(stay_lane_counter > 0){
            stay_lane_counter--;
          }
          

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          int prev_size = previous_path_x.size();

          if (prev_size > 0) {
            car_s = end_path_s;
          }



          // Reset our map before each time
          for (auto &i : map) std::fill(i.begin(), i.end(), false);
          for (auto &i : map_s) std::fill(i.begin(), i.end(), ' ');
          for (auto &i : map_2) std::fill(i.begin(), i.end(), ' ');
          // loop through all other cars
          // std::cout << "b" << sensor_fusion.size() << std::endl;
          int cnt = 0;
          struct closest_car_ahead{
            int car_id=-1;
            double distance=9999;
            double speed;
          } closest_car_ahead;

          // The sensor_fusion have the following content
          // 0 car's unique ID,
          // 1 car's x position in map coordinates,
          // 2 car's y position in map coordinates,
          // 3 car's x velocity in m/s,
          // 4 car's y velocity in m/s,
          // 5 car's s position in frenet coordinates,
          // 6 car's d position in frenet coordinates.
          for (int i = 0; i < sensor_fusion.size(); i++) {
            // and if any if these cars is nearby, add them to our map.
            cnt++;
            double check_car_s = sensor_fusion[i][5];

            // what exactly is "in the future"
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double rel_speed = (car_speed - check_speed)*time_delta;
            // check_car_s += ((double)(50-prev_size)*.02*-rel_speed);
            //move car to our position, not end of
            check_car_s += ((double)prev_size *0.02* check_speed);
            // if within 90 meter in any direction
            double car_distance = check_car_s - car_s;
            if (abs(car_distance) < 120) {
              int car_id = sensor_fusion[i][0];
              double accelration;
              if (prev_car_state.find(car_id) == prev_car_state.end()) {
                accelration = 0;
              } else {
                double speed_r = 0.2;
                double accel_r = 0.2;
                double dist_r = 0.5;
                rel_speed = rel_speed * speed_r + prev_car_state[car_id].v * (1-speed_r);
                accelration = (rel_speed - prev_car_state[car_id].v);///time_delta;
                accelration = (accelration * accel_r + prev_car_state[car_id].a * (1-accel_r));
                car_distance = car_distance * dist_r + prev_car_state[car_id].dist * (1-dist_r);
              }
              prev_car_state[car_id].v = rel_speed;
              prev_car_state[car_id].a = accelration;
              prev_car_state[car_id].dist = car_distance;
              float check_car_d = sensor_fusion[i][6];

              // std::cout << "prui2 " << std::endl;
              map = car_2_map(map, car_distance, rel_speed, check_car_d, accelration, lane);
              //   std::cout << "prui3 " << std::endl;
              int other_car_lane = round((check_car_d - 2) / 4);
              if ((car_distance + 10 < 90 && car_distance + 10 >= 0) &&
                  (other_car_lane >= 0 && other_car_lane <= 2)) {
                // +10 so we also can see behind
                map_s[other_car_lane][car_distance + 10] = 'c';  // this point will have a colision if doing nothing.
              }
              if (other_car_lane == current_lane && car_distance < closest_car_ahead.distance ){
                //car is ahead of us. 
                closest_car_ahead.car_id=car_id;
                closest_car_ahead.distance=car_distance;
                closest_car_ahead.speed=check_speed;
              }
            }
          }
          // here the map is complete: now do the search:
         // std::cout << predicted_path.path_list[0].y!=lane)
          //if(predicted_path.path_list.size() > 2){
          //  predicted_path.path_list[0].x = current_lane;
         // }
          /*if(!predicted_path.to_goal || 
             !verify_path(predicted_path,map,map.size(),map[0].size()) ||
             predicted_path.path_list[0].x != current_lane){
            predicted_path = search_path(map, current_lane);
          }else{
            std::cout << "using old path ok" << std::endl;
          }*/
          predicted_path = search_path(map, current_lane, predicted_path);
          if(!predicted_path.to_goal){
            //same care is blocking us ahead
            closest_car_ahead.distance;
            ref_vel=closest_car_ahead.speed;
            if (ref_vel > max_speed) ref_vel = max_speed;
            if (ref_vel < 10) ref_vel = 10;
            //calcelate speed to get at distance instead:
            
            //double target_accel= (v2 − u2 ) / 2s
          }else{
            ref_vel=max_speed;
          }
         // if (ref_vel < target_speed-2.7) {
         //   ref_vel += 0.5;//244;
         // }else if(ref_vel > target_speed){
         //   ref_vel -= 0.5;//0.224;
         // }
          
          
          
         // }
          // std::vector<pos>::const_iterator ste;
          // for(ste = predicted_path.path_list.begin(); ste !=
          // predicted_path.path_list.end(); ++ste){
          //  std::cout << *ste.x;
          // }
          
          //std::cout << "best path: " << predicted_path.path_list.size() / 2
           //         << ": ";
          for (int i = 0; i < predicted_path.path_list.size(); i++) {
           // std::cout << "(x:" << predicted_path.path_list[i].x
        //              << ",y:" << predicted_path.path_list[i].y << ")";
            map_2[predicted_path.path_list[i].x]
                 [predicted_path.path_list[i].y] = '0';
          }
          for (int xx = 0; xx < 3; xx++) {
            for (int yy = 0; yy < 30; yy++) {
              if (map[xx][yy]) map_2[xx][yy] = 'C';
            }
          }
         // std::cout << std::endl;

          // print the map!
          // Note to self: map is where x is time(second), and map_s is where x
          // is s(meter)
          std::cout << "map t:" << std::endl;
          std::vector<std::vector<bool>>::const_iterator row;
          std::vector<bool>::const_iterator col;
          std::vector<std::vector<char>>::const_iterator crow;
          std::vector<char>::const_iterator ccol;

          for (crow = map_2.begin(); crow != map_2.end(); ++crow) {
            for (ccol = crow->begin(); ccol != crow->end(); ++ccol) {
              std::cout << "|";
              std::cout << *ccol;
            }
            std::cout << "##" << std::endl;
          }
          std::cout << "## map_s:          #" << std::endl;
          for (crow = map_s.begin(); crow != map_s.end(); ++crow) {
            for (ccol = crow->begin(); ccol != crow->end(); ++ccol) {
              std::cout << "|";
              std::cout << *ccol;
            }
            std::cout << "##" << std::endl;
          }

          vector<double> ptsx;
          vector<double> ptsy;
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // std::cout << "prev size " << prev_size << std::endl;
          if (prev_size < 2) {  // when we start we dont have anypoints here
            // std::cout << "first"  << std::endl;

            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsy.push_back(prev_car_y);

            ptsx.push_back(car_x);
            ptsy.push_back(car_y);

          } else {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsy.push_back(ref_y_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y);
          }




          if (predicted_path.path_list.size() > 2 && 
              lane != predicted_path.path_list[1].x && 
              current_lane != predicted_path.path_list[1].x &&
              stay_lane_counter == 0){
            lane = predicted_path.path_list[1].x;
            stay_lane_counter = 70; //wait about 2 seconds


          }          
          vector<double> next_wp0 =
              getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 =
              getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 =
              getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          //}

          for (int i = 0; i < ptsx.size(); i++) {
            // shift car reference angle to local car coordinates.
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          tk::spline s;

          s.set_points(ptsx, ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double prev_pos_x = 0;
          double prev_jerk = 0;
          double prev_jerk_x = 0;
          double prev_accel_x = 0;
          double prev_accel = 0;
          double prev_speed_x=0;
          double prev_speed = 0;
          

          double t=0.02; // todo, check if the "real" time diff works too

          // start with all the prev path pons from last time:
          for (int i = 0; i < previous_path_x.size(); i++) {  // may be  prev_size => previous_path_x.size(); as in qa
            double b_x = previous_path_x[i];
            double b_y = previous_path_y[i];
            next_x_vals.push_back(b_x);
            next_y_vals.push_back(b_y);
            
            double shift_x = b_x - ref_x;
            double shift_y = b_y - ref_y;
            b_x = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
           // b_y = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

         //   #double this_speed = sqrt(pow(b_x-prev_pos_x,2)+pow(b_y-prev_pos_y,2))/t;
            double this_speed_x = (b_x-prev_pos_x)/t;
            //prev_speed_x = (b_x-prev_pos_x)/t;

            double this_accel_x = (this_speed_x-prev_speed_x);
            double this_jerk_x = (this_accel_x-prev_accel_x);
            prev_pos_x = b_x;
            prev_jerk_x = this_jerk_x;
            prev_accel_x = this_accel_x;
            prev_speed_x = this_speed_x;
           // std::cout << ":: from prev accel " << prev_accel_x/t;
          }

          double target_x = 30.0;
         // if(ptsx.size() > 3){
         //   target_x = ptsx[2];
         //   std::cout << "using pssx instead of 30.. : "<< target_x << std::endl;
         // }
          double target_y = s(target_x);
          double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

          double x_add_on = 0;
          double max_A_x=5;
          double max_J_x=3;
          double speed_metric=ref_vel / 2.24;
          for (int i = 1; i <= 50 - previous_path_x.size(); i++) {

            //calculate acceleration and jerk here and smooth if needed!
            double this_max_A_x = prev_accel_x + max_J_x*t;
            double this_min_A_x = prev_accel_x - max_J_x*t;
          //  std::cout << "Given the max jerk, the max acc is " << this_max_A_x << " and min:" << this_min_A_x << ":: from prev accel " << prev_accel_x;
            this_max_A_x = std::min(this_max_A_x, max_A_x*t);
            this_min_A_x = std::max(this_min_A_x,-max_A_x*t);
            //std::cout << ", however combined with max A, this is now" << this_max_A_x << " or min " << this_min_A_x << std::endl;
            double max_speed_x = (prev_speed_x + this_max_A_x); // devided by 2 because we have x and y.
            double min_speed_x = (prev_speed_x + this_min_A_x);
            //max_speed = std::min(max_speed,speed_metric);
            
           // std::cout << "This gives us an max speed of " << max_speed_x << " and min " << min_speed_x;
            double speed_x = std::max(std::min(max_speed_x,speed_metric),min_speed_x);
           // std::cout << "which makes us drive at " << speed_x << " refV: " << speed_metric << std::endl;
            
            double N = (target_dist / (t * speed_x));
            double x_point = x_add_on + (target_x) / N;
            double v_x = (x_point - prev_pos_x)/t; // in m/s
            prev_accel_x = (v_x - prev_speed_x)*t;
            prev_speed_x = v_x;
            prev_pos_x = x_point;



            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // Rotate back to global coordinates
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;  // why not above?
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          } 
       //   std::cout << std::endl;



          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  });  // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}