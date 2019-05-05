#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <set>
#include <string>
#include <vector>
#include <unistd.h> // remove
// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
int resultion_factor = 7;

string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//
struct pos {
  int x;
  int y;
};
struct path {
  vector<pos> path_list;
  bool to_goal = false;
};
struct best_path_item {
  path best_path;
  // vector<pos> path_list;
  double f;
};
struct open_item {
  double f;
  double g;
  pos p;
  bool operator<(const open_item &other) const { return f < other.f; }
  bool operator>(const open_item &other) const { return f > other.f; }
  bool operator==(const open_item &other) {
    return f==other.f && g==other.g && p.x == other.p.x && p.y == other.p.y;
  }
};


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y) {
  double closestLen = 100000;  // large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = std::min(2 * pi() - angle, angle);

  if (angle > pi() / 2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  // see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

bool check_xy_blocker(int x, int y, int xmax, int ymax,
                      vector<vector<bool>> map) {
  // Return true if its an ok path
  return !(x < 0 || x >= xmax || y < 0 || y >= ymax || map[x][y]);
}


bool verify_path(path test_path, vector<vector<bool>> map,int max_x,int max_y){
  //Test if the path between each pos is reacable.
  bool this_move_ok = true;
  if (test_path.path_list.size() < 2){
    return false;
  } 
  if (test_path.path_list.size() > 2 && test_path.path_list.back().y < max_y-2){
    // if we have more than 2, i.e not from algo. and:
    // 1. if not reach goal
    // or 2. wrong lane

    return false;
  }
  for(int i=0;i < test_path.path_list.size()-1;i++){
    int x1 = test_path.path_list[i].x;
    int xdiff = test_path.path_list[i+1].x-x1;
    int y1 = test_path.path_list[i].y;
    int ydiff = abs(test_path.path_list[i+1].y-y1);
   // std::cout << "ja " << x1 << " | " << xdiff << "|" << y1 << "|" << ydiff << std::endl;
    for (int check_y = y1; check_y <= y1+ydiff; check_y++) {
      // ned to redo:
      if (xdiff == 0) {
        this_move_ok = check_xy_blocker(x1, check_y, max_x, max_y, map);
     //   std::cout << "A " << this_move_ok << " x " << x1 << "|" << check_y << "|" << max_x << "|" << max_y << std::endl;
      } else if (xdiff == -1) {
        this_move_ok = check_xy_blocker(x1, check_y, max_x, max_y, map) &&
                        check_xy_blocker(x1 - 1, check_y, max_x, max_y, map);
       // std::cout << "B " << this_move_ok << " x(-1) " << x1 << "|" << check_y << "|" << max_x << "|" << max_y << std::endl;
      } else if (xdiff == 1) {
        this_move_ok = check_xy_blocker(x1, check_y, max_x, max_y, map) &&
                        check_xy_blocker(x1 + 1, check_y, max_x, max_y, map);
        //std::cout << "C " << this_move_ok << " x(+1) " << x1 << "|" << check_y << "|" << max_x << "|" << max_y << std::endl;
      }
      if (!this_move_ok) return false;
    }
  }
  return true;
}

bool check_goal(pos position,int max_y){
  if(position.y > max_y/2){
    return true;
  }else{
    return false;
  }
}
//The two following functions should replace the one above
bool check_step(pos p1, pos p2,vector<vector<bool>> map){
    int x1 = p1.x;
    int xdiff = p2.x-x1;
    int y1 = p1.y;
    int ydiff = p2.y-y1;
    int max_x = map.size();
    int max_y = map[0].size();
   // std::cout << "ja " << x1 << " | " << xdiff << "|" << y1 << "|" << ydiff << std::endl;
    bool this_move_ok = true;
    for (int check_y = y1; check_y <= y1+ydiff; check_y++) {
      // ned to redo:
      if (xdiff == 0) {
        this_move_ok = check_xy_blocker(x1, check_y, max_x, max_y, map);
     //   std::cout << "A " << this_move_ok << " x " << x1 << "|" << check_y << "|" << max_x << "|" << max_y << std::endl;
      } else if (xdiff == -1) {
        this_move_ok = check_xy_blocker(x1, check_y, max_x, max_y, map) &&
                        check_xy_blocker(x1 - 1, check_y, max_x, max_y, map);
       // std::cout << "B " << this_move_ok << " x(-1) " << x1 << "|" << check_y << "|" << max_x << "|" << max_y << std::endl;
      } else if (xdiff == 1) {
        this_move_ok = check_xy_blocker(x1, check_y, max_x, max_y, map) &&
                        check_xy_blocker(x1 + 1, check_y, max_x, max_y, map);
        //std::cout << "C " << this_move_ok << " x(+1) " << x1 << "|" << check_y << "|" << max_x << "|" << max_y << std::endl;
      }
      if (!this_move_ok) return false;
    }
    return this_move_ok;

  // This function takes one step of a path and check if the move is valid, returns true or false
}
path verify_long_path(path test_path, vector<vector<bool>> map,int max_x,int max_y, int lane){
  //this path takes a long path. and returns the same path as far it was possible to go.
  // if not ok, it will also set to_goal as false
  path new_path;
  if(test_path.path_list.size() == 0 || test_path.path_list[0].x != lane){//if empty, return just the start position.
    new_path.path_list.insert(new_path.path_list.end(), pos {lane,0}); 
    return new_path;
  }

  pos prev_pos=test_path.path_list[0];
  new_path.to_goal=false;
  for(int i=1;i < test_path.path_list.size();i++){
    pos p1 = test_path.path_list[i];
    if(check_step(p1,prev_pos,map)){
      new_path.path_list.insert(new_path.path_list.end(), prev_pos); 
      // always stay one behind, this way if we dont get all the way, we start one behind each time!
      // and start search from that, e.g, if we'le 5 steps away, return 4 and search from there, if still cant
      // find the next time we go back 3 steps etc.. downside with this is that we may change lane and then back for "no reason"                                                       
      prev_pos=p1;
      if(check_goal(p1,max_y)){
        new_path.path_list.insert(new_path.path_list.end(), p1);
        new_path.to_goal=true;
        break;
      }
    }else{
      //if not found, break and return whatever we have so far
      break;
    }
  }
  return new_path;
}




path search_path(vector<vector<bool>> map, int lane, path prev_path) {
  /*
   *  This function will calculate the path, based on astar but modified.
   */
  // What do we need. heuristic map,(NO, it can be calulated on fly.)
  // best_score_map (yes) vector of vector
  // best_path_map (yes) (maybe these two can be in the same?)
  // open_set (yes) a queue of starts to try
  //


  int max_x = map.size();
  int max_y = map[0].size();

  // first check if old path is ok, if not all the way, return how long we got but set to_goal to false.
  // move this to the verify path?
  //should we try backwards? i.e try from last ok point as start, and step back until ok?
  //or add heuristic where old path is much lower than other (but still less than straigth)
  path init_path = verify_long_path(prev_path, map,max_x,max_y,lane);
  if(init_path.to_goal){
    //if already go to goal, no need to check again.
   // return init_path;
  }
  //todo, use this

  vector<vector<best_path_item>> best_path_map(max_x, std::vector<best_path_item>(max_y));  // need to init this
  std::set<open_item> open_set;
  vector<open_item> open_backtrack;
  vector<open_item> open_backtrack2;

  int y = 0;
  int x = lane;
  double g = 1;
  double f = 999;  // no need to calculate

  int directions [] = {0,-1, 1};
  int min_len = 3;
  int max_len = 5;

  open_item start_position = {f, g, pos {x, y}};
  open_set.insert(start_position);
  pos start_pos = {x, y};
  best_path_map[x][y].best_path.path_list.insert(best_path_map[x][y].best_path.path_list.begin(), start_pos);
  best_path_map[x][y].f = f;
  path longest_path;
  double longest_path_len=9999;
  int count = 0;
  vector<vector<float>> map_s(3, std::vector<float>(30));
  while (!open_set.empty()) {
    count++;
    auto current_item = *open_set.begin();
    open_backtrack.insert(open_backtrack.end(), current_item);
    open_set.erase(open_set.begin());
    x = current_item.p.x;
    y = current_item.p.y;
    g = current_item.g;
    f = current_item.f;
    path current_path = best_path_map[x][y].best_path;
    for(int &direction : directions){
    //for (int direction = -1; direction <= 1; direction++) {
      bool this_move_ok = true;
      for (int len = min_len; len <= max_len; len++) {
        // now we loop over all the x and y to see we can go here or if its
        // blocked
        path test_path;
        //pos p = {x,y};
        test_path.path_list= {pos {x,y},pos {x+direction,y+len}};
       
       // std::cout << "check path from {" << x << "," << y << "} to {" << x+direction << "," << y+len <<"}" << std::endl;
        bool tmo = verify_path(test_path, map,max_x,max_y);

        for (int check_y = y; check_y <= y + len; check_y++) {
          // ned to redo:
          if (direction == 0) {
            this_move_ok = check_xy_blocker(x, check_y, max_x, max_y, map);
           // std::cout << "a " << this_move_ok << " x " << x << "|" << check_y << "|" << max_x << "|" << max_y << std::endl;
          } else if (direction == -1) {
            this_move_ok = check_xy_blocker(x, check_y, max_x, max_y, map) &&
                           check_xy_blocker(x - 1, check_y, max_x, max_y, map);
           // std::cout << "b " << this_move_ok << " x(-1) " << x << "|" << check_y << "|" << max_x << "|" << max_y << std::endl;

          } else if (direction == 1) {
            this_move_ok = check_xy_blocker(x, check_y, max_x, max_y, map) &&
                           check_xy_blocker(x + 1, check_y, max_x, max_y, map);
           // std::cout << "c " << this_move_ok << " x(+1) " << x << "|" << check_y << "|" << max_x << "|" << max_y << std::endl;
          }
          if (!this_move_ok){
            break;
          }
        }
       // std::cout << "##################dd";
        if(this_move_ok) {
          int landing_y = y + len;
          int landing_x = x + direction;
          //  std::cout << "move ok " << x << "," <<  y << " -> " << landing_x
          //  << "," << landing_y << std::endl;
          double new_g = g + len;
          double new_f = new_g + (max_y - landing_y) * 4 +
                         abs(direction) * (5 + y*5) * (max_len - len + 1);
          // std::cout << "new is " << new_f << " and old " <<
          // best_path_map[landing_x][landing_y].f << std::endl;
          if (new_f < best_path_map[landing_x][landing_y].f ||
              best_path_map[landing_x][landing_y].f == 0) {
            //   std::cout << "old" << best_path_map[x][y].f << std::endl;
            best_path_map[landing_x][landing_y].f = new_f;

            pos this_pos = {landing_x, landing_y};
            path this_path = current_path;
            // maybe I need to check if current path list exists?
            this_path.path_list.insert(this_path.path_list.end(), this_pos);
            best_path_map[landing_x][landing_y].best_path = this_path;
            if(new_f < longest_path_len){
              //std::cout << "new best path with score " << new_f << "at" << landing_x<< << << std::endl;
              longest_path_len = new_f;//landing_y
              longest_path = this_path;
            }
            if (check_goal(this_pos,max_y)){
              this_path.to_goal = true;
              return this_path;
            }
            open_item this_open = {new_f, new_g, pos {landing_x, landing_y}};
            // std::cout << "added new";
            open_set.insert(this_open);
            open_backtrack2.insert(open_backtrack2.end(), this_open);
          }
        } else {
          break;
        }
      }
    }
  }
  std::cout << "failed to find any path" << std::endl;
  /*
  for(auto openit : open_backtrack){
    std::cout << "(" << openit.x << "," << openit.y << ") f:" <<  openit.f << std::endl;
  }
  std::cout << "all open" << std::endl;
  for(auto openit : open_backtrack2){
    std::cout << "(" << openit.x << "," << openit.y << ") f:" <<  openit.f << std::endl;
  }
  std::cout << "make map";
  for (auto &i : map_s)
    std::fill(i.begin(), i.end(), 0);
  for(int xx=0;xx<3;xx++){
    for(int yy=0;yy<30;yy++){
      map_s[xx][yy] = best_path_map[xx][yy].f;
    }
  }
  //map_s[x][y] = -1;
  //map_s[landing_x][landing_y] = 9999;
  std::vector< std::vector<float> >::const_iterator crow;
  std::vector<float>::const_iterator ccol;
  std::cout << "## map_s:          #" << std::endl;
  for (crow = map_s.begin(); crow != map_s.end(); ++crow){
      for (ccol = crow->begin(); ccol != crow->end(); ++ccol){
          std::cout << "|";
          std::cout << *ccol;
      }
      std::cout << "##" << std::endl;
  } */
  return longest_path;
}

vector<vector<bool>> car_2_map(vector<vector<bool>> map, double car_distance,
                               double rel_speed, float check_car_d,
                               double accelration, int self_lane) {
  /*
   *This function will return a maps that
   * will be used in a-star. it takes map as input
   * input:
   * example
   * t0: 0 0 0
   * t1: 0 0 0
   * t2: 0 0 0
   * t3: 0 0 1
   * t4: 0 1 0
   *     g g g
   * a car that may be on the way of changing lane, should be seen as
   * a car taking up 2 lanes
   */
  // rel_speed: if negative, he drive faster than us, positve, were going to
  // catch up with him

  // if the other car is going away from us in both direction
  // std::cout << "dist: " << car_distance << ", speed:"  << rel_speed << ", d:
  // " << check_car_d
  //          << "accel: " << accelration << std::endl;

  //Todo. 1. if car from behind us in same lane, dont care for now(maybe inrease speed?)
  //      2. dont allow too close, rather make us turn quick in these case    
  if ((rel_speed < 0.001 && car_distance > 0) ||
      (rel_speed > -0.001 && car_distance < 0))
    return map;
  double ts;
  double ta;
  int map_len = map[0].size();
  int lane = floor((check_car_d) / 4);
  double lane_frac = fmod(check_car_d, 4)/4.0;
  int lane2 = lane;
  //std::cout << "d: " << check_car_d << " lane: " << lane << " lane_frac: " << lane_frac << std::endl;
  if(lane_frac >= 0.8) lane2++;
  if(lane_frac <= 0.2) lane2--;
 /* if (abs(car_distance) < 7 && (self_lane == lane || self_lane == lane2)) {
    // car is in same lane but close // basically we need to turn quick( or slow down)
    ts = 7; 
    ta = 3;
  }else if (abs(car_distance) < 5) {  // abs(rel_speed) < 2 &&
    ts = 5;                     // if he's next to us
    ta = 3;
  } else */
  if(car_distance <= 0 && self_lane == lane ){
    // if behind in same lane, dont care
    return map;
  //} else  if () {  // rel_speed < 0 && car_distance <= 0)
    // we drive slower, and hes behind of is
  } else if(false){//(abs(car_distance <= 3) && self_lane != lane ){
    // if the other car is very near but not same lane
    for (int t =0; t <= 3; t++) {
      if (t < 90 && t >= 0) {
        // add longer intervall ()
        map[lane][t] = true;
      }
    }
  
  } else {
    
    ts = round((car_distance / rel_speed) * resultion_factor); 
    
    if(false){//ts <= 4 && ts >= 0){ // too close, probably we still have time to react!
      ts = 4;
      ta = 4;
    }else{
      // only V, perhaps add a to this!
      // for the one with a we need to solve the equation 0 = 1/2 A t^2 s + V t
      // -dist
      //accelration = 0;
      if (abs(accelration) > 0.01) {
        float x1 = -1;
        float x2 = -1;
        float discriminant = rel_speed * rel_speed - 2 * accelration * -car_distance;
        if (discriminant > 0) {  //
          x1 = (-rel_speed + sqrt(discriminant)) / (accelration);
          x2 = (-rel_speed - sqrt(discriminant)) / (accelration);
          //   std::cout << "grr (" << x1 << "|" <<x2<< ")" << std::endl;
        } else {
          //      std::cout << "oh noo" << discriminant << std::endl;
          x1 = ts;
        }
        if (x1 > 0 && x2 > 0)
          ta = std::min(x1, x2) * resultion_factor;
        else if (x1 > 0)
          ta = x1 * resultion_factor;
        else if (x2 > 0)
          ta = x2 * resultion_factor;
        else
          ta = ts;

        ta = std::round(ta);
      } else {
        //   std::cout << "acc < min)" << std::endl;
        ta = ts;
      }
    }
  }
  //int t = ts;
  //if (t < 29 && t >= 0) {
  //    // add longer intervall ()
  //  map[lane][t] = true;
  //  map[lane][t+1] = true;
  //}
  if(car_distance < 0){
      ta = -ta;
      ts = -ts;
  }
  //std::cout << "yo (" << ts << "|" << ta << ") : " <<  (check_car_d-2)/4 << " => " << lane << std::endl;
  //ta = ts;
  if(ta < 0) ta = 0;
  if(ta > map_len) ta = map_len;
  if(ts < 0) ts = 0;
  if(ts > map_len) ts = map_len;
  int size=(map_len-((abs(ta)+abs(ts))/2))/10;
  for (int t = std::min(int(ts), int(ta)) - size; t <= std::max(int(ts), int(ta)) + size; t++) {
    if (t < map_len && t >= 0) {
      // add longer intervall ()
      if(!(lane == self_lane && abs(t) < 3))
        map[lane][t] = true;  // this point will have a colision if doing nothing.
      if(lane2 !=lane && lane2 >= 0 && lane2 <= 2)
        if(!(lane == self_lane && abs(t) < 3))
          map[lane2][t] = true;  // this point will have a colision if doing nothing.
    }
  }
  return map;
}
int predict_lane_change() {
  // Go to the classroom, and train the one we have there, then use it here and
  // predict if a car may change lane. 0=stay, -1= change left, 1= change right,
  // then, if this, the car takes up both lanes!
  return 0;
}
vector<double> getXYtime(int t, double d, double car_s, float a, float v,
                         const vector<double> &maps_s,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y) {
  // now just go back and solve S = 1/2 A t^2 + V t
  //                           s =(1/2)a t^2 + v t

  double s = car_s + (v * t) / resultion_factor;
  //double s = car_s + ((1 / 2) * a * t * t + v * t) / resultion_factor;
  // std::cout << " # t: " << t << ",v:" <<  v << ",s " << s << " # ";
  return getXY(s, d, maps_s, maps_x, maps_y);
}
#endif  // HELPERS_H