#include <uWS/uWS.h>
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
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  int lane = 1;
  double ref_vel = 0.0;
    
  h.onMessage([&lane,&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          
          int prev_size = previous_path_x.size();
          //std::cout <<"PREV-SIZE" <<" "<<prev_size<< std::endl;
          
          if (prev_size > 0) {
            car_s = end_path_s;
            car_d = end_path_d;
          }

          int psl = 50; // posted speed limit in mph
          int other_lane; // lane assignment of non-ego_car
          int safe_dist = 30; // safe distance for lane changes.
          int follow_dist = 40; // safe distance to follow cars 
          bool platooning_l = false; //true if there are other cars just ahead (< 2*safe_dist) in adjacent left lane
          bool platooning_r = false;//true if there are other cars just ahead (< 2*safe_dist) in adjacent right lane
          bool too_close = false; //true if car before ego-car (in the same lane) is < safe_dist ahead
          bool too_close_left = false; //true if cars (in adjacent left lane) are within unsafe zone [ego-car +/- safe_dist]
          bool too_close_right = false; //true if cars (in adjacent right lane) are within unsafe zone [ego-car +/- safe_dist]
          double acc = 0.316; //rate of change of velocity 
          double speed_limit = 49.5; //maximum attainable speed
          double sdf = 2.0;  //factor multiplies safe_dist; product used to determine beneficial lane changes
          double mph_ms = 2.23694; //mph to m/s conversion factor
                                     
          for (int i=0; i < sensor_fusion.size(); ++i) {
            float d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double other_car_speed = sqrt(vx*vx + vy*vy);
            double other_car_s = sensor_fusion[i][5];

            other_car_s += ((double)prev_size*0.02*other_car_speed);
                                             
            //Assigning lane values for other cars based on sensor fusion data
            if (d > 0 && d < 4) {
              other_lane = 0;
            } 
            else if(d > 4 && d < 8) {
              other_lane = 1;
            } 
            else if(d > 8 && d < 12) {
              other_lane = 2;
            }

            if ((other_lane - lane) == 0) { //other car in same lane
              //If the 's' value of car ahead of the ego car (in the same lane) is < 30m and
              //its speed is < speed limit then it is too close.
              too_close |= (other_car_s > car_s) && (other_car_s - car_s) < follow_dist && (other_car_speed < psl/mph_ms);
            } 
            else if ((other_lane - lane) == -1) { //other car in adjcent left lane

              // If the 's' value of cars in the adjacent lane falls within the range of
              // ego-car's 's' value +/- safe distance it is too close for a left lane change.
              too_close_left |= (car_s + safe_dist) > other_car_s && (car_s - safe_dist) < other_car_s;
              
              // If cars in adjacent left lane are ahead and within 2 * safe_dist from ego_car and speed < ego_car speed
              // then left lane change is not optimized
              platooning_l |= (other_car_speed < car_speed) && (other_car_s > car_s) && other_car_s < (car_s + sdf*safe_dist);
            }
            else if ((other_lane - lane) == 1) { //other car in adjacent right lane

              // If the 's' value of cars in the adjacent lane falls within the range of
              // ego-car's 's' value +/- safe distance it is too close for a right lane change.
              too_close_right |= (car_s + safe_dist) > other_car_s && (car_s - safe_dist) < other_car_s;

              // If cars in adjacent right lane are ahead and within 2 * safe_dist from ego_car and speed < ego_car speed
              // then right lane change is not optimized
              platooning_r |= (other_car_speed < car_speed) && (other_car_s > car_s) && other_car_s < (car_s + sdf*safe_dist);
            }
          } 
                   
          if(too_close) { // If other car is < follow_dist (40m) ahead check one of the following conditions           
            if (!too_close_right && lane != 2 && !platooning_r) { // safe and efficient to do right lane change
              lane += 1;
              
            }  
            else if (!too_close_left && lane != 0 && !platooning_l) { // safe and efficient to do left lane change
              lane -= 1;
            }
            else {
              ref_vel -= acc; //If none of prior two conditions met - too close behind car hence reduce speed
            }
          }              

          else if(ref_vel < speed_limit) { //Increase speed until speed limit is attained
            ref_vel += acc; 
          }
          //std::cout <<"TC"<<" "<<too_close<<" " <<"TCL"<< " "<<too_close_left<< " " <<"TCR"<< " "<<too_close_right<<" "<<"PL"<< " "<<platooning_l<< " "<<"PR"<<" "<<platooning_r<< std::endl;  
          /* TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          vector<double> ptsx, ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = car_yaw;

          // If there are no points from previous path create a new point using 
          //the current (x,y) point of the car then add both points to the vector
          if (prev_size < 2) { 
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } 
          else {
            //if there are more than 2 points from the previous path add the last and penultimate points to the vector
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            //previous yaw is the angle subtended between the last two pair of points of the previous path
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev); 

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          } 
          
          //generate 3 additional points at further distances from current ego_car 's' value
          // and add to vector which now has 5 pairs of points (x,y) values
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          //Executes the translation with car at origin and rotation to obtain car heading of zero degree
          for (int i = 0; i < ptsx.size(); ++i) {
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x*cos(0-ref_yaw)- shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw)+ shift_y*cos(0-ref_yaw));

          }
          
          //using the previous and generated points the spline generates coefficients for a cubic polynomial
          // that can interpolate any point between the 5 points creating a smoth trajectory for the ego _car
          tk::spline sp;

          sp.set_points(ptsx, ptsy);
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //previous path values ate added to the next path vector first to create smooth motion transitions of ego_car
          for (int i = 0; i < prev_size; ++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          //geenrate a target distance from a future x-value to be used to calculate speed of ego_car
          double target_x = 30.0;
          double target_y = sp(target_x);
          double target_dist = distance(target_x, 0, 0, target_y);
          //std::cout <<"TARGET"<< " "<< target_dist << std::endl;

          double x_add_on = 0;

          //the target distance value is divided by how much the ego_car moves in 0.02 seconds (m/s x s = m)
          //to produce a factor N that divides the target x-value at intervals along the given x horizon
          // then spline is used to obtain the correxponding y-values. Genrally about 3 points 
          // are generated using the spline in this step, except when first intialized with no prior path points.

          for (int i = 0; i <= 50-prev_size; ++i) {
            double N = (target_dist/(0.02*ref_vel/mph_ms));
            double x_point = x_add_on+(target_x)/N;
            double y_point = sp(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;
            //Switching back to the map's frame of reference
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

                    
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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