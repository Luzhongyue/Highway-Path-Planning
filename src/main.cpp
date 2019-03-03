#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "path_planner.h"
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

  
  // initial lane
  int lane = 1;

  // reference velocity to target vehicle
  double ref_v = 0.0; // mph
  
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&ref_v,&lane]
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

          // vectors to recieve data of each lane
          vector<double> left_lane_s;
          vector<double> left_lane_speed;

          vector<double> center_lane_s;
          vector<double> center_lane_speed;

          vector<double> right_lane_s;
          vector<double> right_lane_speed;
      
      // previous path size
      int prev_size = previous_path_x.size();
     
      // sensor fusion
      if (prev_size > 0) {
        car_s = end_path_s;
      }

      // boolean flags
      bool too_close = false; // if a car ahead is too close 
      bool way_too_close = false; // if need emergency stop
      bool safe_left = true; // if is safe to change to left 
      bool safe_right = true; // if is safe to change to right 
      bool lane_change = false; // if change to target lane

      
      

      // a loop to go through the sensor fusion data
      for (int i = 0; i<sensor_fusion.size(); ++i) {
        // d for each car
        float d = sensor_fusion[i][6];
        float target_s = sensor_fusion[i][5];
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double target_speed = sqrt(vx*vx + vy*vy);

        

        // check the cars in the left lane
        int lane_left = lane -1;
        if (d < (2 + 4*lane_left + 2) && d > (2 + 4*lane_left - 2)) {
          
          // check if the cars is in a danger area front
          if ((target_s > car_s) && ((target_s - car_s) < 11)) {
            safe_left = false;
          }

          // check if the car is in a danger area back
          if ((target_s < car_s) && (abs(target_s - car_s) < 30)) {
            safe_left = false;
          }
          // check if the car in the left lane behind is going faster than my car
          if ((target_s < car_s) && (abs(target_s - car_s) < 35) && (target_speed > car_speed)) {
            safe_left = false;
          }       
        }

        // check the cars in the right lane
        int lane_right = lane +1;
        if (d < (2 + 4*lane_right + 2) && d > (2 + 4*lane_right - 2)) {
          
          // check if the cars is in a danger area front
          if ((target_s > car_s) && ((target_s - car_s) < 11)) {
            safe_right = false;
          }

          // check if the car is in a danger area back
          if ((target_s < car_s) && (abs(target_s - car_s) < 30)) {
            safe_right = false;
          }
          // check if the car in the left lane behind is going faster than my car
          if ((target_s < car_s) && (abs(target_s - car_s) < 35) && (target_speed > car_speed)) {
            safe_right = false;
          }       

        }

        // check the car in the same lane
        if (d < (2 + 4*lane + 2) && d > (2 + 4*lane - 2)){
          // check if the car is too close
          if (target_s > car_s && (target_s - car_s) < 30) {
            too_close = true;
          }

          // check if the car is need emergency stop
          if (target_s > car_s && (target_s - car_s) < 15) {
            way_too_close = true;
          }
        }

        // collect the cars in each lane for calculate cost
        // it grabs all cars in front of my car
        // check the left lane
        if (d < 4 && d >= 0) {
          if (target_s > car_s) {
            left_lane_s.push_back(target_s - car_s);
            left_lane_speed.push_back(target_speed);
          }
        }

        // check the center lane
        if (d < 8 && d >= 4) {
          if (target_s > car_s) {
            center_lane_s.push_back(target_s - car_s);
            center_lane_speed.push_back(target_speed);
          }
        }

        // check the right lane
        if (d < 12 && d >= 8) {
          if (target_s > car_s) {
            right_lane_s.push_back(target_s - car_s);
            right_lane_speed.push_back(target_speed);
          }
        }
        
      } // end sensor fusion loop

   

      // feed data to cost function and return the best lane (0,1,2)
      int best_lane = Path_planning(lane,left_lane_s,left_lane_speed,center_lane_s,center_lane_speed,
                                   right_lane_s,right_lane_speed);
      // check if my car in the lane we have requested to avoid being outside for the lane for long periods of time
      // if my car in the lane we requested then set the flag to true, which will make the car to change lane
      if (car_d < (2 + 4*lane + 1.5) && car_d > (2 + 4*lane - 1.5)) {
        lane_change = true;
      }

      // emergency stop
      if (way_too_close) {
        ref_v -= 0.400;
      }

      // slow down
      if (too_close) {
        ref_v -= 0.224;  
      }

      // speed up
      else if (ref_v < 45) {
        ref_v +=0.224;
      }
      
      // check whether my car in the best lane, or it will change lane
      if (best_lane != lane) {
        // if it is saft and the speed is enough high, change lane

        // change right lane
        if (best_lane > lane && safe_right == true && car_speed > 40) {
          lane += 1;
        }

        // change left lane
        if (best_lane < lane && safe_left == true && car_speed > 40) {
          lane -=1;
        }
      } 

      // vectors for spline anchor points
      vector<double>ptsx;
      vector<double>ptsy;

      // reference numbers for the car
      double ref_x = car_x;
      double ref_y = car_y;
      double ref_yaw = deg2rad(car_yaw);

      // if the last size is small, build it up
      if (prev_size < 2) {
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);
        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
      }

      // otherwise push the next set of points to the vector
      else{
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        // use two points that make the path tangent to the previous path's point
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);
        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
      }

      // in frent coordinate, add 30m spaced points ahead of the starting reference
      vector<double>next_p0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      vector<double>next_p1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      vector<double>next_p2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

      ptsx.push_back(next_p0[0]);
      ptsx.push_back(next_p1[0]);
      ptsx.push_back(next_p2[0]);

      ptsy.push_back(next_p0[1]);
      ptsy.push_back(next_p1[1]);
      ptsy.push_back(next_p2[1]);

      for (int i = 0; i < ptsx.size(); ++i) {
        
        // shift car reference angle to 0
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = shift_x*cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw);
        ptsy[i] = shift_x*sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw);

      }

      // create a spline
      tk::spline s;

      // set points to the spline
      s.set_points(ptsx, ptsy);

      // define the actual (x,y) points which will use for the planner
      vector<double>next_x_vals;
      vector<double>next_y_vals;

      // start with all of the previous path points from last time
      for (int i = 0; i < previous_path_x.size(); ++i) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);

      }

      // calculate how to break up spline points so that we travel at our desired reference velocity
      double target_x = 30.0;
      double target_y = s(target_x);
      double target_dist = sqrt(target_x*target_x + target_y*target_y);
      double x_add_on = 0;

      // fill up the rest of planner after filling with previous points, here will always output 50 points
      for (int i = 0; i <= 50 - previous_path_x.size(); ++i) {
        double N =  (target_dist / (0.02*ref_v / 2.24));
        double x_point = x_add_on + (target_x) / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // rotate back to normal after rotating it earlier
        x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
        y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
      }

    
          json msgJson;

          //vector<double> next_x_vals;
          //vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */


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