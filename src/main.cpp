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

  // Start in lane 1.
  int lane = 1;

  double ref_vel = 0; // v in mph

  double follow_vel;


  enum fsm_states {
    keep_lane,
    check_actions,
    slow_down,
    lane_change_left,
    lane_change_right
  };

  fsm_states current_state = fsm_states::keep_lane;


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,
               &lane, &ref_vel, &follow_vel] // The last two variables were added to the capture list to be able to use them below.
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
          
          // Main ego's localization Data
          double ego_x = j[1]["x"];
          double ego_y = j[1]["y"];
          double ego_s = j[1]["s"];
          double ego_d = j[1]["d"];
          double ego_yaw = j[1]["yaw"];
          double ego_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          // 0: car's unique ID
          // 1: car's x position in map coordinates
          // 2: car's y position in map coordinates
          // 3: car's x velocity in m/s
          // 4: car's y velocity in m/s
          // 5: car's s position in frenet coordinates
          // 6: car's d position in frenet coordinates
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size(); // from udacity


          if (prev_size > 0) {
            ego_s = end_path_s;
          }

          //////////////////////
          // 1. Sensor Fusion
          //////////////////////

          bool too_close = false;

          bool left_blocked = false;
          bool right_blocked = false;

          // std::cout << " " << std::endl;

          // Iterate over all detected vehicles on the same side of the road.
          for (int i = 0; i < sensor_fusion.size(); ++i) {

            // Frenet d position of detected vehicle
            float d_pos = sensor_fusion[i][6];

              double v_x = sensor_fusion[i][3];
              double v_y = sensor_fusion[i][4];
              double v_mag = sqrt(v_x*v_x + v_y*v_y);
              double s_pos = sensor_fusion[i][5];

              // Predict s position of detected vehicle.
              s_pos += ((double)prev_size * .02*v_mag);

            // Check if sensed vehicle and ego vehicle are in the same lane.
            if (d_pos < (2+4*lane+2) && d_pos > (2+4*lane-2)) {

              // Check if detected vehicle is in front of ego vehicle and closer than 30m.
              if ((s_pos > ego_s) && (s_pos - ego_s) < 30) {
                // ref_vel = 29.5;
                too_close = true;
                follow_vel = v_mag;
              }
            }

            // Check if lane change left is possible.
            // Check if ego vehicle is in lane 1 or 2, and if detected vehicle is in left lane.
            if ((lane>0) && (d_pos < (2+4*(lane-1)+2)) && (d_pos > (2+4*(lane-1)-2))) {
              // Check if distance to detected vehicle is big enough for lane change.
              if (((s_pos > ego_s) && (s_pos - ego_s) < 40) || ((s_pos < ego_s) && (ego_s - s_pos) < 10)) {
                left_blocked = true;
                
              }
            }

            // Check if lane change right is possible: If there is a car in the right lane: is there enough space for a lane change?
            if ((lane<2) && (d_pos < (2+4*(lane+1)+2)) && (d_pos > (2+4*(lane+1)-2))) {
              if (((s_pos > ego_s) && (s_pos - ego_s) < 40) || ((s_pos < ego_s) && (ego_s - s_pos) < 10)) {
                right_blocked = true;
                
              }
            }
        
          } // I
          
          // if (left_blocked) {

          //   std::cout << "left blocked!" << std::endl;
          // }
          // if (right_blocked) {
          //   std::cout << "right blocked!" << std::endl;
          // }


          ///////////////////////////
          // 2. Behavior Planning
          ///////////////////////////

          // Determine acceleration.
          double acc = .5; // .224 (~5m/s²)

          // Deccelerate if car is too close to the car in the front.
          if (too_close) {

            // std::cout << "\ntoo close! " << std::endl;

            if ((left_blocked && right_blocked) || lane==0 && right_blocked || lane==2 && left_blocked ) {

              if (ref_vel > follow_vel) {
                // std::cout << "ref_vel > follow_vel " << std::endl;
                ref_vel -= acc;
              }
            }

            // current_state = fsm_states::check_actions;
            else {

              if (lane==1) {
                if (!left_blocked) {
                  lane--;
                }
                else if (!right_blocked) {
                  lane++;
                }

              } else if (lane==0) {
                if (!right_blocked) {
                  lane++;
                }
              } else if (lane==2) {
                if (!left_blocked) {
                  lane--;
                }
              }

            }

          }
          // Accelerate if no car is in front of ego vehicle within specified range.
          else if (ref_vel < 49.5){
            ref_vel += acc;
          }


          /////////////////////////
          // 3. Trajectory Planning
          /////////////////////////

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // Widely spaced x-y-points which will be interpolated with splines.
          vector<double> x_points;
          vector<double> y_points;

          double ref_x = ego_x;
          double ref_y = ego_y;
          double ref_yaw = deg2rad(ego_yaw);

          if (prev_size < 2) {
            // Make path tangent to the car.
            double prev_ego_x = ego_x - cos(ego_yaw);
            double prev_ego_y = ego_y - sin(ego_yaw);

            x_points.push_back(prev_ego_x);
            x_points.push_back(ego_x);

            y_points.push_back(prev_ego_y);
            y_points.push_back(ego_y);
          }
          // Previous paths end points are used as starting references.
          else {

            // Make the path tangent to the car by using the last two points from the previous path.
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            x_points.push_back(ref_x_prev);
            x_points.push_back(ref_x);
            
            y_points.push_back(ref_y_prev);
            y_points.push_back(ref_y);

          }

          // Create widely spaced points in Frenet coordinates and transform them into x-y-coordinates.
          vector<double> anchor_point_0 = getXY(ego_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> anchor_point_1 = getXY(ego_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> anchor_point_2 = getXY(ego_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          // Push the values to the x and y vectors.
          x_points.push_back(anchor_point_0[0]);
          x_points.push_back(anchor_point_1[0]);
          x_points.push_back(anchor_point_2[0]);
          
          y_points.push_back(anchor_point_0[1]);
          y_points.push_back(anchor_point_1[1]);
          y_points.push_back(anchor_point_2[1]);


          // Transform the values into the car's coordinate system.
          for (int i = 0; i < x_points.size(); ++i) {
            // Translation:
            double shifted_x = x_points[i] - ref_x;
            double shifted_y = y_points[i] - ref_y;

            // Rotation:
            x_points[i] = (shifted_x * cos(0-ref_yaw) - shifted_y * sin(0-ref_yaw));
            y_points[i] = (shifted_x * sin(0-ref_yaw) + shifted_y * cos(0-ref_yaw));

          }

          // Create a spline.
          tk::spline s;
          s.set_points(x_points, y_points);

          // Add points from the previous path to the new one.
          for (int i = 0; i < previous_path_x.size(); ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;


          for (int i = 1; i <= 50-previous_path_x.size(); ++i) {
            
            double N = (target_dist / (.02*ref_vel / 2.24));
            double x_pnt = x_add_on + (target_x) / N;
            double y_pnt = s(x_pnt);

            x_add_on = x_pnt;

            double x_ref = x_pnt;
            double y_ref = y_pnt;

            // Transform back to global coordinates.
            x_pnt = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_pnt = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_pnt += ref_x;
            y_pnt += ref_y;


            next_x_vals.push_back(x_pnt);
            next_y_vals.push_back(y_pnt);

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