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

  //Define some parameters for path finding
  int target_lane = 1;    // 0 - left, 1 - middle, 2 - right
  double target_velocity = 49.5; //mph
  double max_long_acc = 0.25; // mph per 20ms

  // Need to pass all parameters into lambda
  h.onMessage([&target_lane, &target_velocity, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &max_long_acc]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {

    // Some constants...
    double timestep = 0.02;     //sim timestep
    double mph_to_ms = 0.44704; // convert mph to ms^-1
  
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

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          // Previous path - provided by the simulator.
          // Previous path is the remaining path from the previous iteration that has not been reached
          int prev_size = previous_path_x.size();

          // SENSOR FUSION 
          if (prev_size > 0)
          {
            car_s = end_path_s;
          }

          bool too_close = false;
          double car_infront_speed = 0;


          //Find ref_v to use
          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            // find cars in my lane
            float d = sensor_fusion[i][6];
            if (d < (2+4*target_lane+2) && d > (2+4*target_lane-2))
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];

              // if using prev points project s out
              check_car_s+=((double)prev_size*timestep*check_speed);

              // Check s values are greater than mine and s gap
              if ((check_car_s > car_s) && ((check_car_s-car_s) < 30))
              {
                // too close
                too_close = true;
                car_infront_speed = check_speed;

                // Get ID of car in front and use it's speed to set reference below...

                // pULL OUT TO difference lane // blindly.
                if (target_lane > 0)
                {
                  target_lane = 0;
                }

              }
            }
          }

          // Slow down if too close
          if (too_close && target_velocity > (car_infront_speed/mph_to_ms - 1.0*mph_to_ms))
          {
            target_velocity -= 1.0*mph_to_ms;
          }
          else if (target_velocity < 49.5)
          {
            target_velocity += 1.0*mph_to_ms;
          }

          //Set up car reference location and orientation
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          double ref_speed = car_speed;
                    

          // Sparsely spaced waypoints
          vector<double> ptsx;
          vector<double> ptsy;

          // START BY CALCULATING THE INITIAL DIRECTION OF PATH AND SPEED

          // If previous size is empty, use car to reference
          if (prev_size < 2)
          {
            // Use two points to make the initial path tangient to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else
          {
            // Make the path start at the end of the previous path
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            ref_speed = sqrt((ref_x-ref_x_prev)*(ref_x-ref_x_prev)+(ref_y-ref_y_prev)*(ref_y-ref_y_prev));
            ref_speed = ref_speed / timestep / mph_to_ms; // in mph

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // Push the next three points at steps based on car speed
          double dist_step = target_velocity * 3/5;

          // Now push three points at dist_step spacing in front of the car using Frenet coordinates
          vector<double> next_wp0 = getXY(car_s+dist_step, (2+4*target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+dist_step*2, (2+4*target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+dist_step*3, (2+4*target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          // Add to the way point list
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // Convert the path points into Car Coordinates

          // Simple Shift and rotation
          for (int i = 0; i< ptsx.size(); i++)
          {
            
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
          }

          // Create a spline
          tk::spline s;

          s.set_points(ptsx, ptsy);

          // Create the points for the simulator
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start wth all of the previous points
          for(int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Break up the spline so we travel at reference velocity
          // As we have rotated to car frame - that is 30m along x-axis
          // Assuming it is close enough to linear for calculation
          double target_x = dist_step;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

          double x_add_on = 0;

          // fill in the rest of the path
          
          // Max acceleration in 0.02seconds is c. 0.1mph 
          
          // Needs to be car speed at end of previous path
          double speed_step = ref_speed;


          for (int i = 1; i <= 50-previous_path_x.size(); i++)
          {
            speed_step = speed_step + max_long_acc; 
            if (speed_step > target_velocity) speed_step = target_velocity;

            double N = (target_dist/(timestep*speed_step*mph_to_ms));
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // translate back to world coordinates
            x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            // Add to the final path
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          /*Example from Getting Started
           double dist_inc = 0.4;
           for (int i = 0; i < 50; ++i) {
             //Stay in constant lane
             double next_s = car_s+(i+1)*dist_inc;
             double next_d = 6;

             //Convert from Frenet to XY
             vector<double> XY = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

             //next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
             //next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
             next_x_vals.push_back(XY[0]);
             next_y_vals.push_back(XY[1]);
           }
          */

          // END of path creation
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
