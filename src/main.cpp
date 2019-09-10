#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen/Core"                  //"eigen3/Eigen/Core"
#include "Eigen/QR"             //"eigen3/Eigen/QR"
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

  int lane = 1; //lanes from left to right 0,1,2. We start off in middle lane.

  double ref_vel = 0.0; //Refference velocity.

  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane]
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
          // Previous path's end s and d values (Target location in s and d values)
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int previous_x_size = previous_path_x.size(); //How many values previous x has. Should be the same number of y values

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          /* //------------------Doing path using simple s and d values (Problem: too rigid, not good for the curves)--------------------------------
          double dist_inc = 0.4;
          double lane_width = 4.0; //Width of a lane from 
          for (int i = 0; i < 50; ++i) {
            //std::cout<<"Car yaw: "<<car_yaw<<std::endl;
            double next_s = car_s+(i+1)*dist_inc;
            double next_d = car_d; //Value of zero would be the double yellow line

            //std::cout<<"Car_S: "<<car_s<<std::endl;
            //std::cout<<"Car_D: "<<car_d<<std::endl;

            vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            //next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw))); //Forward x
            //next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw))); //Lateral y

            next_x_vals.push_back(xy[0]); //Forward s
            next_y_vals.push_back(xy[1]); //Lateral d


            //std::cout<<"Next X : "<<next_x_vals[i]<<std::endl;
            //std::cout<<"Next Y : "<<next_y_vals[i]<<std::endl;

            //std::cout<<"Car Speed: "<<car_speed<<std::endl;
          }
          */

          //METHOD: Utilizing spline library to create path generation

          //Utilizing previous path points
          if(previous_x_size > 0){ //If we have previous points
            car_s = end_path_s;    //Make the s the last point s
          }

          bool too_close = false;     //Ture if we have a vehicle ahead in our lane
          bool vehicle_left = false;  //True if we have a vehicle to our left
          bool vehicle_right = false; //True if we have a vehicle to our right

          //Check every vehicle fusion data
          for(int i = 0; i< sensor_fusion.size(); i++){
            float d = sensor_fusion[i][6]; //the i'ths car 6th attribute being d position
            
            double vx = sensor_fusion[i][3];          //Grab velocity x in m/s
            double vy = sensor_fusion[i][4];          //Grab velocity y in m/s
            double check_speed = sqrt(vx*vx+vy*vy);   //Speed
            double check_car_s = sensor_fusion[i][5]; //Grab s

            check_car_s+=((double)previous_x_size*0.02*check_speed); //project car speed in future
            
            if (d<(2+4*lane+2) && d > (2+4*lane-2)){ //If the object is within our lane boundaries
              if ((check_car_s > car_s) && ((check_car_s-car_s) < 30)){
                //ref_vel= 29.5; //mph
                too_close = true;
                std::cout<<"FRONT!"<<std::endl;
              }
            }
            else if (d<(2+4*lane-2) && d>(2+4*lane-6) && d>0){ //If the object is on left lane
              if ((check_car_s > car_s-30) && (check_car_s<car_s+30)){
                vehicle_left = true;
                std::cout<<"Car on LEFT!"<<std::endl;
              }
            }
            else if (d>(2+4*lane+2) && d<(2+4*lane+6) && d<12){ //If the object is on right lane
              if ((check_car_s > car_s-30) && (check_car_s<car_s+30)){
                vehicle_right = true;
                std::cout<<"Car on RIGHT!"<<std::endl;
              }
            }
          }

          if(too_close){ //If we have a vehicle ahead of in same lane
            ref_vel -= 0.224;
            if(!vehicle_left && lane>0){ //and there is no vehicles on the left and we are not in lane 0
              lane-=1;
            }
            else if(!vehicle_right && lane!=2){ //or there is no vehicles on the righ and we are not in lane 2
              lane+=1;
            }
          }
          else if(ref_vel < 49.5){
            ref_vel += 0.224;
          }

          //Doing path using spline library 

          //Init vector of points that will be used to create the spline
          vector<double> spline_points_x;
          vector<double> spline_points_y;

          //Grab the x, y, and yaw measurements as reference
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw); //IN RADIANS

          //If prev_size is small, use the car's position as reference. If it's not, use the last number in previous_x_size
          if(previous_x_size < 2){
            //Define two points tangent to the path of car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            //Add the previous tangent point and the current point to the points that will be used to create the spline
            spline_points_x.push_back(prev_car_x);
            spline_points_x.push_back(car_x);

            spline_points_y.push_back(prev_car_y);
            spline_points_y.push_back(car_y);
          }
          else{
            //Use the last points on previous path as reference point
            ref_x = previous_path_x[previous_x_size-1];
            ref_y = previous_path_y[previous_x_size-1];
            //Grab the second to last point as well
            double ref_x_prev = previous_path_x[previous_x_size-2];
            double ref_y_prev = previous_path_y[previous_x_size-2];
            //use arctan to get reference yaw from these points
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            //Add the previous tangent point and the current point to the points that will be used to create the spline
            spline_points_x.push_back(ref_x_prev);
            spline_points_x.push_back(ref_x);

            spline_points_y.push_back(ref_y_prev);
            spline_points_y.push_back(ref_y);
          }

          //Grab 3 more points in Frenet coordinates, each 30 meters appart (30, 60, 90)
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          //add them to the list of points to create the spline
          spline_points_x.push_back(next_wp0[0]);
          spline_points_x.push_back(next_wp1[0]);
          spline_points_x.push_back(next_wp2[0]);

          spline_points_y.push_back(next_wp0[1]);
          spline_points_y.push_back(next_wp1[1]);
          spline_points_y.push_back(next_wp2[1]);

          //Transform initial spline points from global to car's coordinates orientation
          for(int i=0; i<spline_points_x.size(); i++){
            double shift_x = spline_points_x[i] - ref_x;
            double shift_y = spline_points_y[i] - ref_y;

            spline_points_x[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            spline_points_y[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }

          //Init the spline
          tk::spline s;

          //Feed the x and y value of the points to the spline
          s.set_points(spline_points_x, spline_points_y);

          //Feed any previous points left over
          for(int i=0; i<previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);

          }

          //Calculate the points needed to go at desired speed
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x*target_x)+(target_y*target_y)); //Pythagorean theorem

          double x_add_on = 0;

          for(int j=0; j<=50-previous_path_x.size(); j++){ //Add as many values needed to get back to 50 total points
            double N = (target_dist/(0.02*ref_vel/2.24));  //Calculate how manny path points are needed to drive at the desired velocity (ref_vel) divide by 2.24 for m/s
            double x_point = x_add_on + (target_dist/N);  //Add on the x of each N to get the x point
            double y_point = s(x_point);                  //Correspodning spline value

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            //rotate back to global coordinates
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

            x_point+=ref_x;
            y_point+=ref_y;

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