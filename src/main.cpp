#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
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

// Euclidian distance between two points.
double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// The closest waypoint by Euclidian distance. This my not be the best waypoint to visit next
// because it could be behind you.
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

// The best waypoint to visit next, based on your orientation, even if it is not the closest.
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_dx, const vector<double> &maps_dy)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

	return {x,y};
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../../../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  // Our speed limit is 49mph(21.88m/s) just under the 50mph legal limit.
  double speed_limit = 49.0 / 2.24;
  double target_velocity = 0.0;
  // The lane the ego car occupies.
  double lane = 1;

  ifstream in_map_(map_file_.c_str(), ifstream::in);
  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&speed_limit,&target_velocity,&lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          // Each lane is 4m.
          double lane_width = 4.0;
          double lane_center = lane_width / 2.0;
          // The width of a lane to the left or right of the center.
          double lane_padding = lane_width / 2.0;

          // The simulator processes 1 point every 20ms(0.02 points/sec)
          double simulator_points_per_second = 20.0 / 1000.0;

          // The minimum comfortable gap(30m) ego car must have when following another car.
          double min_gap = 30.0;

          // When changing speed, decrease or increase by 0.5mph(0.22m/s)
          double speed_change_factor = 0.5 / 2.24;

        int prev_size = previous_path_x.size();

        if (prev_size > 0) {
          car_s = end_path_s;
        }

        // Keeps the state of whether an obstacle exists {to the left of ego lane, in ego lane, to the right of ego lane}
        vector<bool> has_obstacle = {false, false, false};

        for (int car_id = 0; car_id < sensor_fusion.size(); car_id++) {
          float d = sensor_fusion[car_id][6];
          int left_lane = lane;
          int right_lane = lane;

          // Check if a lane to the left of ego exists.
          if(lane > 0) {
            left_lane--;
          }

          // Check if a lane to the right of ego exists.
          if(lane < 2) {
            right_lane++;
          }

          double vx = sensor_fusion[car_id][3];
          double vy = sensor_fusion[car_id][4];
          double sensed_car_speed = sqrt(vx * vx + vy * vy);
          double sensed_car_s = sensor_fusion[car_id][5];

          // Calculate s value of sensed car if all unprocessed path points of ego are executed at its current speed.
          sensed_car_s += (double)prev_size * simulator_points_per_second * sensed_car_speed;

          // Check if sensed car is in ego lane.
          if (d > (lane_center + (lane_width * lane) - lane_padding) && d < (lane_center + (lane_width * lane) + lane_padding)) {
            // Check if sensed car is ahead of ego car but less than comfortable distance.
            if((sensed_car_s > car_s) && (sensed_car_s - car_s) < min_gap) {
              has_obstacle[1] = true;
            }
          }

          // If no lane exists to the left of ego, then there's an obstacle.
          if (lane == left_lane) {
            has_obstacle[0] = true;
          }
          else if (!has_obstacle[0] && d > (lane_center + (lane_width * left_lane) - lane_padding) && d < (lane_center + (lane_width * left_lane) + lane_padding)) {
            // Check if sensed car is left of ego but less than comfortable distance.
            if((sensed_car_s > car_s) && (sensed_car_s - car_s) < min_gap) {
              has_obstacle[0] = true;
            }
          }

          // If no lane exists to the right of ego, then there's an obstacle.
          if (lane == right_lane) {
            has_obstacle[2] = true;
          }
          else if (!has_obstacle[2] && d > (lane_center + (lane_width * right_lane) - lane_padding) && d < (lane_center + (lane_width * right_lane) + lane_padding)) {
            // Check if sensed car is right of ego but less than comfortable distance.
            if((sensed_car_s > car_s) && (sensed_car_s - car_s) < min_gap) {
              has_obstacle[2] = true;
            }
          }
        }

        // Adjust accordingly if there's an obstacle in ego lane.
        if (has_obstacle[1]) {
          // Lower the speed if we're too close.
          target_velocity -= speed_change_factor;

          // Try to move left if no obstacle exists in that lane.
          if (!has_obstacle[0]) {
            lane--;
          } else if (!has_obstacle[2]) {
            // If a left move can't be made, try to move right if possible.
            lane++;
          }
        } else if (target_velocity < speed_limit) {
          // Speed up if there are no obstacles and ego is below the speed limit.
          target_velocity += speed_change_factor;
        }

        vector<double> ptsx;
        vector<double> ptsy;

        double ref_x = car_x;
        double ref_y = car_y;
        double ref_yaw = deg2rad(car_yaw);

        // If path finder was recently instantiated and there's not enough previous path information,
        // construct add two points to the path based on the car's current position.
        if(prev_size < 2) {
          double prev_car_x = car_x - cos(car_yaw);
          double prev_car_y = car_y - sin(car_yaw);

          ptsx.push_back(prev_car_x);
          ptsx.push_back(car_x);

          ptsy.push_back(prev_car_y);
          ptsy.push_back(car_y);
        } else {
          // Add two points to the overall path by extrapolating two of the most recent, previous path points.
          // This extrapolation can be done by getting the tangent of of the two previous points.
          ref_x = previous_path_x[prev_size - 1];
          ref_y = previous_path_y[prev_size - 1];

          double ref_x_prev = previous_path_x[prev_size - 2];
          double ref_y_prev = previous_path_y[prev_size - 2];
          ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

          ptsx.push_back(ref_x_prev);
          ptsx.push_back(ref_x);

          ptsy.push_back(ref_y_prev);
          ptsy.push_back(ref_y);
        }

        // Generate synthetic waypoints with Frenet coordinates from a recently encountered waypoint.
        // Synthetic waypoints are 30m, 60m and 90m ahead. Adjust the waypoints to be in the center of the current lane
        // rather than on the road divider (double yellow lines).

        vector<double> next_wp0 = getXY(car_s + 30, (lane_center + (lane_width * lane)), map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);
        vector<double> next_wp1 = getXY(car_s + 60, (lane_center + (lane_width * lane)), map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);
        vector<double> next_wp2 = getXY(car_s + 90, (lane_center + (lane_width * lane)), map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);

        // Add the synthetic waypoints to the car path.
        ptsx.push_back(next_wp0[0]);
        ptsx.push_back(next_wp1[0]);
        ptsx.push_back(next_wp2[0]);

        ptsy.push_back(next_wp0[1]);
        ptsy.push_back(next_wp1[1]);
        ptsy.push_back(next_wp2[1]);

        // Shift coordinates of all path points from global coordinates to vehicle coordinates i.e. heading angle is 0 degrees.
        // This should make it easier to smoothen using Spline.
        for (int i = 0; i < ptsx.size(); i++) {
          double shift_x = ptsx[i] - ref_x;
          double shift_y = ptsy[i] - ref_y;

          ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
          ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
        }

        tk::spline spline_function;

        spline_function.set_points(ptsx, ptsy);

        vector<double> next_x_vals;
        vector<double> next_y_vals;

        // In order to enable a smooth transition from the previous set of points to the newly generated
        // points, we first append the previous path points that have no yet been visited.
        for (int i = 0; i < prev_size; i++) {
          next_x_vals.push_back(previous_path_x[i]);
          next_y_vals.push_back(previous_path_y[i]);
        }

        // We want our path points to extend up to 30m ahead.
        double target_x = 30.0;
        double target_y = spline_function(target_x);



        // The distance of the target point from the car.
        double target_distance = sqrt(target_x * target_x + target_y * target_y);

        // The number of path points to place between the car and the target distance.
        // A fewer number of points will raise the velocity while more points will lower the velocity.
        double number_of_path_points = target_distance / (simulator_points_per_second * target_velocity);

        // We want a maximum of 50 path points.
        int max_points = 50;
        double last_point_distance = 0;

        // Append new path points to list of previous points that have not been visited.
        for (int i = 0; i < max_points - prev_size; i++) {
          double x_point = last_point_distance + (target_x / number_of_path_points);
          double y_point = spline_function(x_point);
          last_point_distance = x_point;

          // Shift path points from vehicle coordinates back to global coordinates.
          double x_ref = x_point;
          double y_ref = y_point;

          x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
          y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

          x_point += ref_x;
          y_point += ref_y;

          next_x_vals.push_back(x_point);
          next_y_vals.push_back(y_point);
        }

          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
