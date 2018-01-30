#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <tuple>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "vehicle.h"
#include "spline.h"
#include "cost.h"

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
string hasData(string s)
{
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos)
	{
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos)
	{
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x, y, map_x, map_y);
		if (dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y - y), (map_x - x));

	double angle = fabs(theta - heading);
	angle = min(2 * pi() - angle, angle);

	if (angle > pi() / 4)
	{
		closestWaypoint++;
		if (closestWaypoint == maps_x.size())
		{
			closestWaypoint = 0;
		}
	}

	return closestWaypoint;
}

tuple<double, double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0)
	{
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

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if (centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return make_tuple(frenet_s, frenet_d);
}

// Transform from Frenet s,d coordinates to Cartesian x,y
tuple<double, double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
	{
		prev_wp++;
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

	return make_tuple(x, y);\
}

int dToLaneNumber(double d)
{
	return ((int) d) / 4; 
}

int laneNumberToD(int lane)
{
	return (2 + 4 * lane);
}

Vehicle createVehicle(vector<double> sensor_data)
{
	int id = sensor_data[0];
	double x_pos = sensor_data[1];
	double y_pos = sensor_data[2];
	double vx = sensor_data[3];
	double vy = sensor_data[4];
	double s = sensor_data[5];
	float d = sensor_data[6];
	double velocity = sqrt(vx * vx + vy * vy);
	int lane = 0;
	return Vehicle(id, dToLaneNumber(d), s, velocity, 0);
}

map<int, Vehicle> createVehicles(vector<vector<double>> sensor_fusion)
{
	map<int, Vehicle> surroundings;
	for (auto data : sensor_fusion)
	{
		surroundings[data[0]] = createVehicle(data);
	}
	return surroundings;
}
/**
 *  
 */
tuple<double, int> process_sensor_fusion(
	vector<vector<double>> & sensor_fusion,
	map<int, Vehicle> & surroundings,
	Vehicle & self,
	double car_s, 
	double car_v, 
	int car_lane, 
	int future_steps)
{
	vector<Vehicle> predictions;
	self.update(car_lane, car_s, car_v, future_steps);
	for (auto data : sensor_fusion) {
		Vehicle prediction;
		if (surroundings.find(data[0]) == surroundings.end()) {
			prediction = createVehicle(data);

		}
		else {
			prediction = surroundings[data[0]].generate_predictions(future_steps, data);
		}
		predictions.push_back(prediction);
		surroundings[data[0]] = prediction;
	}
	vector<Vehicle> trajectory = self.choose_next_state(predictions);
	double new_velocity = trajectory[1].velocity;
	double new_lane = trajectory[1].lane; 
	self.state = trajectory[1].state;
	return make_tuple(new_velocity, new_lane);
}

bool inSameLane(double other_d, double our_lane)
{
	return (other_d < (2 + 4 * our_lane + 2) && other_d > (2 + 4 * our_lane - 2));
}

float inefficiency_cost(int target_speed, int intended_lane, int final_lane, vector<int> lane_speeds)
{
	/*
    Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than target_speed.
    */

	float speed_intended = lane_speeds[intended_lane];
	float speed_final = lane_speeds[final_lane];
	float cost = (2.0 * target_speed - speed_intended - speed_final) / target_speed;
	return cost;
}

double roadVelocity(double vx, double vy)
{
	return sqrt(vx * vx + vy * vy);
}

const double max_acceleration = 0.7; // interval for each run is around 0.1s
const double POINT_INTERVAL = 30; // trajectory point interval

int main()
{
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	const string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	const double max_s = 6945.554;

	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line))
	{
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

	int lane = 1;
	double ref_vel = 0;
	map<int, Vehicle> surroundings;
	Vehicle self = Vehicle(-1, lane, 0, 0, 0, "KL");
	
	self.configure(49.5, 3, 7000, 1, max_acceleration);
	
	h.onMessage([
		&map_waypoints_x, 
		&map_waypoints_y, 
		&map_waypoints_s, 
		&map_waypoints_dx, 
		&map_waypoints_dy, 
		&lane, 
		&ref_vel,
		&surroundings,
		&self
	](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,  uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);
		//cout << sdata << endl;
		if (!(length && length > 2 && data[0] == '4' && data[1] == '2'))
			return;
		auto json_string = hasData(data);
		if (json_string == "")
		{
			// Manual driving
			std::string msg = "42[\"manual\",{}]";
			ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			return;
		}

		auto j = json::parse(json_string);

		string event = j[0].get<string>();
		if (event != "telemetry")
			return;
		// j[1] is the data JSON object

		// Main car's localization Data
		double car_x = j[1]["x"];
		double car_y = j[1]["y"];
		double car_s = j[1]["s"];
		double car_d = j[1]["d"];
		double car_yaw = j[1]["yaw"];
		double car_speed = j[1]["speed"];
		// Previous path data given to the Planner
		vector<double> previous_path_x = j[1]["previous_path_x"];
		vector<double> previous_path_y = j[1]["previous_path_y"];
		// Previous path's end s and d values
		double end_path_s = j[1]["end_path_s"];
		double end_path_d = j[1]["end_path_d"];

		// Sensor Fusion Data, a list of all other cars on the same side of the road.
		vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
		int prev_size = previous_path_x.size();

		if (prev_size > 0)
		{
			car_s = end_path_s;
		}

		bool too_close = false;

		if (surroundings.empty()) {
			surroundings = createVehicles(sensor_fusion);
		}

		auto result = process_sensor_fusion(
			sensor_fusion, 
			surroundings, 
			self, 
			car_s, ref_vel, lane, prev_size);
		
		auto new_speed = get<0>(result); 
		auto new_lane = get<1>(result);
		ref_vel = new_speed;
		lane = new_lane;
		vector<double> ptsx;
		vector<double> ptsy;

		double ref_x = car_x;
		double ref_y = car_y;
		double ref_yaw = deg2rad(car_yaw);

		// if previous size is almost empty, use the car as starting reference
		if (prev_size < 2)
		{
			double prev_car_x = car_x - cos(car_yaw);
			double prev_car_y = car_y - sin(car_yaw);

			double to_x[] = {prev_car_x, car_x};
			ptsx.insert(ptsx.end(), to_x, end(to_x));

			double to_y[] = {prev_car_y, car_y};
			ptsy.insert(ptsy.end(), to_y, end(to_y));
		}
		else // use the previous path's end point as starting reference
		{
			ref_x = previous_path_x[prev_size - 1];
			ref_y = previous_path_y[prev_size - 1];

			double ref_x_prev = previous_path_x[prev_size - 2];
			double ref_y_prev = previous_path_y[prev_size - 2];
			ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

			double to_x[] = {ref_x_prev, ref_x};
			ptsx.insert(ptsx.end(), to_x, end(to_x));

			double to_y[] = {ref_y_prev, ref_y};
			ptsy.insert(ptsy.end(), to_y, end(to_y));
		}

		vector<tuple<double, double>> next_wps;
		for (int i = 0; i < 3; i++)
		{
			next_wps.push_back(getXY(car_s + (i + 1) * POINT_INTERVAL, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y));
		}

		double next_xxx[] = {get<0>(next_wps[0]), get<0>(next_wps[1]), get<0>(next_wps[2])};
		double next_yyy[] = {get<1>(next_wps[0]), get<1>(next_wps[1]), get<1>(next_wps[2])};

		ptsx.insert(ptsx.end(), next_xxx, end(next_xxx));
		ptsy.insert(ptsy.end(), next_yyy, end(next_yyy));

		// shift car reference angle to 0 degree
		for (int i = 0; i < ptsx.size(); i++)
		{
			// shift car reference angle to 0 degrees
			double shift_x = ptsx[i] - ref_x;
			double shift_y = ptsy[i] - ref_y;

			ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
			ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
		}

		tk::spline s;

		s.set_points(ptsx, ptsy);

		vector<double> next_x_vals;
		vector<double> next_y_vals;

		for (int i = 0; i < previous_path_x.size(); i++)
		{
			next_x_vals.push_back(previous_path_x[i]);
			next_y_vals.push_back(previous_path_y[i]);
		}

		double target_x = 300;
		double target_y = s(target_x);
		double target_dist = sqrt(target_x * target_x + target_y * target_y);

		double x_add_on = 0;

		for (int i = 0; i < 50 - previous_path_x.size(); i++)
		{
			double N = target_dist / (0.02 * ref_vel / 2.24);
			double x_point = x_add_on + (target_x / N);
			double y_point = s(x_point);

			x_add_on = x_point;

			double x_ref = x_point;
			double y_ref = y_point;

			x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
			y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

			x_point += ref_x;
			y_point += ref_y;
			next_x_vals.push_back(x_point);
			next_y_vals.push_back(y_point);
		}

		json msgJson;

		msgJson["next_x"] = next_x_vals;
		msgJson["next_y"] = next_y_vals;

		auto msg = "42[\"control\"," + msgJson.dump() + "]";

		//this_thread::sleep_for(chrono::milliseconds(1000));
		ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
					   size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1)
		{
			res->end(s.data(), s.length());
		}
		else
		{
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
	if (h.listen(port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
