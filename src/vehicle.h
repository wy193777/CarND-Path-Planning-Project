#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

  map<string, int> lane_direction = {
    {"PLCL", 1}, 
    {"PLCR", -1},

    {"LCL", 1}, 
    {"LCR", -1}, 
  
    {"LCL2", 2},
    {"LCR2", -2}
  };

  int L = 1;

  int id;

  int preferred_buffer = 20; // impacts "keep lane" behavior.

  int lane;

  int s;

  float velocity;

  float acceleration;

  float target_speed;

  int lanes_available;

  float max_acceleration;

  int goal_lane;

  double goal_s;

  string state;

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(int id, int lane, float s, float v, float a, string state="CS");
  /**
  * Destructor
  */
  virtual ~Vehicle();

  vector<Vehicle> choose_next_state(vector<Vehicle> predictions);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state, vector<Vehicle> predictions);

  vector<float> get_kinematics(vector<Vehicle> predictions, int lane);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(vector<Vehicle> predictions);

  vector<Vehicle> lane_change_trajectory(string state, vector<Vehicle> predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state, vector<Vehicle> predictions);

  void increment(double dt);

  double position_at(double t);

  bool lane_change_not_possible(string state, Vehicle prediction, int new_lane);

  bool get_vehicle_behind(vector<Vehicle> predictions, int lane, Vehicle & rVehicle);

  bool get_vehicle_ahead(vector<Vehicle> predictions, int lane, Vehicle & rVehicle);

  Vehicle generate_predictions(int future_steps, vector<double> sensor_data);

  void realize_next_state(vector<Vehicle> trajectory);

  void update(int lane, float s, float velocity, int future_steps);

  void configure(
    double target_speed,
    int lanes_available,
    double goal_s,
    int goal_lane,
    double max_acceleration
  );

};

#endif