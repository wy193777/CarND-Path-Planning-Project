#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle() {}

Vehicle::Vehicle(int lane, float s, float velocity, float acceleration, string state)
{

    this->lane = lane;
    this->s = s;
    this->velocity = velocity;
    this->acceleration = acceleration;
    this->state = state;
    max_acceleration = -1;
}

Vehicle::~Vehicle() {}

vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions)
{
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is acceleration map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are acceleration vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
    vector<string> states = successor_states();
    float cost;
    vector<float> costs;
    vector<string> final_states;
    vector<vector<Vehicle>> final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it)
    {
        vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
        if (trajectory.size() != 0)
        {
            cost = calculate_cost(*this, predictions, trajectory);
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
        }
    }

    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states()
{
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    if (state.compare("KL") == 0)
    {
        states.push_back("PLCL");
        states.push_back("PLCR");
    }
    else if (state.compare("PLCL") == 0)
    {
        if (lane != lanes_available - 1)
        {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    }
    else if (state.compare("PLCR") == 0)
    {
        if (lane != 0)
        {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }
    //If state is "LCL" or "LCR", then just return "KL"
    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions)
{
    /*
    Given acceleration possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<Vehicle> trajectory;
    if (state.compare("CS") == 0)
    {
        trajectory = constant_speed_trajectory();
    }
    else if (state.compare("KL") == 0)
    {
        trajectory = keep_lane_trajectory(predictions);
    }
    else if (state.compare("LCL") == 0 || state.compare("LCR") == 0)
    {
        trajectory = lane_change_trajectory(state, predictions);
    }
    else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0)
    {
        trajectory = prep_lane_change_trajectory(state, predictions);
    }
    return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane)
{
    /* 
    Gets next timestep kinematics (position, velocity, acceleration) 
    for acceleration given lane. Tries to choose the maximum velocity and acceleration, 
    given other vehicle positions and accel/velocity constraints.
    */
    float max_velocity_accel_limit = this->max_acceleration + this->velocity;
    float new_position;
    float new_velocity;
    float new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead))
    {

        if (get_vehicle_behind(predictions, lane, vehicle_behind))
        {
            new_velocity = vehicle_ahead.velocity; //must travel at the speed of traffic, regardless of preferred buffer
        }
        else
        {
            float max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.velocity - 0.5 * (this->acceleration);
            new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
        }
    }
    else
    {
        new_velocity = min(max_velocity_accel_limit, this->target_speed);
    }

    new_accel = new_velocity - this->velocity; //Equation: (v_1 - v_0)/t = acceleration
    new_position = this->s + new_velocity + new_accel / 2.0;
    return {new_position, new_velocity, new_accel};
}

vector<Vehicle> Vehicle::constant_speed_trajectory()
{
    /*
    Generate acceleration constant speed trajectory.
    */
    printf("constant_speed_trajectory\n");
    float next_pos = position_at(1);
    vector<Vehicle> trajectory = {
        Vehicle(this->lane, this->s, this->velocity, this->acceleration, this->state),
        Vehicle(this->lane, next_pos, this->velocity, 0, this->state),
        Vehicle(this->lane, position_at(2), this->velocity, 0, this->state)};
    return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions)
{
    /*
    Generate acceleration keep lane trajectory.
    */
    printf("keep_lane_trajectory\n");
    vector<Vehicle> trajectory = {
        Vehicle(lane, this->s, this->velocity, this->acceleration, state)};
    vector<float> kinematics = get_kinematics(predictions, this->lane);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
    trajectory.push_back(Vehicle(this->lane, new_s + 30, new_v, new_a, "KL"));
    return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions)
{
    /*
    Generate acceleration trajectory preparing for acceleration lane change.
    */
    printf("prep_lane_change_trajectory\n");
    float new_s;
    float new_v;
    float new_a;
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory = {
        Vehicle(this->lane, this->s, this->velocity, this->acceleration, this->state)};
    vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

    if (get_vehicle_behind(predictions, this->lane, vehicle_behind))
    {
        //Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];
    }
    else
    {
        vector<float> best_kinematics;
        vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
        //Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1])
        {
            best_kinematics = next_lane_new_kinematics;
        }
        else
        {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
    }

    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions)
{
    /*
    Generate acceleration lane change trajectory.
    */
    printf("lane_change_trajectory\n");
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    //Check if acceleration lane change is possible (check if another vehicle occupies that spot).
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it)
    {
        next_lane_vehicle = it->second[0];
        if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane)
        {
            //If lane change is not possible, return empty trajectory.
            return trajectory;
        }
    }
    trajectory.push_back(
        Vehicle(this->lane, this->s, this->velocity, this->acceleration, this->state));
    vector<float> kinematics = get_kinematics(predictions, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
    return trajectory;
}

void Vehicle::increment(double dt = 1)
{
    this->s = position_at(dt);
}

double Vehicle::position_at(double t)
{
    return this->s + this->velocity * t + this->acceleration * t * t / 2.0;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle &rVehicle)
{
    /*
    Returns acceleration true if acceleration vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if acceleration vehicle is found.
    */
    int max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it)
    {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s)
        {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle &rVehicle)
{
    /*
    Returns acceleration true if acceleration vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if acceleration vehicle is found.
    */
    double min_s = this->goal_s;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it)
    {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s)
        {
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon)
{
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */
    vector<Vehicle> predictions;
    for (int i = 0; i < horizon; i++)
    {
        float next_s = position_at(i);
        float next_v = 0;
        if (i < horizon - 1)
        {
            next_v = position_at((i + 1) * 0.02) - s;
        }
        predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
    }
    return predictions;
}

void Vehicle::realize_next_state(vector<Vehicle> trajectory)
{
    /*
    Sets state and kinematics for ego vehicle using the last state of the trajectory.
    */
    Vehicle next_state = trajectory[1];
    this->state = next_state.state;
    this->lane = next_state.lane;
    this->s = next_state.s;
    this->velocity = next_state.velocity;
    this->acceleration = next_state.acceleration;
}

void Vehicle::update(int lane, float s, float velocity, float acceleration)
{
    this->lane = lane;
    this->s = s;
    this->velocity = velocity;
    this->acceleration = acceleration;
}

void Vehicle::configure(
    double target_speed,
    int lanes_available,
    double goal_s,
    int goal_lane,
    double max_acceleration)
{
    /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle. 
    */
    this->target_speed = target_speed;
    this->lanes_available = lanes_available;
    this->goal_s = goal_s;
    this->goal_lane = goal_lane;
    this->max_acceleration = max_acceleration;
}