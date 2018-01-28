#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;

namespace Cost {
float calculate_cost(
    const Vehicle &vehicle,
    const vector<Vehicle> &predictions,
    const vector<Vehicle> &trajectory);

float goal_distance_cost(
    const Vehicle &vehicle,
    const vector<Vehicle> &trajectory,
    const vector<Vehicle> &predictions,
    map<string, float> &data);

float inefficiency_cost(
    const Vehicle &vehicle,
    const vector<Vehicle> &trajectory,
    const vector<Vehicle> &predictions,
    map<string, float> &data);

float lane_speed(const vector<Vehicle> &predictions, int lane);

map<string, float> get_helper_data(
    const Vehicle &vehicle,
    const vector<Vehicle> &trajectory,
    const vector<Vehicle> &predictions);
}

#endif