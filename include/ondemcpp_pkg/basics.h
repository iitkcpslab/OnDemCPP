#include <vector>

#include <ros/ros.h>

#include "ondemcpp_pkg/State.h"


using namespace std;


typedef vector<ondemcpp_pkg::State> robots_states_vec;
typedef vector<robots_states_vec> robots_states_mat;

typedef vector<bool> bool_vec;
typedef vector<bool_vec> bool_mat;

typedef vector<int> int_vec;
typedef vector<int_vec> int_mat;

typedef vector<uint> uint_vec;

typedef vector<float> float_vec;
typedef vector<float_vec> float_mat;