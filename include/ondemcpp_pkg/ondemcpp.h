/*
Purpose : OnDemCPP
Last updated : 
Last updated on : 26 July 2022
Author : Ratijit Mitra
*/


#include <stdio.h>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <stack>


#include <ros/ros.h>
#include <ros/package.h>


#include "ondemcpp_pkg/ma.h"
#include "ondemcpp_pkg/config.h"
#include "ondemcpp_pkg/debug.h"


#define OPTIMAL_PATH_LENGTHS_FILE "_capl.txt"


using namespace std;
using namespace ros;


class OnDemCPP
{
	public:
		robots_states_mat runOnDemCPP(int ws_size_x, int ws_size_y, bool_mat ws_graph, int num_of_robs, robots_states_vec robs_states, int num_of_goals, robots_states_vec goals_locs, uint horizon, bool_vec S_r, robots_states_mat paths_old);
		
		void showQueue(queue<ondemcpp_pkg::State> BFS_QUEUE);
		void showMatrix(int_mat v_2d);
		
		int_mat bfs(int ws_size_x, int ws_size_y, bool_mat ws_graph, ondemcpp_pkg::State cur_rob_state);
		int_mat bfs_longitudinal(int ws_size_x, int ws_size_y, bool_mat ws_graph, ondemcpp_pkg::State cur_rob_state);
		int_mat compute_optimal_costs(int ws_size_x, int ws_size_y, bool_mat ws_graph, robots_states_vec robs_states, robots_states_vec goals_locs);
		
		robots_states_vec optimal_path_bfs(int ws_size_x, int ws_size_y, bool_mat ws_graph, ondemcpp_pkg::State rob_loc, ondemcpp_pkg::State goal_loc);
		robots_states_vec optimal_path_bfs_longitudinal(int ws_size_x, int ws_size_y, bool_mat ws_graph, ondemcpp_pkg::State rob_loc, ondemcpp_pkg::State goal_loc);
		robots_states_mat compute_optimal_paths(bool_mat ws_graph, robots_states_vec robs_states, robots_states_vec goals_locs, int_vec opt_goal_vec, bool_vec S_r, robots_states_mat paths_old, uint &active_count);
		
		bool is_type1_crossover_path_pair(int rob_1, int rob_2, ondemcpp_pkg::State rob1_start_loc, robots_states_vec path_2);
		bool is_type2_crossover_path_pair(int rob_1, int rob_2, robots_states_vec path_1, robots_states_vec path_2);

		bool is_nested_path_pair(int rob_1, int rob_2, robots_states_vec path_1, robots_states_vec path_2);
		robots_states_vec adjust_path(ondemcpp_pkg::State init_state, robots_states_vec opt_path);
		bool test_path(int i, robots_states_vec path, int_vec goal_vec_new, robots_states_mat feasible_paths, bool_vec S_r);
		robots_states_mat get_feasible_paths(int_vec &opt_goal_vec, robots_states_mat opt_paths, uint &killed_count, uint &revived_count, uint &revived_goal_count, int ws_size_x, int ws_size_y, bool_vec S_r);

		bool_mat compute_partial_orders(int num_of_robs, robots_states_mat paths, bool_vec S_r);

		int_vec compute_total_order(int num_of_robs, bool_mat adj, bool_mat &adj_residue);

		void break_dependency_cycles(int_vec &opt_goal_vec, bool_mat po_residue, bool_vec S_r);
		void adjust_dependent_paths(int_vec opt_goal_vec, robots_states_mat &paths, bool_vec S_r, uint &active_count);

		int_vec compute_start_time_offsets(int_vec total_order, robots_states_mat paths, bool_vec S_r, int_vec &opt_goal_vec, bool &flag_inactivated_robot);

		robots_states_mat compute_optimal_trajectories(int num_of_robs, int_vec time_offsets, robots_states_mat paths, bool_vec S_r, uint horizon);

		void monitor_paths(int ws_size_x, int ws_size_y, int num_of_robs, robots_states_mat trajectories);
};