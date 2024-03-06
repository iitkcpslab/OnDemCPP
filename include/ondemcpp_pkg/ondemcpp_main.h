/*
Purpose : OnDemCPP Main
Last updated : 
Last updated on : 3 Aug 2022
Author : Ratijit Mitra
*/


#include <stdio.h>
#include <math.h>
#include <cstdlib>		// srand
#include <algorithm>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <stack>
#include <iomanip>		// fixed


#include <ros/ros.h>
#include <ros/package.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <pthread.h>


#include <boost/thread/thread.hpp>


#include "ondemcpp_pkg/ondemcpp.h"


#include "ondemcpp_pkg/M_Res.h"
#include "ondemcpp_pkg/M_Req.h"


#define RESULT_PER_HORIZON_FILE "resultPerHorizon.txt"
#define RESULT_FILE "result.txt"
#define PATH_LEN_FILE "path_len.txt"


using namespace std;
using namespace ros;


class OnDemCPP_MAIN
{
	public:
		void initializeOnDemCPP(NodeHandle *nh);
		bool getLocalView(ondemcpp_pkg::M_Req::Request &req, ondemcpp_pkg::M_Req::Response &res);
		void updateGlobalView(ondemcpp_pkg::M_Req::Request &req);
		void printGlobalView();
		void checkCoveragePlanningCriteria();
		void countExploredCells();
		void stopCoveragePlanning();
		void sendPaths(robots_states_mat paths_tbs, bool flag_send_to_all);
		void callRobot(ServiceClient path_sc, ondemcpp_pkg::M_Res pfh_srv, string path_srv_name, uint rob_id, uint path_len);
		void printHorizonInformation(uint clusters_count, int horizon_length, double comp_time);
		void updateHorizonInformation(double comp_time, uint hor_len);
		void startCoveragePlanning();
		void skipCoveragePlanning();
		void determineHorizonLength(robots_states_mat paths, uint &hor_len);
		void processPaths(robots_states_mat &paths, uint &hor_len, robots_states_mat &paths_tbs);

		NodeHandle *nh;
		
		int ws_size_x;				// Workspace size
		int ws_size_y;
		int rob_count;				// Number of robots

		float_mat gv;						// Globalview
		robots_states_vec robots_states;
		robots_states_mat paths;					// Paths
		uint_vec path_lens;					// Path lengths

		bool_vec S_req;						// Set of requesters
		bool_mat assigned_goals;		// Goals which have been already assigned to robots

		uint hor_id;					// Horizon ID

		ServiceServer lv_ss;		// Localview service server
		uint lv_count;					// Number of localviews

		bool begin_mis;					// Begin the mission
		Time mis_time_begin;		// Begining of the mission time
		double tot_comp_time;			// Total computation time
		uint tot_hor_len;				// Total horizon length

		uint active_rob_count;		// Number of active robots
		uint nr_rob_count;			// Number of active robots which have no residual paths
		uint obs_count;				// Number of Obstacle Cells
		uint unassigned_goal_count;			// Number of Goal Cells
		uint cov_count;				// Number of Covered Cells
		uint newly_assigned_goal_count;

		Time pre_comp_start_time;
		string ondemcpp_pkg_path;
};