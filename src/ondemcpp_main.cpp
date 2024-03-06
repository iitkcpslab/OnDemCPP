/*
Purpose : On Demand Coverage Path Planner
Last updated : 
Last updated on : 3 Aug 2022
Author : Ratijit Mitra
*/


#include "ondemcpp_pkg/ondemcpp_main.h"


using namespace ros;


void OnDemCPP_MAIN::initializeOnDemCPP(NodeHandle *nh)
{
	this->nh = nh;

	nh->getParam("ws_x", ws_size_x);
	nh->getParam("ws_y", ws_size_y);
	nh->getParam("rc", rob_count);
	cout << ws_size_x << "x" << ws_size_y << "_" << rob_count << endl;

	float_mat gv_init(ws_size_x, float_vec(ws_size_y, -1));			// -1 = Unexplored
	gv = gv_init;

	bool_mat assigned_goals_tmp(ws_size_x, bool_vec(ws_size_y, false));			// All goals are initially unassigned (false)
	assigned_goals = assigned_goals_tmp;

	bool_vec is_participant_rob_tmp(rob_count, false);
	S_req = is_participant_rob_tmp;

	robots_states_vec robots_states_tmp(rob_count);
	robots_states = robots_states_tmp;

	robots_states_mat paths_tmp(rob_count, robots_states_vec(1));
	paths = paths_tmp;

	uint_vec path_lens_tmp(rob_count, 0);
	path_lens = path_lens_tmp;

	lv_ss = nh->advertiseService("lv", &OnDemCPP_MAIN::getLocalView, this);
	// printf("Server = share_workspace started.\n==================================================\n");

	hor_id = 0;
	lv_count = 0;

	begin_mis = false;
	tot_comp_time = 0;
	tot_hor_len = 0;

	ondemcpp_pkg_path = package::getPath("ondemcpp_pkg") + OUTPUT_DIR;
	ofstream rph_file;			// Empty resultPerHorizon.txt
	rph_file.open((ondemcpp_pkg_path + RESULT_PER_HORIZON_FILE).c_str(), ios::out);
	rph_file.close();
}


bool OnDemCPP_MAIN::getLocalView(ondemcpp_pkg::M_Req::Request &req, ondemcpp_pkg::M_Req::Response &res)
{
	lv_count++;

	if(lv_count == 1)
	{
		string pre_comp_file_path = ondemcpp_pkg_path + "pre_comp" + TXT_EXT;
		fstream pre_comp_file;
		pre_comp_file.open(pre_comp_file_path.c_str(), fstream::app);
		pre_comp_start_time = Time::now();
		pre_comp_file << pre_comp_start_time << " - ";
		pre_comp_file.close();
	}

	if(!begin_mis)
	{
		mis_time_begin = Time::now();
		begin_mis = true;
	}

	int rob_id = (int)req.robot_id;

	string rob_file_path = ondemcpp_pkg_path + "r_" + to_string(rob_id) + TXT_EXT;
	fstream rob_file;
	rob_file.open(rob_file_path.c_str(), fstream::app);
	Time gv_update_start_time = Time::now();
	rob_file << gv_update_start_time << " - ";
	robots_states[rob_id].x = req.state.x;
	robots_states[rob_id].y = req.state.y;
	robots_states[rob_id].theta = req.state.theta;

	updateGlobalView(req);
	cout << "CP recieved local view from R_" << rob_id << " (" << robots_states[rob_id].x << ", " << robots_states[rob_id].y << ", " << fixed << setprecision(0) << robots_states[rob_id].theta << "). #Requests = " << lv_count << endl;
	Time gv_update_end_time = Time::now();
	rob_file << gv_update_end_time << " = " << fixed << (gv_update_end_time - gv_update_start_time).toSec() << endl;

	if((!hor_id && (lv_count == rob_count)) || (hor_id && (lv_count == active_rob_count)))
	{
		boost::thread* thread_1 = new boost::thread(boost::bind(&OnDemCPP_MAIN::checkCoveragePlanningCriteria, this));
		thread_1->detach();
	}

	res.next_horizon = hor_id;
	return true;
}


void OnDemCPP_MAIN::printGlobalView()
{
	cout << "\nGlobal view...\n";

	for(uint row_id = 0; row_id < ws_size_x; row_id++)
    {
    	for(uint col_id = 0; col_id < ws_size_y; col_id++)
    		cout << gv[row_id][col_id] << " ";

    	cout << endl;
    }
}


void OnDemCPP_MAIN::updateGlobalView(ondemcpp_pkg::M_Req::Request &req)
{
	size_t lv_short_size = req.lv_short.size();

	for(size_t lv_index = 0; lv_index < lv_short_size; lv_index++)
	{
		ondemcpp_pkg::CellInfo cell_info_tmp = req.lv_short[lv_index];
		float lv = float(cell_info_tmp.cell_type);		// Local view

		if((lv == 0) || (lv == 0.5))		// Obstacle or Goal
		{
			if(gv[cell_info_tmp.cell_x][cell_info_tmp.cell_y] == -1)
				gv[cell_info_tmp.cell_x][cell_info_tmp.cell_y] = lv;
		}
		else		// Covered
			gv[cell_info_tmp.cell_x][cell_info_tmp.cell_y] = lv;
	}

	// printGlobalView();
}


void OnDemCPP_MAIN::countExploredCells()
{
	obs_count = unassigned_goal_count = cov_count = 0;

	for(uint row_id = 0; row_id < ws_size_x; row_id++)
		for(uint col_id = 0; col_id < ws_size_y; col_id++)
		{
			float gv_cell = gv[row_id][col_id];		// Global view

			if(gv_cell == 0)		// Obstacle
				obs_count++;
			else if(gv_cell == 1)		// Covered
				cov_count++;
			else if((gv_cell == 0.5) && !assigned_goals[row_id][col_id])		// 0.5 = Goal
				unassigned_goal_count++;
		}

	cout << "Problem: #O = " << obs_count << " #UG = " << unassigned_goal_count << " #C = " << cov_count;
}


void OnDemCPP_MAIN::checkCoveragePlanningCriteria()
{
	countExploredCells();

	if(unassigned_goal_count)		// Have unassigned goals
	{
		cout << ": START" << endl;
		startCoveragePlanning();
	}
	else if(active_rob_count - nr_rob_count)		// Have residual paths
	{
		cout << ": SKIP" << endl;
		skipCoveragePlanning();
	}
	else		// Have no residual path
	{
		cout << ": STOP" << endl;
		stopCoveragePlanning();
	}
}


void OnDemCPP_MAIN::stopCoveragePlanning()
{
	Duration mis_time = Time::now() - mis_time_begin;		// Mission time

	ofstream r_file;
	string rph_file_path = ondemcpp_pkg_path + RESULT_FILE;
	r_file.open(rph_file_path.c_str());

	#ifdef TURTLEBOT
		r_file << "Workspace Size = " << ws_size_x << " x " << ws_size_y << ", #Robots = " << rob_count << ", #Horizons = " << hor_id << ", Total Computation Time = " << fixed << tot_comp_time << ", Total Horizon Length = " << tot_hor_len << ", Mission Time = " << fixed << mis_time.toSec() << ", TurtleBot";
	#else
		r_file << "Workspace Size = " << ws_size_x << " x " << ws_size_y << ", #Robots = " << rob_count << ", #Horizons = " << hor_id << ", Total Computation Time = " << fixed << tot_comp_time << ", Total Horizon Length = " << tot_hor_len << ", Mission Time = " << fixed << mis_time.toSec() << ", QuadCopter";
	#endif

	r_file.close();

	robots_states_mat paths_tbs(rob_count, robots_states_vec());
	sendPaths(paths_tbs, true);

	ofstream path_len_file;
	string path_len_file_path = ondemcpp_pkg_path + PATH_LEN_FILE;
	path_len_file.open(path_len_file_path.c_str());

	for(uint rob_id = 0; rob_id < rob_count; rob_id++)
		path_len_file << rob_id << " = " << path_lens[rob_id] << endl;

	path_len_file.close();
	
	cout << "\n\n********** Coverage Completed **********" << endl;
	cout << "#Horizons = " << hor_id << ", Total computation time = " << tot_comp_time << ", Total horizon length = " << tot_hor_len << endl;
	shutdown();
}


void OnDemCPP_MAIN::sendPaths(robots_states_mat paths_tbs, bool flag_send_to_all)
{
	for(uint rob_id = 0; rob_id < rob_count; rob_id++)			// Send individual paths to each robot
	{
		robots_states_vec path = paths_tbs[rob_id];
		int path_len = path.size() - 1;
		ondemcpp_pkg::M_Res pfh_srv;

		for(int state_id = 0; state_id <= path_len; state_id++)
			pfh_srv.request.states.push_back(path[state_id]);

		if(flag_send_to_all || path_len)		// Active robot
		{
			string path_srv_name = "/robot_" + to_string(rob_id) + "/path";
			ServiceClient path_sc = nh->serviceClient<ondemcpp_pkg::M_Res>(path_srv_name);
			
			if(flag_send_to_all)
				callRobot(path_sc, pfh_srv, path_srv_name, rob_id, path_len);
			else
			{
				boost::thread* thread_2 = new boost::thread(boost::bind(&OnDemCPP_MAIN::callRobot, this, path_sc, pfh_srv, path_srv_name, rob_id, path_len));
				thread_2->detach();
			}
		}
	}
}


void OnDemCPP_MAIN::callRobot(ServiceClient path_sc, ondemcpp_pkg::M_Res pfh_srv, string path_srv_name, uint rob_id, uint path_len)
{
	if(path_sc.call(pfh_srv))
	{
		cout << path_srv_name << "." << endl;
		path_lens[rob_id] += path_len;
	}
	else
		cout << path_srv_name << "! states size = " << pfh_srv.request.states.size() << endl;
}


void OnDemCPP_MAIN::printHorizonInformation(uint req_count, int hor_len, double comp_time)
{
	ofstream rph_file;
	string rph_file_path = ondemcpp_pkg_path + RESULT_PER_HORIZON_FILE;
	rph_file.open(rph_file_path.c_str(), fstream::app);
	rph_file << "Horizon = " << hor_id << ", #Requesters = " << req_count << ", #O = " << obs_count << ", #UG = " << unassigned_goal_count << ", #C = " << cov_count << ", Computation time = " << fixed << comp_time << ", Horizon length = " << hor_len << ", #Active robots = " << active_rob_count << endl;
	rph_file.close();

	cout << "Horizon = " << hor_id << ", #Requesters = " << req_count << ", #O = " << obs_count << ", #UG = " << unassigned_goal_count << ", #C = " << cov_count << ", Computation time = " << fixed << comp_time << ", Horizon length = " << hor_len << ", #Active robots = " << active_rob_count << endl;
	cout << "====================================================================================================" << endl;
}


void OnDemCPP_MAIN::updateHorizonInformation(double comp_time, uint hor_len)
{
	lv_count = 0;
	hor_id++;
	tot_comp_time += comp_time;
	tot_hor_len += hor_len;
	unassigned_goal_count -= newly_assigned_goal_count;
}


void OnDemCPP_MAIN::startCoveragePlanning()
{
	bool_mat ws_graph(ws_size_x, bool_vec(ws_size_y, true));		// Graph representation of the workspace; true = Covered, Goal; false = Unexplored, Obstacle
	robots_states_vec unassigned_goal_locs;		// Locations of Unassigned Goals

	string pre_comp_file_path = ondemcpp_pkg_path + "pre_comp" + TXT_EXT;
	fstream pre_comp_file;
	pre_comp_file.open(pre_comp_file_path.c_str(), fstream::app);
	Time pre_comp_end_time = Time::now();
	pre_comp_file << pre_comp_end_time << " = " << fixed << (pre_comp_end_time - pre_comp_start_time).toSec() << endl;
	pre_comp_file.close();

	Time comp_time_begin = Time::now();

	ondemcpp_pkg::State unassigned_goal_loc;

	for(uint row_id = 0; row_id < ws_size_x; row_id++)
		for(uint col_id = 0; col_id < ws_size_y; col_id++)
		{
			float gv_cell = gv[row_id][col_id];

			if((gv_cell == -1) || (gv_cell == 0))		// -1 = Unexplored, 0 = Obstacle
				ws_graph[row_id][col_id] = false;
			else if((gv_cell == 0.5) && !assigned_goals[row_id][col_id])		// 0.5 = Goal
			{
				unassigned_goal_loc.x = row_id;
				unassigned_goal_loc.y = col_id;
				unassigned_goal_locs.push_back(unassigned_goal_loc);
			}
		}

	robots_states_vec req_rob_states;		// States of requesters
	uint req_count = 0;						// Number of requesters
	
	cout << "\nRequesters: ";

	for(uint rob_id = 0; rob_id < rob_count; rob_id++)
		if(paths[rob_id].size() - 1)
		{
			cout << "0 ";
			S_req[rob_id] = false;
		}
		else
		{
			cout << "R_" << rob_id << " ";
			S_req[rob_id] = true;
			req_count++;
			req_rob_states.push_back(robots_states[rob_id]);
		}

	OnDemCPP ondemcpp_obj;
	paths = ondemcpp_obj.runOnDemCPP(ws_size_x, ws_size_y, ws_graph, req_rob_states.size(), req_rob_states, unassigned_goal_locs.size(), unassigned_goal_locs, hor_id, S_req, paths);		// Paths of Robots

	Duration comp_time = Time::now() - comp_time_begin;		// Computation Time
	uint hor_len;			// Horizon length

	string post_comp_file_path = ondemcpp_pkg_path + "post_comp" + TXT_EXT;
	fstream post_comp_file;
	post_comp_file.open(post_comp_file_path.c_str(), fstream::app);
	Time post_comp_start_time = Time::now();
	post_comp_file << post_comp_start_time << " - ";

	robots_states_mat paths_tbs(rob_count, robots_states_vec());
	processPaths(paths, hor_len, paths_tbs);
	sendPaths(paths_tbs, false);
	printHorizonInformation(req_count, hor_len, comp_time.toSec());
	updateHorizonInformation(comp_time.toSec(), hor_len);

	Time post_comp_end_time = Time::now();
	post_comp_file << post_comp_end_time << " = " << fixed << (post_comp_end_time - post_comp_start_time).toSec() << endl;
	post_comp_file.close();
}


void OnDemCPP_MAIN::skipCoveragePlanning()
{
	uint req_count = 0;				// Number of requesters
	
	cout << "\nRequesters: ";

	string pre_comp_file_path = ondemcpp_pkg_path + "pre_comp" + TXT_EXT;
	fstream pre_comp_file;
	pre_comp_file.open(pre_comp_file_path.c_str(), fstream::app);
	Time pre_comp_end_time = Time::now();
	pre_comp_file << pre_comp_end_time << " = " << fixed << (pre_comp_end_time - pre_comp_start_time).toSec() << endl;
	pre_comp_file.close();

	Time comp_time_begin = Time::now();

	for(uint rob_id = 0; rob_id < rob_count; rob_id++)
		if(paths[rob_id].size() - 1)
			cout << "0 ";
		else
		{
			cout << "R_" << rob_id << " ";
			req_count++;
		}

	Duration comp_time = Time::now() - comp_time_begin;		// Computation Time

	string post_comp_file_path = ondemcpp_pkg_path + "post_comp" + TXT_EXT;
	fstream post_comp_file;
	post_comp_file.open(post_comp_file_path.c_str(), fstream::app);
	Time post_comp_start_time = Time::now();
	post_comp_file << post_comp_start_time << " - ";

	uint hor_len;			// Horizon length
	robots_states_mat paths_tbs(rob_count, robots_states_vec());
	processPaths(paths, hor_len, paths_tbs);
	sendPaths(paths_tbs, false);
	printHorizonInformation(req_count, hor_len, comp_time.toSec());
	updateHorizonInformation(comp_time.toSec(), hor_len);

	Time post_comp_end_time = Time::now();
	post_comp_file << post_comp_end_time << " = " << fixed << (post_comp_end_time - post_comp_start_time).toSec() << endl;
	post_comp_file.close();
}


void OnDemCPP_MAIN::determineHorizonLength(robots_states_mat paths, uint &hor_len)
{
	bool is_hor_len_init = false;		// Is the horizon length initialized?

	for(uint rob_id = 0; rob_id < rob_count; rob_id++)
	{
		uint path_len = paths[rob_id].size() - 1;

		if(path_len)
			if(!is_hor_len_init)
			{
				hor_len = path_len;
				is_hor_len_init = true;
			}
			else if(hor_len > path_len)
				hor_len = path_len;
	}
}


void OnDemCPP_MAIN::processPaths(robots_states_mat &paths, uint &hor_len, robots_states_mat &paths_tbs)
{
	determineHorizonLength(paths, hor_len);
	active_rob_count = 0;
	nr_rob_count = 0;
	robots_states_vec path;		// Path of a Robot
	newly_assigned_goal_count = 0;
	ondemcpp_pkg::State rob_goal_state;

	for(uint rob_id = 0; rob_id < rob_count; rob_id++)
	{
		path = paths[rob_id];
		uint path_len = path.size() - 1;

		if(path_len)		// Active
		{
			for(uint state_id = 0; state_id <= hor_len; state_id++)
			{
				paths_tbs[rob_id].push_back(path[state_id]);

				if(state_id != hor_len)
					paths[rob_id].erase(paths[rob_id].begin());
			}

			active_rob_count++;
			rob_goal_state = path[path_len];

			if(!assigned_goals[rob_goal_state.x][rob_goal_state.y])
			{
    			assigned_goals[rob_goal_state.x][rob_goal_state.y] = true;
				newly_assigned_goal_count++;
			}

    		if(path_len == hor_len)		// No residual path
    			nr_rob_count++;
		}
		else		// Inactive
		{
			paths_tbs[rob_id].push_back(path[0]);
			S_req[rob_id] = true;
		}
	}
}


int main(int argc, char *argv[])
{	
	init(argc, argv, "ondemcpp_node");
	NodeHandle nh("~");
	
	OnDemCPP_MAIN *ondemcpp_main_obj = new OnDemCPP_MAIN();
	ondemcpp_main_obj -> initializeOnDemCPP(&nh);

	spin();
	return 0;
}