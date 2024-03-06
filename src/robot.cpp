/*
Purpose: Robot
Last updated: 
Last updated on: 4 Aug 2022
Author: Ratijit Mitra
*/


#include <ros/ros.h>
#include <ros/package.h>

#include <math.h>
#include <string>
#include <stdlib.h>
#include <vector> 
#include <iostream>
#include <fstream>
#include <string.h>
#include <map>

#include "ondemcpp_pkg/basics.h"
#include "ondemcpp_pkg/config.h"

#include "ondemcpp_pkg/M_Res.h"
#include "ondemcpp_pkg/M_Req.h"

#include "ondemcpp_pkg/CellInfo.h"


#define CP_SERVICE_NAME "/ondemcpp_node/lv"


using namespace ros;


class RobotClass
{	
	public:
		NodeHandle *nh;

		int rob_id;						// Robot ID
		float loc_x;					// Current state
		float loc_y;
		float loc_theta;

		uint ws_size_x;					// Workspace size
		uint ws_size_y;
		float_mat ws_lidar;			// LiDAR values
		float_mat lv;					// Local view of the workspace

		uint hor_id;					// Horizon ID
		ServiceServer path_ss;		// Path Service Server
		robots_states_vec path;
		map<pair<uint, uint>, float> cell_info_map;


		//====================================================================================================
		void initializeRobot(NodeHandle *nh)
		{
			this->nh = nh;
			nh->getParam("rid", rob_id);

			hor_id = 0;
			cell_info_map.clear();

			path_ss = nh->advertiseService("path", &RobotClass::receivePath, this);
		}


		//====================================================================================================
		void populateLiDARValues()
		{
			string ws_obs_robs_file_path = package::getPath("ondemcpp_pkg") + "/input/ws_obs_robs.txt";
			ifstream ws_obs_robs_file;
		    ws_obs_robs_file.open(ws_obs_robs_file_path.c_str()); 
		  	string line;
		  	uint row = 0, col;
		  	float lidar_val;

		    while(ws_obs_robs_file)
		    {
		        getline(ws_obs_robs_file, line);
		        col = 0;
		        char *token = strtok(&line[0], ",");
		        float_vec ws_lidar_row;

		        while(token)
		        {
		        	lidar_val = atof(token);		// 0.0 = Obstacle, 0.5 = Free

		        	if(lidar_val >= 1)		// Robot IDs
		        	{
		        		if(int(lidar_val) == rob_id + 1)			// Robot IDs start from 0
		        		{
		        			loc_x = row;
							loc_y = col;
							loc_theta = 0;		// 0 = E, 1 = N, 2 = W, 3 = S, rob_id % 4, rand() % 4

							cout << "\nR_" << rob_id << " Initial state (" << loc_x << ", " << loc_y << ", " << loc_theta << ")";
		        		}
			        	
			        	ws_lidar_row.push_back(0.5);
		        	}
		        	else
			        	ws_lidar_row.push_back(lidar_val);
		        	
		        	token = strtok(NULL, ",");
		        	col++;
		        	
	        		if(ws_size_y < col)
	        			ws_size_y = col;
		        }

		        row++;
		        ws_lidar.push_back(ws_lidar_row);
		    }

		    ws_size_x = row - 1;
		    ws_obs_robs_file.close();

		    float_mat lv_init(ws_size_x, float_vec(ws_size_y, -1));		// Initialize: -1 = Unexplored
		    lv = lv_init;
		}
		
		
		//====================================================================================================
		void readLiDAR(int nbr_x, int nbr_y)
		{
			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y))		// Valid
				if(lv[nbr_x][nbr_y] == -1)		// Only update unexplored cells
				{
					float lidar_val = lv[nbr_x][nbr_y] = ws_lidar[nbr_x][nbr_y];

					pair<uint, uint> cell_loc_key(nbr_x, nbr_y);

					if(cell_info_map.find(cell_loc_key) == cell_info_map.end())
					{
						pair<pair<uint, uint>, float> cell_info(cell_loc_key, lidar_val);
						cell_info_map.insert(cell_info);
					}
				}
		}
		
		
		//====================================================================================================
		void updateLocalView()
		{
			int cur_x = int(loc_x);
	        int cur_y = int(loc_y);

	        readLiDAR(cur_x + 1, cur_y);		// East cell
	        readLiDAR(cur_x, cur_y + 1);		// North cell
	        readLiDAR(cur_x - 1, cur_y);		// West cell
	        readLiDAR(cur_x, cur_y - 1);		// South cell

			lv[cur_x][cur_y] = 1;			// Robot's current cell is covered

			pair<uint, uint> cell_loc_key(cur_x, cur_y);

			if(cell_info_map.find(cell_loc_key) == cell_info_map.end())
			{
				pair<pair<uint, uint>, float> cell_info(cell_loc_key, 1);
				cell_info_map.insert(cell_info);
			}
			else
				cell_info_map.at(cell_loc_key) = 1;

			// printLocalView();
		}
		
		
		//====================================================================================================
		void printLocalView()
		{
			cout << "\nR_" << rob_id << " Local View (" << hor_id << ")";

			for(uint row_id = 0; row_id < ws_size_x; row_id++)
		    {
		    	for(uint col_id = 0; col_id < ws_size_y; col_id++)
		    		cout << lv[row_id][col_id] << " ";

		    	cout << endl;
		    }
		}
		
		
		//====================================================================================================
		void sendLocalView()
		{
			ServiceClient lv_sc = nh->serviceClient<ondemcpp_pkg::M_Req>(CP_SERVICE_NAME);
			
			ondemcpp_pkg::M_Req lv_srv;
			lv_srv.request.robot_id = rob_id;
			lv_srv.request.state.x = loc_x;
	    	lv_srv.request.state.y = loc_y;
			lv_srv.request.state.theta = loc_theta;

			ondemcpp_pkg::CellInfo cell_info_tmp;
			map<pair<uint, uint>, float>::iterator cell_info_map_it;
			
			for(cell_info_map_it = cell_info_map.begin(); cell_info_map_it != cell_info_map.end(); ++cell_info_map_it)
			{
				// cout << endl << (cell_info_map_it->first).first << "," << (cell_info_map_it->first).second << ": " << cell_info_map_it->second;
				cell_info_tmp.cell_x = uint((cell_info_map_it->first).first);
				cell_info_tmp.cell_y = uint((cell_info_map_it->first).second);
				cell_info_tmp.cell_type = float(cell_info_map_it->second);
				lv_srv.request.lv_short.push_back(cell_info_tmp);
			}

			if(lv_sc.call(lv_srv))
				cout << "\nR_" << rob_id << " sendLocalView(" << hor_id << ").";
			else
			{
				cout << "\nR_" << rob_id << " sendLocalView(" << hor_id << ")! lv_short size = " << lv_srv.request.lv_short.size();
				shutdown();
			}
		}


		bool receivePath(ondemcpp_pkg::M_Res::Request &req, ondemcpp_pkg::M_Res::Response &res)
		{
			path.clear();
			// ondemcpp_pkg::State robot_state;

			for(uint state_id = 0; state_id < req.states.size(); state_id++)
			// {
				// robot_state.x = req.states[state_id].x;
				// robot_state.y = req.states[state_id].y;
				// robot_state.theta = req.states[state_id].theta;
				// path.push_back(robot_state);
				path.push_back(req.states[state_id]);
			// }

			size_t path_size = path.size();

			if(path_size)
			{
				cell_info_map.clear();

				#ifdef TURTLEBOT
					followPath(path_size);
				#else
					followCardinalPath(path_size);
				#endif

				res.next_horizon = hor_id;
			}
			else
			{
				cout << "\nR_" << rob_id << " Coverage Completed ("  << hor_id << ")";
				shutdown();
			}

	  		return true;
		}


		void followPath(size_t path_size)
		{
			float loc_x_next, loc_y_next, loc_theta_next;
			int loc_theta_int = (int)loc_theta, loc_theta_next_int;

			printf("\nFollowing path...\n");
			Time primitive_begin, primitive_end;

			for(size_t state_id = 1; state_id < path_size; state_id++)
			{
				primitive_begin = Time::now();
				loc_x_next = path[state_id].x;
				loc_y_next = path[state_id].y;
				loc_theta_next = path[state_id].theta;
				loc_theta_next_int = (int)loc_theta_next;

				if(loc_theta == loc_theta_next)
					if((loc_x == loc_x_next) && (loc_y == loc_y_next))
						cout << "\nR_" << rob_id << " Waited";
					else if(((loc_x + 1) == loc_x_next) || ((loc_x - 1) == loc_x_next) || ((loc_y + 1) == loc_y_next) || ((loc_y - 1) == loc_y_next))
						cout << "\nR_" << rob_id << " Moved Forward";
					else
					{
						cout << "\nR_" << rob_id << " Invalid motion!";
						shutdown();
					}
				else
					if(((loc_theta_int + 1) % 4) == loc_theta_next_int)
						cout << "\nR_" << rob_id << " Rotated Left";
					else if(((loc_theta_next_int + 1) % 4) == loc_theta_int)
						cout << "\nR_" << rob_id << " Rotated Right";
					else
					{
						cout << "\nR_" << rob_id << " Invalid motion!";
						shutdown();
					}

				loc_x = loc_x_next;
				loc_y = loc_y_next;
				loc_theta = loc_theta_next;
				loc_theta_int = (int)loc_theta;
				
				updateLocalView();
				primitive_end = Time::now();
				Duration(Duration(MOT_PREM_EXEC_TIME) - (primitive_end - primitive_begin)).sleep();
			}

			hor_id++;
			sendLocalView();
		}


		void followCardinalPath(size_t path_size)
		{
			float loc_x_next, loc_y_next, loc_theta_next;
			int loc_theta_int = (int)loc_theta, loc_theta_next_int;

			printf("\nFollowing path...\n");
			Time primitive_begin, primitive_end;

			for(size_t state_id = 1; state_id < path_size; state_id++)
			{
				primitive_begin = Time::now();
				loc_x_next = path[state_id].x;
				loc_y_next = path[state_id].y;
				loc_theta_next = path[state_id].theta;
				loc_theta_next_int = (int)loc_theta_next;

				if((loc_x == loc_x_next) && (loc_y == loc_y_next))
					cout << "\nR_" << rob_id << " Halted = " << state_id;
				else
					if(loc_x == loc_x_next)
						if((loc_y + 1) == loc_y_next)
							cout << "\nR_" << rob_id << " Moved Top (" << loc_x_next << ", " << loc_y_next << ") = " << state_id;
						else if((loc_y - 1) == loc_y_next)
							cout << "\nR_" << rob_id << " Moved Bottom (" << loc_x_next << ", " << loc_y_next << ") = " << state_id;
						else
						{
							cout << "\nR_" << rob_id << " Invalid motion! (" << loc_x << ", " << loc_y << ") to (" << loc_x_next << ", " << loc_y_next << ") = " << state_id;
							shutdown();
						}
					else if(loc_y == loc_y_next)
						if((loc_x + 1) == loc_x_next)
							cout << "\nR_" << rob_id << " Moved Right (" << loc_x_next << ", " << loc_y_next << ") = " << state_id;
						else if((loc_x - 1) == loc_x_next)
							cout << "\nR_" << rob_id << " Moved Left (" << loc_x_next << ", " << loc_y_next << ") = " << state_id;
						else
						{
							cout << "\nR_" << rob_id << " Invalid motion! (" << loc_x << ", " << loc_y << ") to (" << loc_x_next << ", " << loc_y_next << ") = " << state_id;
							shutdown();
						}
					else
					{
						cout << "\nR_" << rob_id << " Invalid motion! (" << loc_x << ", " << loc_y << ") to (" << loc_x_next << ", " << loc_y_next << ") = " << state_id;
						shutdown();
					}

				loc_x = loc_x_next;
				loc_y = loc_y_next;
				loc_theta = loc_theta_next;

				updateLocalView();
				primitive_end = Time::now();
				Duration(Duration(MOT_PREM_EXEC_TIME) - (primitive_end - primitive_begin)).sleep();
			}

			hor_id++;
			sendLocalView();
		}
};


//====================================================================================================
int main(int argc, char *argv[])
{	
	init(argc, argv, "robot_node");		// Rename using __name:=robot_i in the rosrun command where i (>= 0) is the robot id
	NodeHandle nh("~");
	
	RobotClass *robot_obj = new RobotClass();
	robot_obj->initializeRobot(&nh);
	robot_obj->populateLiDARValues();
	robot_obj->updateLocalView();
	robot_obj->sendLocalView();

	spin();
	return 0;
}