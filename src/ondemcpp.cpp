/*
Purpose : 	OnDemCPP -
				1.	compute_optimal_costs()
					1a.	optimal_cost_bfs()
				2.	munkres()
				3.	compute_optimal_paths()
					3a. optimal_path_bfs()
				4.	while(true)
					5.	adjust_crossover_paths()
						5a.	is_crossover_path_pair()
						5b.	adjust_crossover_path_pair()
					6. adjust_nested_paths()
						6a.	is_nested_path_pair()
						6b.	traverse_nary_tree()
					7. 	adjust_collision_prone_paths()
					8. 	if(Assignment changed)
							continue
					9.	compute_partial_orders()
					10.	compute_total_order()
					11.	if(Invalid TO)
							12. break_dependency_cycles()
						else
							13.	compute_start_time_offsets()
							14. if(Assignment has not changed)
									break										
					15.	adjust_dependent_paths()
				16.	compute_optimal_trajectories()
Last updated: Start time offsets
Last updated on: 17 Nov 2022
Author: Ratijit Mitra
*/


#include "ondemcpp_pkg/ondemcpp.h"


//==================================================1. Optimal Costs : START
void OnDemCPP::showQueue(queue<ondemcpp_pkg::State> BFS_QUEUE)
{
	while(!BFS_QUEUE.empty())
	{
		ondemcpp_pkg::State temp_loc = BFS_QUEUE.front();
        BFS_QUEUE.pop();

		cout << "(" << temp_loc.x << ", " << temp_loc.y << ", " << temp_loc.theta << ") ";
	}
}


void OnDemCPP::showMatrix(int_mat v_2d)
{
	int v_2d_size = v_2d[0].size();
	int i, j;

	//for(j = v_2d_size - 1; j >= 0; j--)
	for(i = 0; i < v_2d_size; i++)
	{
		//for(i = 0; i < v_2d_size; i++)
		for(j = 0; j < v_2d_size; j++)
		{
			cout << v_2d[i][j] << " ";
		}

		cout << endl;
	}
}


int_mat OnDemCPP::bfs(int ws_size_x, int ws_size_y, bool_mat ws_graph, ondemcpp_pkg::State cur_rob_state)
{
	int cur_rob_x = cur_rob_state.x;
	int cur_rob_y = cur_rob_state.y;
	int cur_rob_theta = cur_rob_state.theta;
	//cout << "(" << cur_rob_x << ", " << cur_rob_y << ", " << cur_rob_theta <<")";

	int_mat cost_mat(ws_size_x, int_vec(ws_size_y, COST_INF));
	int_mat direction(ws_size_x, int_vec(ws_size_y, -1));

	cost_mat[cur_rob_x][cur_rob_y] = 0;

	//==================================================EnQueue : START
	queue<ondemcpp_pkg::State> BFS_QUEUE;
	ondemcpp_pkg::State temp_loc;
	temp_loc.x = cur_rob_x;
	temp_loc.y = cur_rob_y;
	temp_loc.theta = cur_rob_theta;
	BFS_QUEUE.push(temp_loc);

	//printf("After enqueue...\n");showQueue(BFS_QUEUE);
	//==================================================EnQueue : END
	direction[cur_rob_x][cur_rob_y] = cur_rob_theta;

	while(!BFS_QUEUE.empty())
	{
		//==================================================DeQueue : START
		temp_loc = BFS_QUEUE.front(); 
    	BFS_QUEUE.pop();

		int cur_x = temp_loc.x;		//Coordinate of the current cell
		int cur_y = temp_loc.y;
		int cur_theta = temp_loc.theta;

		//printf("After dequeue...\n");showQueue(BFS_QUEUE);
		//==================================================DeQueue : END

		if(direction[cur_x][cur_y] == cur_theta)
		{
			int nbr_x = cur_x + 1;		//Right neighbor
			int nbr_y = cur_y;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				{
					int num_of_rot = 0;		//Number of rotations
					int temp_theta = cur_theta;

					while(temp_theta != 0)		//0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
					{
						switch(temp_theta)		//Rotate the Robot
						{
							case 1:	temp_theta = 0;
									break;
							case 2:	temp_theta = 1;
									break;
							case 3:	temp_theta = 0;
									break;
						}
						
						num_of_rot++;
					}

					int temp_cost = cost_mat[cur_x][cur_y] + 1;
					int nbr_cost = cost_mat[nbr_x][nbr_y];

					if(nbr_cost > temp_cost + num_of_rot)
					{
						cost_mat[nbr_x][nbr_y] = temp_cost + num_of_rot;
						direction[nbr_x][nbr_y] = temp_theta;

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						//printf("\nAfter enqueue...\n");showQueue(BFS_QUEUE);
						//==================================================EnQueue : END
					}
				}
			}

			nbr_x = cur_x;		//Top neighbor
			nbr_y = cur_y + 1;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				{
					int num_of_rot = 0;		//Number of rotations
					int temp_theta = cur_theta;

					while(temp_theta != 1)		//0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
					{
						switch(temp_theta)		//Rotate the Robot
						{
							case 0:	temp_theta = 1;
									break;
							case 2:	temp_theta = 1;
									break;
							case 3:	temp_theta = 2;
									break;
						}
						
						num_of_rot++;
					}

					int temp_cost = cost_mat[cur_x][cur_y] + 1;
					int nbr_cost = cost_mat[nbr_x][nbr_y];

					if(nbr_cost > temp_cost + num_of_rot)
					{
						cost_mat[nbr_x][nbr_y] = temp_cost + num_of_rot;
						direction[nbr_x][nbr_y] = temp_theta;

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						//printf("\nAfter enqueue...\n");showQueue(BFS_QUEUE);
						//==================================================EnQueue : END
					}
				}
			}

			nbr_x = cur_x - 1;		//Left neighbor
			nbr_y = cur_y;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				{
					int num_of_rot = 0;		//Number of rotations
					int temp_theta = cur_theta;

					while(temp_theta != 2)		//0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
					{
						switch(temp_theta)		//Rotate the Robot
						{
							case 0:	temp_theta = 3;
									break;
							case 1:	temp_theta = 2;
									break;
							case 3:	temp_theta = 2;
									break;
						}
						
						num_of_rot++;
					}

					int temp_cost = cost_mat[cur_x][cur_y] + 1;
					int nbr_cost = cost_mat[nbr_x][nbr_y];

					if(nbr_cost > temp_cost + num_of_rot)
					{
						cost_mat[nbr_x][nbr_y] = temp_cost + num_of_rot;
						direction[nbr_x][nbr_y] = temp_theta;

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						//printf("After enqueue...\n");showQueue(BFS_QUEUE);
						//==================================================EnQueue : END
					}
				}
			}

			nbr_x = cur_x;		//Bottom neighbor
			nbr_y = cur_y - 1;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				{
					int num_of_rot = 0;		//Number of rotations
					int temp_theta = cur_theta;

					while(temp_theta != 3)		//0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
					{
						switch(temp_theta)		//Rotate the Robot
						{
							case 0:	temp_theta = 3;
									break;
							case 1:	temp_theta = 0;
									break;
							case 2:	temp_theta = 3;
									break;
						}
						
						num_of_rot++;
					}

					int temp_cost = cost_mat[cur_x][cur_y] + 1;
					int nbr_cost = cost_mat[nbr_x][nbr_y];

					if(nbr_cost > temp_cost + num_of_rot)
					{
						cost_mat[nbr_x][nbr_y] = temp_cost + num_of_rot;
						direction[nbr_x][nbr_y] = temp_theta;

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						//printf("After enqueue...\n");showQueue(BFS_QUEUE);
						//==================================================EnQueue : END
					}
				}
			}
		}
	}

	//printf("Cost Matrix...\n");showMatrix(cost_mat);printf("\n");

	return cost_mat;
}


int_mat OnDemCPP::bfs_longitudinal(int ws_size_x, int ws_size_y, bool_mat ws_graph, ondemcpp_pkg::State cur_rob_state)
{
	int cur_rob_x = cur_rob_state.x;
	int cur_rob_y = cur_rob_state.y;
	// int cur_rob_theta = cur_rob_state.theta;
	//cout << "(" << cur_rob_x << ", " << cur_rob_y << ", " << cur_rob_theta <<")";

	//int_mat color(ws_size_x, int_vec(ws_size_y, 1));		//1 : White, 2 : Grey, 3 : Black
	int_mat cost_mat(ws_size_x, int_vec(ws_size_y, COST_INF));
	//int_mat predecessor_x(ws_size_x, int_vec(ws_size_y, -1));		//Predecessor cell of a cell
	//int_mat predecessor_y(ws_size_x, int_vec(ws_size_y, -1));
	// int_mat direction(ws_size_x, int_vec(ws_size_y, -1));

	//color[cur_rob_x][cur_rob_y] = 2;
	cost_mat[cur_rob_x][cur_rob_y] = 0;

	//==================================================EnQueue : START
	queue<ondemcpp_pkg::State> BFS_QUEUE;
	ondemcpp_pkg::State temp_loc;
	temp_loc.x = cur_rob_x;
	temp_loc.y = cur_rob_y;
	// temp_loc.theta = cur_rob_theta;
	BFS_QUEUE.push(temp_loc);

	//printf("After enqueue...\n");showQueue(BFS_QUEUE);
	//==================================================EnQueue : END
	//direction[cur_rob_x][cur_rob_y] = cur_rob_theta;

	while(!BFS_QUEUE.empty())
	{
		//==================================================DeQueue : START
		temp_loc = BFS_QUEUE.front(); 
    	BFS_QUEUE.pop();

		int cur_x = temp_loc.x;		//Coordinate of the current cell
		int cur_y = temp_loc.y;
		// int cur_theta = temp_loc.theta;

		//printf("After dequeue...\n");showQueue(BFS_QUEUE);
		//==================================================DeQueue : END

		// if(direction[cur_x][cur_y] == cur_theta)
		// {
			int nbr_x = cur_x + 1;		//Right neighbor
			int nbr_y = cur_y;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				//printf("\nRight neighbor = (%d, %d).\n", nbr_x, nbr_y);

				//int nbr_clr = color[nbr_x][nbr_y];		//Color of the neighbor - 1 : White, 2 : Grey, 3 : Black

				//if((nbr_clr == 1) || (nbr_clr == 2))
				// {
					//color[nbr_x][nbr_y] = 2;

					// int num_of_rot = 0;		//Number of rotations
					// int temp_theta = cur_theta;

					// while(temp_theta != 0)		//0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
					// {
					// 	switch(temp_theta)		//Rotate the Robot
					// 	{
					// 		case 1:	temp_theta = 0;
					// 				break;
					// 		case 2:	temp_theta = 1;
					// 				break;
					// 		case 3:	temp_theta = 0;
					// 				break;
					// 	}
						
					// 	num_of_rot++;
					// }

					int temp_dist = cost_mat[cur_x][cur_y] + 1;
					int nbr_dst = cost_mat[nbr_x][nbr_y];

					// if(nbr_dst > temp_dist + num_of_rot)
					if(nbr_dst > temp_dist)
					{
						// cost_mat[nbr_x][nbr_y] = temp_dist + num_of_rot;
						cost_mat[nbr_x][nbr_y] = temp_dist;
						//predecessor_x[nbr_x][nbr_y] = cur_x;
						//predecessor_y[nbr_x][nbr_y] = cur_y;
						// direction[nbr_x][nbr_y] = temp_theta;

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						// temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						//printf("\nAfter enqueue...\n");showQueue(BFS_QUEUE);
						//==================================================EnQueue : END
					}
				// }
			}

			nbr_x = cur_x;		//Top neighbor
			nbr_y = cur_y + 1;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				//printf("\nTop neighbor = (%d, %d).\n", nbr_x, nbr_y);

				//int nbr_clr = color[nbr_x][nbr_y];		//Color of the neighbor - 1 : White, 2 : Grey, 3 : Black

				//if((nbr_clr == 1) || (nbr_clr == 2))
				// {
					//color[nbr_x][nbr_y] = 2;

					// int num_of_rot = 0;		//Number of rotations
					// int temp_theta = cur_theta;

					// while(temp_theta != 1)		//0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
					// {
					// 	switch(temp_theta)		//Rotate the Robot
					// 	{
					// 		case 0:	temp_theta = 1;
					// 				break;
					// 		case 2:	temp_theta = 1;
					// 				break;
					// 		case 3:	temp_theta = 2;
					// 				break;
					// 	}
						
					// 	num_of_rot++;
					// }

					int temp_dist = cost_mat[cur_x][cur_y] + 1;
					int nbr_dst = cost_mat[nbr_x][nbr_y];

					// if(nbr_dst > temp_dist + num_of_rot)
					if(nbr_dst > temp_dist)
					{
						// cost_mat[nbr_x][nbr_y] = temp_dist + num_of_rot;
						cost_mat[nbr_x][nbr_y] = temp_dist;
						//predecessor_x[nbr_x][nbr_y] = cur_x;
						//predecessor_y[nbr_x][nbr_y] = cur_y;
						// direction[nbr_x][nbr_y] = temp_theta;

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						// temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						//printf("\nAfter enqueue...\n");showQueue(BFS_QUEUE);
						//==================================================EnQueue : END
					}
				// }
			}

			nbr_x = cur_x - 1;		//Left neighbor
			nbr_y = cur_y;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				//printf("\nLeft neighbor = (%d, %d).\n", nbr_x, nbr_y);

				//int nbr_clr = color[nbr_x][nbr_y];		//Color of the neighbor - 1 : White, 2 : Grey, 3 : Black

				//if((nbr_clr == 1) || (nbr_clr == 2))
				// {
					//color[nbr_x][nbr_y] = 2;

					// int num_of_rot = 0;		//Number of rotations
					// int temp_theta = cur_theta;

					// while(temp_theta != 2)		//0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
					// {
					// 	switch(temp_theta)		//Rotate the Robot
					// 	{
					// 		case 0:	temp_theta = 3;
					// 				break;
					// 		case 1:	temp_theta = 2;
					// 				break;
					// 		case 3:	temp_theta = 2;
					// 				break;
					// 	}
						
					// 	num_of_rot++;
					// }

					int temp_dist = cost_mat[cur_x][cur_y] + 1;
					int nbr_dst = cost_mat[nbr_x][nbr_y];

					// if(nbr_dst > temp_dist + num_of_rot)
					if(nbr_dst > temp_dist)
					{
						// cost_mat[nbr_x][nbr_y] = temp_dist + num_of_rot;
						cost_mat[nbr_x][nbr_y] = temp_dist;
						//predecessor_x[nbr_x][nbr_y] = cur_x;
						//predecessor_y[nbr_x][nbr_y] = cur_y;
						// direction[nbr_x][nbr_y] = temp_theta;

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						// temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						//printf("After enqueue...\n");showQueue(BFS_QUEUE);
						//==================================================EnQueue : END
					}
				// }
			}

			nbr_x = cur_x;		//Bottom neighbor
			nbr_y = cur_y - 1;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				//printf("\nBottom neighbor = (%d, %d).\n", nbr_x, nbr_y);

				//int nbr_clr = color[nbr_x][nbr_y];		//Color of the neighbor - 1 : White, 2 : Grey, 3 : Black

				//if((nbr_clr == 1) || (nbr_clr == 2))
				// {
					//color[nbr_x][nbr_y] = 2;
					
					// int num_of_rot = 0;		//Number of rotations
					// int temp_theta = cur_theta;

					// while(temp_theta != 3)		//0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
					// {
					// 	switch(temp_theta)		//Rotate the Robot
					// 	{
					// 		case 0:	temp_theta = 3;
					// 				break;
					// 		case 1:	temp_theta = 0;
					// 				break;
					// 		case 2:	temp_theta = 3;
					// 				break;
					// 	}
						
					// 	num_of_rot++;
					// }

					int temp_dist = cost_mat[cur_x][cur_y] + 1;
					int nbr_dst = cost_mat[nbr_x][nbr_y];

					// if(nbr_dst > temp_dist + num_of_rot)
					if(nbr_dst > temp_dist)
					{
						// cost_mat[nbr_x][nbr_y] = temp_dist + num_of_rot;
						cost_mat[nbr_x][nbr_y] = temp_dist;
						//predecessor_x[nbr_x][nbr_y] = cur_x;
						//predecessor_y[nbr_x][nbr_y] = cur_y;
						// direction[nbr_x][nbr_y] = temp_theta;

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						// temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						//printf("After enqueue...\n");showQueue(BFS_QUEUE);
						//==================================================EnQueue : END
					}
				// }
			}

			//color[cur_x][cur_y] = 3;
		// }
	}

	//printf("Color Matrix...\n");showMatrix(color);printf("\n");
	//printf("Cost Matrix...\n");showMatrix(cost_mat);printf("\n");
	//printf("Predecessor_X Matrix...\n");showMatrix(predecessor_x);printf("\n");
	//printf("Predecessor_Y Matrix...\n");showMatrix(predecessor_y);printf("\n");

	return cost_mat;
}


int_mat OnDemCPP::compute_optimal_costs(int ws_size_x, int ws_size_y, bool_mat ws_graph, robots_states_vec robs_states, robots_states_vec goals_locs)
{
	int num_of_robs = robs_states.size();
	int num_of_goals = goals_locs.size();
	int_mat opt_cost_mat(num_of_robs, int_vec(num_of_goals, -1));		//Optimal Cost Matrix

	for(int rob_index = 0; rob_index < num_of_robs; rob_index++)
	{
		ondemcpp_pkg::State cur_rob_state = robs_states[rob_index];
		//cout << "\nR_" << rob_index << " ";

		#ifdef TURTLEBOT
			int_mat cost_mat = bfs(ws_size_x, ws_size_y, ws_graph, cur_rob_state);
		#else
			int_mat cost_mat = bfs_longitudinal(ws_size_x, ws_size_y, ws_graph, cur_rob_state);
		#endif

		for(int goal_index = 0; goal_index < num_of_goals; goal_index++)
		{
			ondemcpp_pkg::State cur_goal_loc = goals_locs[goal_index];
			opt_cost_mat[rob_index][goal_index] = cost_mat[cur_goal_loc.x][cur_goal_loc.y];
		}

		//cout << "\nR_" << rob_index << " Cost Matrix...\n";showMatrix(cost_mat);
	}

	return opt_cost_mat;
}
//==================================================1. Optimal Costs : END


//==================================================3. Optimal Paths : START
robots_states_vec OnDemCPP::optimal_path_bfs(int ws_size_x, int ws_size_y, bool_mat ws_graph, ondemcpp_pkg::State rob_loc, ondemcpp_pkg::State goal_loc)
{
	int cur_rob_x = rob_loc.x;
	int cur_rob_y = rob_loc.y;
	int cur_rob_theta = rob_loc.theta;
	//cout << "Robot (" << cur_rob_x << ", " << cur_rob_y << ")\t\t";

	int cur_goal_x = goal_loc.x;
	int cur_goal_y = goal_loc.y;
	//cout << "Goal (" << cur_goal_x << ", " << cur_goal_y << ")" << endl;

	//int_mat color(ws_size_x, int_vec(ws_size_y, 1));		//1 : White, 2 : Grey, 3 : Black
	int_mat cost_mat(ws_size_x, int_vec(ws_size_y, COST_INF));
	int_mat predecessor_x(ws_size_x, int_vec(ws_size_y, -1));		//Predecessor cell of a cell
	int_mat predecessor_y(ws_size_x, int_vec(ws_size_y, -1));
	int_mat direction(ws_size_x, int_vec(ws_size_y, -1));

	//color[cur_rob_x][cur_rob_y] = 2;
	cost_mat[cur_rob_x][cur_rob_y] = 0;

	//==================================================EnQueue : START
	queue<ondemcpp_pkg::State> BFS_QUEUE;
	BFS_QUEUE.push(rob_loc);

	//printf("\nAfter enqueue...\n");showQueue(BFS_QUEUE);
	//==================================================EnQueue : END
	direction[cur_rob_x][cur_rob_y] = cur_rob_theta;

	while(!BFS_QUEUE.empty())
	{
		//==================================================DeQueue : START
		ondemcpp_pkg::State temp_loc = BFS_QUEUE.front();
    	BFS_QUEUE.pop();

		int cur_x = temp_loc.x;		//Coordinate of the current cell
		int cur_y = temp_loc.y;
		int cur_theta = temp_loc.theta;

		//printf("\nAfter dequeue...\n");showQueue(BFS_QUEUE);
		//==================================================DeQueue : END

		if(direction[cur_x][cur_y] == cur_theta)
		{
			int nbr_x = cur_x + 1;		//Right neighbor
			int nbr_y = cur_y;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				//printf("Right neighbor = (%d, %d).\n", nbr_x, nbr_y);

				//if(color[nbr_x][nbr_y] == 1)		//White
				{
					//color[nbr_x][nbr_y] = 2;		//Grey



					int num_of_rot = 0;		//Number of rotations
					int temp_theta = cur_theta;

					while(temp_theta != 0)		//0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
					{
						switch(temp_theta)		//Rotate the Robot
						{
							case 1:	temp_theta = 0;
									break;
							case 2:	temp_theta = 1;
									break;
							case 3:	temp_theta = 0;
									break;
						}
						
						num_of_rot++;
					}

					int temp_dist = cost_mat[cur_x][cur_y] + 1;
					int nbr_dst = cost_mat[nbr_x][nbr_y];

					if(nbr_dst > temp_dist + num_of_rot)
					{
						cost_mat[nbr_x][nbr_y] = temp_dist + num_of_rot;
						predecessor_x[nbr_x][nbr_y] = cur_x;
						predecessor_y[nbr_x][nbr_y] = cur_y;
						direction[nbr_x][nbr_y] = temp_theta;

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						//printf("\nAfter enqueue1...\n");showQueue(BFS_QUEUE);
						//==================================================EnQueue : END

						//if((nbr_x == cur_goal_x) && (nbr_y == cur_goal_y))
						//	break;
					}
				}
			}

			nbr_x = cur_x;		//Top neighbor
			nbr_y = cur_y + 1;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				//printf("Top neighbor = (%d, %d).\n", nbr_x, nbr_y);

				//if(color[nbr_x][nbr_y] == 1)		//White
				{
					//color[nbr_x][nbr_y] = 2;		//Grey

					int num_of_rot = 0;		//Number of rotations
					int temp_theta = cur_theta;

					while(temp_theta != 1)		//0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
					{
						switch(temp_theta)		//Rotate the Robot
						{
							case 0:	temp_theta = 1;
									break;
							case 2:	temp_theta = 1;
									break;
							case 3:	temp_theta = 2;
									break;
						}
						
						num_of_rot++;
					}

					int temp_dist = cost_mat[cur_x][cur_y] + 1;
					int nbr_dst = cost_mat[nbr_x][nbr_y];

					if(nbr_dst > temp_dist + num_of_rot)
					{
						cost_mat[nbr_x][nbr_y] = temp_dist + num_of_rot;
						predecessor_x[nbr_x][nbr_y] = cur_x;
						predecessor_y[nbr_x][nbr_y] = cur_y;
						direction[nbr_x][nbr_y] = temp_theta;

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						//printf("\nAfter enqueue2...\n");showQueue(BFS_QUEUE);
						//==================================================EnQueue : END

						//if((nbr_x == cur_goal_x) && (nbr_y == cur_goal_y))
						//	break;
					}
				}
			}

			nbr_x = cur_x - 1;		//Left neighbor
			nbr_y = cur_y;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				//printf("Left neighbor = (%d, %d).\n", nbr_x, nbr_y);

				//if(color[nbr_x][nbr_y] == 1)		//White
				{
					//color[nbr_x][nbr_y] = 2;		//Grey					

					int num_of_rot = 0;		//Number of rotations
					int temp_theta = cur_theta;

					while(temp_theta != 2)		//0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
					{
						switch(temp_theta)		//Rotate the Robot
						{
							case 0:	temp_theta = 3;
									break;
							case 1:	temp_theta = 2;
									break;
							case 3:	temp_theta = 2;
									break;
						}
						
						num_of_rot++;
					}

					int temp_dist = cost_mat[cur_x][cur_y] + 1;
					int nbr_dst = cost_mat[nbr_x][nbr_y];

					if(nbr_dst > temp_dist + num_of_rot)
					{
						cost_mat[nbr_x][nbr_y] = temp_dist + num_of_rot;
						predecessor_x[nbr_x][nbr_y] = cur_x;
						predecessor_y[nbr_x][nbr_y] = cur_y;
						direction[nbr_x][nbr_y] = temp_theta;

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						//printf("\nAfter enqueue3...\n");showQueue(BFS_QUEUE);
						//==================================================EnQueue : END

						//if((nbr_x == cur_goal_x) && (nbr_y == cur_goal_y))
						//	break;
					}
				}
			}

			nbr_x = cur_x;		//Bottom neighbor
			nbr_y = cur_y - 1;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				//printf("Bottom neighbor = (%d, %d).\n", nbr_x, nbr_y);

				//if(color[nbr_x][nbr_y] == 1)		//White
				{
					//color[nbr_x][nbr_y] = 2;		//Grey
					
					int num_of_rot = 0;		//Number of rotations
					int temp_theta = cur_theta;

					while(temp_theta != 3)		//0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
					{
						switch(temp_theta)		//Rotate the Robot
						{
							case 0:	temp_theta = 3;
									break;
							case 1:	temp_theta = 0;
									break;
							case 2:	temp_theta = 3;
									break;
						}
						
						num_of_rot++;
					}

					int temp_dist = cost_mat[cur_x][cur_y] + 1;
					int nbr_dst = cost_mat[nbr_x][nbr_y];

					if(nbr_dst > temp_dist + num_of_rot)
					{
						cost_mat[nbr_x][nbr_y] = temp_dist + num_of_rot;
						predecessor_x[nbr_x][nbr_y] = cur_x;
						predecessor_y[nbr_x][nbr_y] = cur_y;
						direction[nbr_x][nbr_y] = temp_theta;

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						//printf("\nAfter enqueue4...\n");showQueue(BFS_QUEUE);
						//==================================================EnQueue : END

						//if((nbr_x == cur_goal_x) && (nbr_y == cur_goal_y))
						//	break;
					}
				}
			}

			//color[cur_x][cur_y] = 3;printf("Direction Matrix...\n");showMatrix(direction);printf("\n");
		}
	}

	//printf("Color Matrix...\n");showMatrix(color);printf("\n");
	//printf("Cost Matrix...\n");showMatrix(cost_mat);printf("\n");
	//printf("Predecessor_X Matrix...\n");showMatrix(predecessor_x);printf("\n");
	//printf("Predecessor_Y Matrix...\n");showMatrix(predecessor_y);printf("\n");
	//printf("Direction Matrix...\n");showMatrix(direction);printf("\n");
	
	//==================================================Backtract : START
	robots_states_vec opt_path;
	ondemcpp_pkg::State temp_loc;
	temp_loc.x = cur_goal_x;
	temp_loc.y = cur_goal_y;
	temp_loc.theta = direction[cur_goal_x][cur_goal_y];
	opt_path.push_back(temp_loc);

	int last_theta = temp_loc.theta;
	int pred_cell_x = predecessor_x[cur_goal_x][cur_goal_y];
	int pred_cell_y = predecessor_y[cur_goal_x][cur_goal_y];

	while(1)
	{
		temp_loc.x = pred_cell_x;
		temp_loc.y = pred_cell_y;
		temp_loc.theta = direction[pred_cell_x][pred_cell_y];

		if(last_theta != temp_loc.theta)		//Insert a Rotation
		{
			ondemcpp_pkg::State rot_loc;
			rot_loc.x = pred_cell_x;
			rot_loc.y = pred_cell_y;
			rot_loc.theta = last_theta;		//Single Rotation
			opt_path.push_back(rot_loc);

			if(abs(last_theta - temp_loc.theta) == 2)		//Double Rotation
			{
				rot_loc.theta = (last_theta + 1) % 4;
				opt_path.push_back(rot_loc);
			}
		}

		opt_path.push_back(temp_loc);
		last_theta = temp_loc.theta;
		
		if((pred_cell_x == cur_rob_x) && (pred_cell_y == cur_rob_y))
			break;
		else
		{
			int pred_cell_x_bkp = pred_cell_x;
			pred_cell_x = predecessor_x[pred_cell_x][pred_cell_y];
			pred_cell_y = predecessor_y[pred_cell_x_bkp][pred_cell_y];
		}
	}
	
	reverse(opt_path.begin(), opt_path.end());
	//==================================================Backtract : END

	return opt_path;
}


robots_states_vec OnDemCPP::optimal_path_bfs_longitudinal(int ws_size_x, int ws_size_y, bool_mat ws_graph, ondemcpp_pkg::State rob_loc, ondemcpp_pkg::State goal_loc)
{
	int cur_rob_x = rob_loc.x;
	int cur_rob_y = rob_loc.y;
	// int cur_rob_theta = rob_loc.theta;
	//cout << "Robot (" << cur_rob_x << ", " << cur_rob_y << ")\t\t";

	int cur_goal_x = goal_loc.x;
	int cur_goal_y = goal_loc.y;
	//cout << "Goal (" << cur_goal_x << ", " << cur_goal_y << ")" << endl;

	//int_mat color(ws_size_x, int_vec(ws_size_y, 1));		//1 : White, 2 : Grey, 3 : Black
	int_mat cost_mat(ws_size_x, int_vec(ws_size_y, COST_INF));
	int_mat predecessor_x(ws_size_x, int_vec(ws_size_y, -1));		//Predecessor cell of a cell
	int_mat predecessor_y(ws_size_x, int_vec(ws_size_y, -1));
	// int_mat direction(ws_size_x, int_vec(ws_size_y, -1));

	//color[cur_rob_x][cur_rob_y] = 2;
	cost_mat[cur_rob_x][cur_rob_y] = 0;

	//==================================================EnQueue : START
	queue<ondemcpp_pkg::State> BFS_QUEUE;
	BFS_QUEUE.push(rob_loc);
	//printf("\nAfter enqueue...\n");showQueue(BFS_QUEUE);
	//==================================================EnQueue : END
	// direction[cur_rob_x][cur_rob_y] = cur_rob_theta;

	while(!BFS_QUEUE.empty())
	{
		//==================================================DeQueue : START
		ondemcpp_pkg::State temp_loc = BFS_QUEUE.front(); 
    	BFS_QUEUE.pop();

		int cur_x = temp_loc.x;		//Coordinate of the current cell
		int cur_y = temp_loc.y;
		// int cur_theta = temp_loc.theta;

		//printf("\nAfter dequeue...\n");showQueue(BFS_QUEUE);
		//==================================================DeQueue : END

		// if(direction[cur_x][cur_y] == cur_theta)
		// {
			int nbr_x = cur_x + 1;		//Right neighbor
			int nbr_y = cur_y;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				//printf("Right neighbor = (%d, %d).\n", nbr_x, nbr_y);

				//if(color[nbr_x][nbr_y] == 1)		//White
				// {
					//color[nbr_x][nbr_y] = 2;		//Grey

					// int num_of_rot = 0;		//Number of rotations
					// int temp_theta = cur_theta;

					// while(temp_theta != 0)		//0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
					// {
					// 	switch(temp_theta)		//Rotate the Robot
					// 	{
					// 		case 1:	temp_theta = 0;
					// 				break;
					// 		case 2:	temp_theta = 1;
					// 				break;
					// 		case 3:	temp_theta = 0;
					// 				break;
					// 	}
						
					// 	num_of_rot++;
					// }

					int temp_dist = cost_mat[cur_x][cur_y] + 1;
					int nbr_dst = cost_mat[nbr_x][nbr_y];

					// if(nbr_dst > temp_dist + num_of_rot)
					if(nbr_dst > temp_dist)
					{
						// cost_mat[nbr_x][nbr_y] = temp_dist + num_of_rot;
						cost_mat[nbr_x][nbr_y] = temp_dist;
						predecessor_x[nbr_x][nbr_y] = cur_x;
						predecessor_y[nbr_x][nbr_y] = cur_y;
						// direction[nbr_x][nbr_y] = temp_theta;

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						// temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						//printf("\nAfter enqueue1...\n");showQueue(BFS_QUEUE);
						//==================================================EnQueue : END

						//if((nbr_x == cur_goal_x) && (nbr_y == cur_goal_y))
						//	break;
					}
				// }
			}

			nbr_x = cur_x;		//Top neighbor
			nbr_y = cur_y + 1;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				//printf("Top neighbor = (%d, %d).\n", nbr_x, nbr_y);

				//if(color[nbr_x][nbr_y] == 1)		//White
				// {
					//color[nbr_x][nbr_y] = 2;		//Grey

					// int num_of_rot = 0;		//Number of rotations
					// int temp_theta = cur_theta;

					// while(temp_theta != 1)		//0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
					// {
					// 	switch(temp_theta)		//Rotate the Robot
					// 	{
					// 		case 0:	temp_theta = 1;
					// 				break;
					// 		case 2:	temp_theta = 1;
					// 				break;
					// 		case 3:	temp_theta = 2;
					// 				break;
					// 	}
						
					// 	num_of_rot++;
					// }

					int temp_dist = cost_mat[cur_x][cur_y] + 1;
					int nbr_dst = cost_mat[nbr_x][nbr_y];

					// if(nbr_dst > temp_dist + num_of_rot)
					if(nbr_dst > temp_dist)
					{
						// cost_mat[nbr_x][nbr_y] = temp_dist + num_of_rot;
						cost_mat[nbr_x][nbr_y] = temp_dist;
						predecessor_x[nbr_x][nbr_y] = cur_x;
						predecessor_y[nbr_x][nbr_y] = cur_y;
						// direction[nbr_x][nbr_y] = temp_theta;

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						// temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						//printf("\nAfter enqueue2...\n");showQueue(BFS_QUEUE);
						//==================================================EnQueue : END

						//if((nbr_x == cur_goal_x) && (nbr_y == cur_goal_y))
						//	break;
					}
				// }
			}

			nbr_x = cur_x - 1;		//Left neighbor
			nbr_y = cur_y;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				//printf("Left neighbor = (%d, %d).\n", nbr_x, nbr_y);

				//if(color[nbr_x][nbr_y] == 1)		//White
				// {
					//color[nbr_x][nbr_y] = 2;		//Grey					

					// int num_of_rot = 0;		//Number of rotations
					// int temp_theta = cur_theta;

					// while(temp_theta != 2)		//0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
					// {
					// 	switch(temp_theta)		//Rotate the Robot
					// 	{
					// 		case 0:	temp_theta = 3;
					// 				break;
					// 		case 1:	temp_theta = 2;
					// 				break;
					// 		case 3:	temp_theta = 2;
					// 				break;
					// 	}
						
					// 	num_of_rot++;
					// }

					int temp_dist = cost_mat[cur_x][cur_y] + 1;
					int nbr_dst = cost_mat[nbr_x][nbr_y];

					// if(nbr_dst > temp_dist + num_of_rot)
					if(nbr_dst > temp_dist)
					{
						// cost_mat[nbr_x][nbr_y] = temp_dist + num_of_rot;
						cost_mat[nbr_x][nbr_y] = temp_dist;
						predecessor_x[nbr_x][nbr_y] = cur_x;
						predecessor_y[nbr_x][nbr_y] = cur_y;
						// direction[nbr_x][nbr_y] = temp_theta;

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						// temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						//printf("\nAfter enqueue3...\n");showQueue(BFS_QUEUE);
						//==================================================EnQueue : END

						//if((nbr_x == cur_goal_x) && (nbr_y == cur_goal_y))
						//	break;
					}
				// }
			}

			nbr_x = cur_x;		//Bottom neighbor
			nbr_y = cur_y - 1;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				//printf("Bottom neighbor = (%d, %d).\n", nbr_x, nbr_y);

				//if(color[nbr_x][nbr_y] == 1)		//White
				// {
					//color[nbr_x][nbr_y] = 2;		//Grey
					
					// int num_of_rot = 0;		//Number of rotations
					// int temp_theta = cur_theta;

					// while(temp_theta != 3)		//0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
					// {
					// 	switch(temp_theta)		//Rotate the Robot
					// 	{
					// 		case 0:	temp_theta = 3;
					// 				break;
					// 		case 1:	temp_theta = 0;
					// 				break;
					// 		case 2:	temp_theta = 3;
					// 				break;
					// 	}
						
					// 	num_of_rot++;
					// }

					int temp_dist = cost_mat[cur_x][cur_y] + 1;
					int nbr_dst = cost_mat[nbr_x][nbr_y];

					// if(nbr_dst > temp_dist + num_of_rot)
					if(nbr_dst > temp_dist)
					{
						// cost_mat[nbr_x][nbr_y] = temp_dist + num_of_rot;
						cost_mat[nbr_x][nbr_y] = temp_dist;
						predecessor_x[nbr_x][nbr_y] = cur_x;
						predecessor_y[nbr_x][nbr_y] = cur_y;
						// direction[nbr_x][nbr_y] = temp_theta;

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						// temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						//printf("\nAfter enqueue4...\n");showQueue(BFS_QUEUE);
						//==================================================EnQueue : END

						//if((nbr_x == cur_goal_x) && (nbr_y == cur_goal_y))
						//	break;
					}
				// }
			}

			//color[cur_x][cur_y] = 3;printf("Direction Matrix...\n");showMatrix(direction);printf("\n");
		// }
	}

	//printf("Color Matrix...\n");showMatrix(color);printf("\n");
	//printf("Cost Matrix...\n");showMatrix(cost_mat);printf("\n");
	//printf("Predecessor_X Matrix...\n");showMatrix(predecessor_x);printf("\n");
	//printf("Predecessor_Y Matrix...\n");showMatrix(predecessor_y);printf("\n");
	//printf("Direction Matrix...\n");showMatrix(direction);printf("\n");
	
	//==================================================Backtract : START
	robots_states_vec opt_path;
	ondemcpp_pkg::State temp_loc;
	temp_loc.x = cur_goal_x;
	temp_loc.y = cur_goal_y;
	// temp_loc.theta = direction[cur_goal_x][cur_goal_y];
	opt_path.push_back(temp_loc);

	// int last_theta = temp_loc.theta;
	int pred_cell_x = predecessor_x[cur_goal_x][cur_goal_y];
	int pred_cell_y = predecessor_y[cur_goal_x][cur_goal_y];

	while(1)
	{
		temp_loc.x = pred_cell_x;
		temp_loc.y = pred_cell_y;
		// temp_loc.theta = direction[pred_cell_x][pred_cell_y];

		// if(last_theta != temp_loc.theta)		//Insert a Rotation
		// {
		// 	struct loc rot_loc;
		// 	rot_loc.x = pred_cell_x;
		// 	rot_loc.y = pred_cell_y;
		// 	rot_loc.theta = last_theta;		//Single Rotation
		// 	opt_path.push_back(rot_loc);

		// 	if(abs(last_theta - temp_loc.theta) == 2)		//Double Rotation
		// 	{
		// 		rot_loc.theta = (last_theta + 1) % 4;
		// 		opt_path.push_back(rot_loc);
		// 	}
		// }

		opt_path.push_back(temp_loc);
		// last_theta = temp_loc.theta;
		
		if((pred_cell_x == cur_rob_x) && (pred_cell_y == cur_rob_y))
			break;
		else
		{
			int pred_cell_x_bkp = pred_cell_x;
			pred_cell_x = predecessor_x[pred_cell_x][pred_cell_y];
			pred_cell_y = predecessor_y[pred_cell_x_bkp][pred_cell_y];
		}
	}
	
	reverse(opt_path.begin(), opt_path.end());
	//==================================================Backtract : END

	return opt_path;
}


robots_states_mat OnDemCPP::compute_optimal_paths(bool_mat ws_graph, robots_states_vec robs_states, robots_states_vec goals_locs, int_vec opt_goal_vec, bool_vec S_r, robots_states_mat paths_old, uint &active_count)
{
	uint ws_size_x = ws_graph.size();			// Size of the workspace
	uint ws_size_y = ws_graph[0].size();		// Size of the workspace
	uint rob_count = S_r.size();		// Total number of robots
	uint rob_index = 0;
	robots_states_mat opt_paths;		//Optimal Paths

	for(uint rob_id = 0; rob_id < rob_count; rob_id++)
	{
		robots_states_vec opt_path;
		
		if(S_r[rob_id])
		{
			ondemcpp_pkg::State rob_state = robs_states[rob_index];
			int opt_goal_index = opt_goal_vec[rob_index];

			if(opt_goal_index == -1)		//Hasn't been assigned a goal
				opt_path.push_back(rob_state);
			else
			{
				active_count++;
				ondemcpp_pkg::State goal_loc = goals_locs[opt_goal_index];

				#ifdef TURTLEBOT
					opt_path = optimal_path_bfs(ws_size_x, ws_size_y, ws_graph, rob_state, goal_loc);
				#else
					opt_path = optimal_path_bfs_longitudinal(ws_size_x, ws_size_y, ws_graph, rob_state, goal_loc);
				#endif
			}

			rob_index++;
		}
		else
		{
			uint path_len = paths_old[rob_id].size() - 1;

			for(uint state_id = 0; state_id <= path_len; state_id++)
				opt_path.push_back(paths_old[rob_id][state_id]);
		}

		opt_paths.push_back(opt_path);
	}

	return opt_paths;
}


bool OnDemCPP::is_type1_crossover_path_pair(int rob_1, int rob_2, ondemcpp_pkg::State rob1_start_loc, robots_states_vec path_2)
{
	int path2_len = path_2.size() - 1;		// Length of path_2

	if(path2_len > 1)
	{
		for(int i = 1; i < path2_len; i++)
		{
			ondemcpp_pkg::State path2_loc = path_2[i];

			if((path2_loc.x == rob1_start_loc.x) && (path2_loc.y == rob1_start_loc.y))
			{
				cout << "\nR_" << rob_1 << " ! R_" << rob_2;
				return true;
			}
		}

		return false;
	}
	else
		return false;
}


bool OnDemCPP::is_type2_crossover_path_pair(int rob_1, int rob_2, robots_states_vec path_1, robots_states_vec path_2)
{
	int path1_len = path_1.size() - 1;		// Length of path_1
	int path2_len = path_2.size() - 1;		// Length of path_2

	if((path1_len > 1) && (path2_len > 1))
	{
		ondemcpp_pkg::State path1_start_loc = path_1[0];
		ondemcpp_pkg::State path2_start_loc = path_2[0];
		bool flag_start1_found, flag_start2_found;
		flag_start1_found = flag_start2_found = false;

		for(int i = 1; i < path1_len; i++)
		{
			ondemcpp_pkg::State path1_loc = path_1[i];

			if((path1_loc.x == path2_start_loc.x) && (path1_loc.y == path2_start_loc.y))
			{
				flag_start2_found = true;
				break;
			}
		}

		for(int i = 1; i < path2_len; i++)
		{
			ondemcpp_pkg::State path2_loc = path_2[i];

			if((path2_loc.x == path1_start_loc.x) && (path2_loc.y == path1_start_loc.y))
			{
				flag_start1_found = true;
				break;
			}
		}

		if(flag_start1_found && flag_start2_found)
		{
			cout << "\nP_" << rob_1 << " x P_" << rob_2;
			return true;
		}
		else
			return false;
	}
	else
		return false;
}


bool OnDemCPP::is_nested_path_pair(int rob_1, int rob_2, robots_states_vec path_1, robots_states_vec path_2)
{
	int path_1_len = path_1.size() - 1;		// Length of path_1
	int path_2_len = path_2.size() - 1;		// Length of path_2
	ondemcpp_pkg::State path_1_start_loc = path_1[0];
	ondemcpp_pkg::State path_1_goal_loc = path_1[path_1_len];
	bool flag_start1_found, flag_goal1_found, r1_in_r2;
	flag_start1_found = flag_goal1_found = r1_in_r2 = false;

	for(int i = 1; i < path_2_len; i++)		// Is R_1 nested in R_2?
	{
		ondemcpp_pkg::State path_2_loc = path_2[i];

		if(!flag_start1_found && (path_1_start_loc.x == path_2_loc.x) && (path_1_start_loc.y == path_2_loc.y))
			flag_start1_found = true;
		else if(!flag_goal1_found && (path_1_goal_loc.x == path_2_loc.x) && (path_1_goal_loc.y == path_2_loc.y))
			flag_goal1_found = true;

		r1_in_r2 = flag_start1_found && flag_goal1_found;

		if(r1_in_r2)
		{
			cout << "\nR_" << rob_2 << " -> R_" << rob_1;
			return true;
		}
	}

	return false;
}


robots_states_vec OnDemCPP::adjust_path(ondemcpp_pkg::State init_state, robots_states_vec opt_path)
{
	robots_states_vec path;
	path.push_back(init_state);
	uint path_len = opt_path.size() - 1;
	ondemcpp_pkg::State opt_path_loc;
	uint k = path_len - 1;

	do
	{
		opt_path_loc = opt_path[k];

		if((init_state.x == opt_path_loc.x) && (init_state.y == opt_path_loc.y))
			break;

		k--;
	}
	while(k > 0);

	opt_path_loc = opt_path[++k];

	if(init_state.theta != opt_path_loc.theta)			// Needs Rotations
	{
		uint tmp_theta = (uint(init_state.theta) + 1) % 4;
		uint lr_count = 1;			// Left rotation counter

		while(tmp_theta != opt_path_loc.theta)
		{
			tmp_theta = (tmp_theta + 1) % 4;
			lr_count++;
		}

		uint rr_count = 4 - lr_count;			// Right rotation counter
		uint r_count = 1;		// Rotation counter

		if(lr_count <= rr_count)		// Rotate left
		{
			tmp_theta = init_state.theta;

			while(r_count <= lr_count)
			{
				tmp_theta = (tmp_theta + 1) % 4;
				init_state.theta = tmp_theta;
				path.push_back(init_state);
				r_count++;
			}
		}
		else
		{
			tmp_theta = init_state.theta;

			while(r_count <= rr_count)
			{
				tmp_theta = (tmp_theta - 1 + 4) % 4;
				init_state.theta = tmp_theta;
				path.push_back(init_state);
				r_count++;
			}
		}
	}

	for(uint l = k; l <= path_len; l++)
		path.push_back(opt_path[l]);

	return path;
}


bool OnDemCPP::test_path(int nearest_rob_id, robots_states_vec nearest_rob_path, int_vec goal_vec_new, robots_states_mat feasible_paths, bool_vec S_r)
{
	// int num_of_robs = goal_vec_new.size();
	int rob_count = S_r.size();

	for(int rob_id = 0; rob_id < rob_count; rob_id++)
		if(nearest_rob_id != rob_id)
		{
			robots_states_vec rob_path = feasible_paths[rob_id];

			if(S_r[rob_id])		// Requester
			{
				int rob_id_req = -1;

				for(uint rob_id2 = 0; rob_id2 <= rob_id; rob_id2++)
					if(S_r[rob_id2])
						rob_id_req++;

				if((goal_vec_new[rob_id_req] != -1) && 
					(is_nested_path_pair(nearest_rob_id, rob_id, nearest_rob_path, rob_path) || 
					is_nested_path_pair(rob_id, nearest_rob_id, rob_path, nearest_rob_path)))
					return false;
			}
			else		// Non-requester
			{
				if(rob_path.size() - 1)			// Non-requester Active
				{
					if(is_nested_path_pair(rob_id, nearest_rob_id, rob_path, nearest_rob_path))
						return false;
				}
				else if(is_type1_crossover_path_pair(rob_id, nearest_rob_id, rob_path[0], nearest_rob_path))		// Non-requester Inactive
					return false;
			}
		}

	return true;
}


robots_states_mat OnDemCPP::get_feasible_paths(int_vec &opt_goal_vec, robots_states_mat opt_paths, uint &killed_count, uint &revived_count, uint &revived_goal_count, int ws_size_x, int ws_size_y, bool_vec S_r)
{
	//================================================== Detection : START
	// int num_of_robs = opt_goal_vec.size();		// Number of robots
	int rob_count = S_r.size();						// Total number of robots
	bool_vec is_killed_rob(rob_count, false);		// Is R_i killed?
	int_vec killed_robs;							// The set of killed robots
	// cout << "\nKilled";

	for(int i = 0; i < rob_count; i++)		// Type-2 crossover path and Nested path
	{
		int i_req = -1;

		if(S_r[i])
			for(uint k = 0; k <= i; k++)
				if(S_r[k])
					i_req++;

		for(int j = 0; j < rob_count; j++)
		{
			int j_req = -1;

			if(S_r[j])
				for(uint k = 0; k <= j; k++)
					if(S_r[k])
						j_req++;

			if((i != j) && ((S_r[i] && (opt_goal_vec[i_req] != -1)) || (!S_r[i] && (opt_paths[i].size() - 1))) && (S_r[j] && opt_goal_vec[j_req] != -1) && 
				(!is_killed_rob[i] || !is_killed_rob[j]) && 
				((S_r[i] && S_r[j] && is_type2_crossover_path_pair(i, j, opt_paths[i], opt_paths[j])) || 
					(((!S_r[i] && S_r[j]) || (S_r[i] && S_r[j])) && is_nested_path_pair(i, j, opt_paths[i], opt_paths[j]))))
			{
				if(S_r[i] && !is_killed_rob[i])
				{
					is_killed_rob[i] = true;
					killed_robs.push_back(i);
					killed_count++;
					// cout << " R_" << i;
				}

				if(S_r[j] && !is_killed_rob[j])
				{
					is_killed_rob[j] = true;
					killed_robs.push_back(j);
					killed_count++;
					// cout << " R_" << j;
				}
			}
		}
	}

	for(int i = 0; i < rob_count; i++)		// Adds inactive robots
	{
		int i_req = -1;

		if(S_r[i])
			for(uint k = 0; k <= i; k++)
				if(S_r[k])
					i_req++;

		if((S_r[i] && (opt_goal_vec[i_req] == -1)) || (!S_r[i] && !(opt_paths[i].size() - 1)))
			killed_robs.push_back(i);
	}
	
	while(!killed_robs.empty())
	{
		int i = killed_robs.back();
		killed_robs.pop_back();

		for(int j = 0; j < rob_count; j++)
			if(S_r[j])
			{
				int j_req = -1;

				for(uint k = 0; k <= j; k++)
					if(S_r[k])
						j_req++;

				if((i != j) && (opt_goal_vec[j_req] != -1) && !is_killed_rob[j] && is_type1_crossover_path_pair(i, j, opt_paths[i][0], opt_paths[j]))
				{
					is_killed_rob[j] = true;
					killed_robs.push_back(j);
					killed_count++;
					// cout << " R_" << j;
				}
			}
	}

	// cout << "\n\nKilled...";

	// for(int i = 0; i < rob_count; i++)
	// 	if(is_killed_rob[i])
	// 		cout << " R_" << i;
	//================================================== Detection : END
	//================================================== Correction : START
	int_mat W_r(ws_size_x, int_vec(ws_size_y, -1));						// Workspace with Inactive/Killed Robot IDs
	bool_mat W_killed_goals(ws_size_x, bool_vec(ws_size_y, false));		// Goals of killed robots
	// int_vec goal_vec_new(rob_count, -1);									// Goals of Inactive/Killed robots
	int_vec goal_vec_new(opt_goal_vec.size(), -1);							// Goals of Inactive/Killed robots
	robots_states_mat feasible_paths;												// The set of feasible paths

	for(int i = 0; i < rob_count; i++)		// Initialization
	{
		robots_states_vec path;

		if(S_r[i])
		{
			int i_req = -1;

			for(uint i2 = 0; i2 <= i; i2++)
				if(S_r[i2])
					i_req++;

			if((opt_goal_vec[i_req] != -1) && !is_killed_rob[i])		// Active unkilled
				path = opt_paths[i];
			else
			{
				ondemcpp_pkg::State start_state = opt_paths[i][0];
				W_r[(int)start_state.x][(int)start_state.y] = i;
				path.push_back(start_state);

				if(is_killed_rob[i])
				{
					int path_len = opt_paths[i].size() - 1;
					ondemcpp_pkg::State goal_state = opt_paths[i][path_len];
					W_killed_goals[(int)goal_state.x][(int)goal_state.y] = true;		// Needs to be visited
				}
			}
		}
		else
			path = opt_paths[i];

		feasible_paths.push_back(path);
	}

	for(int i = 0; i < rob_count; i++)
		if(S_r[i] && is_killed_rob[i])
		{
			int i_req = -1;

			for(uint i2 = 0; i2 <= i; i2++)
				if(S_r[i2])
					i_req++;

			// cout << "\nTry reassigning G_" << opt_goal_vec[i] << " (of R_" << i;
			robots_states_vec path_i = opt_paths[i];
			int path_i_len = path_i.size() - 1;

			for(int loc_id = path_i_len - 1; loc_id >= 0; loc_id--)
			{
				ondemcpp_pkg::State path_i_loc = path_i[loc_id];
				int nearest_rob = W_r[(int)path_i_loc.x][(int)path_i_loc.y];		// The nearest robot to the goal opt_goal_vec[i]

				if(nearest_rob != -1)
				{
					// cout << ") to R_" << nearest_rob;
					robots_states_vec path;

					if(nearest_rob == i)
						path = path_i;
					else
						path = adjust_path(opt_paths[nearest_rob][0], path_i);

					if(test_path(nearest_rob, path, goal_vec_new, feasible_paths, S_r))
					{
						W_r[(int)path_i_loc.x][(int)path_i_loc.y] = -1;		// Revived
						int nearest_rob_req = -1;

						for(uint j = 0; j <= nearest_rob; j++)
							if(S_r[j])
								nearest_rob_req++;

						// goal_vec_new[nearest_rob] = opt_goal_vec[i];
						goal_vec_new[nearest_rob_req] = opt_goal_vec[i_req];
						
						int path_len = path.size() - 1;

						for(int loc_id = 1; loc_id <= path_len; loc_id++)
						{
							ondemcpp_pkg::State path_state = path[loc_id];
							feasible_paths[nearest_rob].push_back(path_state);
							W_killed_goals[(int)path_state.x][(int)path_state.y] = false;		// To be visited
						}

						revived_count++;
						// cout << ": OK";
					}

					break;
				}
			}
		}
	//================================================== Correction : END
	// cout << "\n\nRevived...";

	for(int i = 0; i < rob_count; i++)
		if(S_r[i])
		{
			int i_req = -1;

			for(uint i2 = 0; i2 <= i; i2++)
				if(S_r[i2])
					i_req++;
			
			if((opt_goal_vec[i_req] == -1) || is_killed_rob[i])
			{
				int goal_id_new = goal_vec_new[i_req];
				opt_goal_vec[i_req] = goal_id_new;

				// if(goal_id_new != -1)
					// cout << " R_" << i;
			}
		}

	revived_goal_count = killed_count;

	for(uint i = 0; i < ws_size_x; i++)
		for(uint j = 0; j < ws_size_y; j++)
			if(W_killed_goals[i][j])
				revived_goal_count--;

	return feasible_paths;
}
//==================================================3. Optimal Paths : END


//==================================================4. Partial Order : START
bool_mat OnDemCPP::compute_partial_orders(int num_of_robs, robots_states_mat paths, bool_vec S_r)
{
	bool_mat partial_order(num_of_robs, bool_vec(num_of_robs, false));		// Initialized to false

	for(uint rob_id1 = 0; rob_id1 < num_of_robs; rob_id1++)
	{
		ondemcpp_pkg::State start_state_1 = paths[rob_id1][0];
		uint path_len_1 = paths[rob_id1].size() - 1;
		ondemcpp_pkg::State goal_state_1 = paths[rob_id1][path_len_1];

		for(uint rob_id2 = 0; rob_id2 < num_of_robs; rob_id2++)
			if((rob_id1 != rob_id2) && (S_r[rob_id1] || S_r[rob_id2]))
			{
				uint path_len_2 = paths[rob_id2].size() - 1;

				if(path_len_2)		// Robot 2 has been assigned a goal
					for(uint state_id2 = 0; state_id2 <= path_len_2; state_id2++)
					{
						ondemcpp_pkg::State state2 = paths[rob_id2][state_id2];

						if((start_state_1.x == state2.x) && (start_state_1.y == state2.y))		// Check if S_2..........S_1..........G_2
						{
							partial_order[rob_id1][rob_id2] = true;
							cout << "\nS_" << rob_id1 << " > " << "P_" << rob_id2;
						}

						if((goal_state_1.x == state2.x) && (goal_state_1.y == state2.y))		// Check if S_2..........G_1..........G_2
						{
							partial_order[rob_id2][rob_id1] = true;
							cout << "\nG_" << rob_id1 << " > " << "P_" << rob_id2;
						}
					}
			}
	}

	return partial_order;
}
//==================================================4. Partial Order : END


//==================================================5. Total Order : START
int_vec OnDemCPP::compute_total_order(int num_of_robs, bool_mat adj, bool_mat &adj_residue)
{
	int_vec in_degrees(num_of_robs, 0);
	bool_vec visited(num_of_robs, false);
	queue<int> Q;

	for(int i = 0; i < num_of_robs; i++)
	{
		for(int j = 0; j < num_of_robs; j++)
		{
			if(adj[i][j])
			{
				in_degrees[j]++;
			}
		}
	}

	//cout << "\nIn-degrees...\n";

	for(int i = 0; i < num_of_robs; i++)
	{
		//cout << in_degrees[i] << " ";

		if(!in_degrees[i])
		{
			Q.push(i);
			//cout << "Pushed R_" << i << " ";
			visited[i] = true;
		}
	}

	cout << endl;
	//int_vec total_order;
	int_vec total_order(num_of_robs, -1);		//Initialized with invalid robot id (-1)
	uint to_index = 0;

	while(!Q.empty()) 
    { 
        int robot_index = Q.front(); 
        Q.pop();
        //cout << "\nPopped R_" << robot_index << endl;
        
        //total_order.push_back(robot_index);
        total_order[to_index++] = robot_index;

        for(int j = 0; j < num_of_robs; j++)
        {
        	if(adj[robot_index][j] && !visited[j])
        	{
        		in_degrees[j]--;
        		adj_residue[robot_index][j] = false;		//Traversed

        		if(!in_degrees[j])
				{
					Q.push(j);
					//cout << "Pushed R_" << j << " ";
					visited[j] = true;
				}
        	}
        }

        /*cout << "\n\nIn-degrees...\n";

		for(int i = 0; i < num_of_robs; i++)
		{
			cout << in_degrees[i] << " ";
		}

		cout << endl;*/
    }

	return total_order;
}


void OnDemCPP::break_dependency_cycles(int_vec &opt_goal_vec, bool_mat po_residue, bool_vec S_r)
{
	int num_of_robs = opt_goal_vec.size();
	uint rob_count = po_residue[0].size();

	//================================================== Directed Cyclic Graph (DCG) to Directed Acyclic Graph (DAG) : START
	int_vec opt_goal_vec_l, opt_goal_vec_r;		// Left & Right Vectors of opt_goal_vec
	opt_goal_vec_l = opt_goal_vec_r = opt_goal_vec;

	uint count_l, count_r;
	count_l = count_r = 0;

	int i_index, j_index;
	i_index = -1;

	for(int i = 0; i < rob_count; i++)
	{
		if(S_r[i])
			i_index++;

		j_index = -1;

		for(int j = 0; j < rob_count; j++)
		{
			if(S_r[j])
				j_index++;

			if(po_residue[i][j])
			{
				// cout << "po_residue[" << i << "][" << j << "] i_index = " << i_index << " j_index = " << j_index;

				if(i < j)
				{
					// cout << " Left\n";

					if(S_r[i] && (opt_goal_vec_r[i_index] != -1))
					{
						opt_goal_vec_r[i_index] = -1;		//Goal Unassigned
						count_l++;
					}
					else if(S_r[j] && (opt_goal_vec_r[j_index] != -1))
					{
						opt_goal_vec_r[j_index] = -1;		//Goal Unassigned
						count_l++;
					}
				}
				else if(i > j)
				{
					// cout << " Right\n";

					if(S_r[i] && (opt_goal_vec_l[i_index] != -1))
					{
						opt_goal_vec_l[i_index] = -1;		//Goal Unassigned
						count_r++;
					}
					else if(S_r[j] && (opt_goal_vec_l[j_index] != -1))
					{
						opt_goal_vec_l[j_index] = -1;		//Goal Unassigned
						count_r++;
					}
				}
			}
		}
	}

	/*cout << "\nOG:\t\t";
	for(uint i = 0; i < num_of_robs; i++)
		cout << opt_goal_vec[i] << " ";

	cout << "count_l = " << count_l << " count_r = " << count_r;*/

	if(count_l && count_r)
	{
		if(count_l >= count_r)
			opt_goal_vec = opt_goal_vec_l;
		else
			opt_goal_vec = opt_goal_vec_r;
	}
	else if(count_l)
		opt_goal_vec = opt_goal_vec_r;
	else if(count_r)
		opt_goal_vec = opt_goal_vec_l;

	/*cout << "\nLeft:\t";
	for(uint i = 0; i < num_of_robs; i++)
		cout << opt_goal_vec_l[i] << " ";
	
	cout << "\nRight:\t";
	for(uint i = 0; i < num_of_robs; i++)
		cout << opt_goal_vec_r[i] << " ";*/
	//================================================== Directed Cyclic Graph (DCG) to Directed Acyclic Graph (DAG) : END
}


void OnDemCPP::adjust_dependent_paths(int_vec opt_goal_vec, robots_states_mat &paths, bool_vec S_r, uint &active_count)
{
	uint rob_count = S_r.size();

	for(uint rob_id = 0; rob_id < rob_count; rob_id++)
	{
		uint path_len = paths[rob_id].size() - 1;

		if(S_r[rob_id])
		{
			int req_rob_id = -1;

			for(uint rob_id_tmp = 0; rob_id_tmp <= rob_id; rob_id_tmp++)
				if(S_r[rob_id_tmp])
					req_rob_id++;
			
			if(opt_goal_vec[req_rob_id] == -1)
			{
				active_count--;

				for(uint state_id = path_len; state_id > 0; state_id--)
					paths[rob_id].erase(paths[rob_id].begin() + state_id);
			}
		}
		else
			active_count--;
	}
}
//==================================================5. Total Order : END


//==================================================6. Timeoffsets : START
int_vec OnDemCPP::compute_start_time_offsets(int_vec total_order, robots_states_mat paths, bool_vec S_r, int_vec &opt_goal_vec, bool &flag_inactivated_robot)
{
	int rob_count = total_order.size();
	int_vec sto_vec(rob_count, 0);		// Initialized to 0
	int to_index = 1;

	while(to_index < rob_count)
	{
		int rob_id = total_order[to_index];
		int sto_rob_id = 0;
		int path_len_rob_id = paths[rob_id].size() - 1;
		int pred_index = to_index - 1;

		while(pred_index >= 0)
		{
			int pred_id = total_order[pred_index];		// Total order predecessor robot index
			int sto_pred_id = sto_vec[pred_id];
			int path_len_pred_id = paths[pred_id].size() - 1;
			bool flag_scc, flag_hoc;
			flag_scc = flag_hoc = false;

			//================================================== Same Cell Collision (SCC) : START
			int time_max = sto_rob_id + path_len_rob_id;

			if(time_max < (sto_pred_id + path_len_pred_id))
				time_max = sto_pred_id + path_len_pred_id;

			ondemcpp_pkg::State cell_rob_id, cell_pred_id;

			for(int time = 0; time <= time_max; time++)
			{
				if(time <= sto_rob_id)
					cell_rob_id = paths[rob_id][0];
				else if(time <= (sto_rob_id + path_len_rob_id))
					cell_rob_id = paths[rob_id][time - sto_rob_id];
				else
					cell_rob_id = paths[rob_id][path_len_rob_id];

				if(time <= sto_pred_id)
					cell_pred_id = paths[pred_id][0];
				else if(time <= (sto_pred_id + path_len_pred_id))
					cell_pred_id = paths[pred_id][time - sto_pred_id];
				else
					cell_pred_id = paths[pred_id][path_len_pred_id];

				if((cell_rob_id.x == cell_pred_id.x) && (cell_rob_id.y == cell_pred_id.y))		// Same Cell Collision
				{
					flag_scc = true;
					break;
				}
			}
			//================================================== Same Cell Collision (SCC) : END
			//================================================== Head-on Collision (HoC) : START
			int time_min = sto_rob_id + path_len_rob_id;

			if(time_min > (sto_pred_id + path_len_pred_id))
				time_min = sto_pred_id + path_len_pred_id;

			ondemcpp_pkg::State prev_cell_rob_id, prev_cell_pred_id;
			prev_cell_rob_id = paths[rob_id][0];
			prev_cell_pred_id = paths[pred_id][0];

			for(int time = 1; time <= time_min; time++)
			{
				if(time <= sto_rob_id)
					cell_rob_id = paths[rob_id][0];
				else if(time <= (sto_rob_id + path_len_rob_id))
					cell_rob_id = paths[rob_id][time - sto_rob_id];

				if(time <= sto_pred_id)
					cell_pred_id = paths[pred_id][0];
				else if(time <= (sto_pred_id + path_len_pred_id))
					cell_pred_id = paths[pred_id][time - sto_pred_id];

				if((cell_rob_id.x == prev_cell_pred_id.x) && (cell_rob_id.y == prev_cell_pred_id.y) && (cell_pred_id.x == prev_cell_rob_id.x) && (cell_pred_id.y == prev_cell_rob_id.y))		// Head-on Collision
				{
					flag_hoc = true;
					break;
				}

				prev_cell_rob_id = cell_rob_id;
				prev_cell_pred_id = cell_pred_id;
			}
			//================================================== Head-on Collision (HoC) : END
			
			if(flag_scc || flag_hoc)
			{
				if(S_r[rob_id])
				{
					sto_rob_id++;
					pred_index = to_index - 1;
				}
				else if(S_r[pred_id])
				{
					int pred_id2 = -1;

					for(uint pred_id_tmp = 0; pred_id_tmp <= pred_id; pred_id_tmp++)
						if(S_r[pred_id_tmp])
							pred_id2++;
					
					opt_goal_vec[pred_id2] = -1;
					printf("\nInactivated R_%d for R_%d", pred_id, rob_id);
					flag_inactivated_robot = true;
					return sto_vec;
				}
			}
			else
				pred_index--;
		}
		
		sto_vec[rob_id] = sto_rob_id;
		to_index++;
	}

	return sto_vec;
}
//==================================================6. Timeoffsets : END


//==================================================7. Optimal Trajectories : START
robots_states_mat OnDemCPP::compute_optimal_trajectories(int num_of_robs, int_vec start_time_offsets, robots_states_mat paths, bool_vec S_r, uint hor_id)
{
	uint rob_count = S_r.size();
	//================================================== Save : START
	string ondemcpp_pkg_path = package::getPath("ondemcpp_pkg");
	string capl_file_name;
	capl_file_name = ondemcpp_pkg_path + OUTPUT_DIR + boost::lexical_cast<string>(hor_id) + OPTIMAL_PATH_LENGTHS_FILE;	
	ofstream capl_file;			// Collision Averted Path Lengths File
	capl_file.open(capl_file_name.c_str(), ios::out);

	for(uint rob_id = 0; rob_id < rob_count; rob_id++)
		if(S_r[rob_id])
		{
			int sum_of_path_len_and_time_offset = (paths[rob_id].size() - 1) + start_time_offsets[rob_id];
			capl_file << rob_id << ":" << (paths[rob_id].size() - 1) << "," << start_time_offsets[rob_id] << "," << sum_of_path_len_and_time_offset << endl;
		}

	capl_file.close();
	//================================================== Save : END
	//================================================== Paths : START
	robots_states_mat trajectories(rob_count, robots_states_vec());
	ondemcpp_pkg::State robot_loc;

	for(int robot_index = 0; robot_index < rob_count; robot_index++)
	{
		int time_offset_robot_index;

		if(S_r[robot_index])
			time_offset_robot_index = start_time_offsets[robot_index];
		else
			time_offset_robot_index = 0;

		robots_states_vec path_robot_index = paths[robot_index];
		int path_len_robot_index = path_robot_index.size() - 1;

		robot_loc.x = path_robot_index[0].x;
		robot_loc.y = path_robot_index[0].y;
		robot_loc.theta = path_robot_index[0].theta;
		trajectories[robot_index].push_back(robot_loc);

		for(int time = 0; time < (time_offset_robot_index + path_len_robot_index); time++)
		{
			if(time >= time_offset_robot_index)
			{
				robot_loc.x = path_robot_index[time - time_offset_robot_index + 1].x;
				robot_loc.y = path_robot_index[time - time_offset_robot_index + 1].y;
				robot_loc.theta = path_robot_index[time - time_offset_robot_index + 1].theta;
			}
			else		// Append initial location
			{
				robot_loc.x = path_robot_index[0].x;
				robot_loc.y = path_robot_index[0].y;
				robot_loc.theta = path_robot_index[0].theta;
			}

			trajectories[robot_index].push_back(robot_loc);
		}
	}
	//================================================== Paths : END

	return trajectories;
}


void OnDemCPP::monitor_paths(int ws_size_x, int ws_size_y, int num_of_robs, robots_states_mat trajectories)
{
	int_vec path_len_vec(num_of_robs, -1);
	int hor_len = trajectories[0].size() - 1;		// Horizon Length
	path_len_vec[0] = hor_len;

	for(int rob_id = 1; rob_id < num_of_robs; rob_id++)
	{
		int path_len = trajectories[rob_id].size() - 1;
		path_len_vec[rob_id] = path_len;

		if(hor_len < path_len)
			hor_len = path_len;
	}

	//================================================== Detect Same Cell Collsion : START
	for(int time = 0; time <= hor_len; time++)
	{
		int_mat ws_cells(ws_size_x, int_vec(ws_size_y, -1));
		
		for(int rob_id = 0; rob_id < num_of_robs; rob_id++)
		{
			int path_len = path_len_vec[rob_id];
			ondemcpp_pkg::State rob_loc;

			if(time <= path_len)
				rob_loc = trajectories[rob_id][time];
			else
				rob_loc = trajectories[rob_id][path_len];

			if(ws_cells[rob_loc.x][rob_loc.y] != -1)		// Same Cell Collision
			{
				cout << "\nMonitor failed! SCC R_" << rob_id << " <> R_" << ws_cells[rob_loc.x][rob_loc.y] << " (" << rob_loc.x << ", " << rob_loc.y << ") @ t = " << time << "\n";
				exit(1);
			}
			else
				ws_cells[rob_loc.x][rob_loc.y] = rob_id;
		}
	}
	//================================================== Detect Same Cell Collsion : END
	//================================================== Detect Head-on Collsion : START
	for(int time = 1; time <= hor_len; time++)
		for(int rob_id1 = 0; rob_id1 < num_of_robs - 1; rob_id1++)
			for(int rob_id2 = rob_id1 + 1; rob_id2 < num_of_robs; rob_id2++)
			{
				int path1_len = path_len_vec[rob_id1];
				int path2_len = path_len_vec[rob_id2];
				ondemcpp_pkg::State cell_1, cell_2, cell_1_prev, cell_2_prev;

				if(time <= path1_len)
					cell_1 = trajectories[rob_id1][time];
				else
					cell_1 = trajectories[rob_id1][path1_len];

				if(time <= path2_len)
					cell_2 = trajectories[rob_id2][time];
				else
					cell_2 = trajectories[rob_id2][path2_len];

				if(time - 1 <= path1_len)
					cell_1_prev = trajectories[rob_id1][time - 1];
				else
					cell_1_prev = trajectories[rob_id1][path1_len];

				if(time - 1 <= path2_len)
					cell_2_prev = trajectories[rob_id2][time - 1];
				else
					cell_2_prev = trajectories[rob_id2][path2_len];

				if((cell_1.x == cell_2_prev.x) && (cell_1.y == cell_2_prev.y) && (cell_2.x == cell_1_prev.x) && (cell_2.y == cell_1_prev.y))		// Head-on Collision
				{
					cout << "\nMonitor failed! HoC R_" << rob_id1 << " <> R_" << rob_id2 << " (" << cell_1.x << ", " << cell_1.y << ") <> (" << cell_2.x << ", " << cell_2.y << ") @ t = " << time << "\n";
					exit(1);
				}
			}
	//================================================== Detect Head-on Collsion : END
}
//==================================================7. Optimal Trajectories : END


robots_states_mat OnDemCPP::runOnDemCPP(int ws_size_x, int ws_size_y, bool_mat ws_graph, int num_of_robs, robots_states_vec robs_states, int num_of_goals, robots_states_vec goals_locs, uint hor_id, bool_vec S_r, robots_states_mat paths_old)
{
	uint rob_count = S_r.size();		// Total number of robots

	#ifdef DEBUG_WS_GRAPH
		cout << "\nWS_Graph...\n";

		for(uint row_id = 0; row_id < ws_size_x; row_id++)
	    {
	    	for(uint col_id = 0; col_id < ws_size_y; col_id++)
	    		cout << ws_graph[row_id][col_id] << " ";

	    	cout << endl;
	    }
	#endif

	#ifdef DEBUG_ROBS_STATES
		cout << "\nRobot states...";
		uint rob_id = 0;

		for(uint i = 0; i < rob_count; i++)
		{
			if(S_r[i])
			{
				ondemcpp_pkg::State rob_loc = robs_states[rob_id];
				cout << "\nR_" << rob_id << " (" << rob_loc.x << ", " << rob_loc.y << ", " << rob_loc.theta << "): Path_" << i;
				rob_id++;
			}
		}
	#endif

	#ifdef DEBUG_GOALS_LOCS
		cout << "\n\nGoals...";

		for(uint i = 0; i < num_of_goals; i++)
		{
			ondemcpp_pkg::State goal_loc = goals_locs[i];
			cout << "\nG_" << i << " (" << goal_loc.x << ", " << goal_loc.y << ")";
		}
	#endif

	string ondemcpp_pkg_path = package::getPath("ondemcpp_pkg");
	ofstream debug_file;
	string debug_filename;

	//================================================== 1. Optimal Costs : START
	int_mat opt_cost_mat = compute_optimal_costs(ws_size_x, ws_size_y, ws_graph, robs_states, goals_locs);
	printf("\n\nOptimal Cost Matrix...\n");

	#ifdef DEBUG_OC
		debug_filename = ondemcpp_pkg_path + OUTPUT_DIR + DEBUG_OC_FILENAME + boost::lexical_cast<string>(hor_id) + CSV_EXT;
		debug_file.open(debug_filename.c_str(), ios::out);

		for(int rob_index = 0; rob_index < num_of_robs; rob_index++)
		{
			for(int goal_index = 0; goal_index < num_of_goals; goal_index++)
			{
				cout << opt_cost_mat[rob_index][goal_index] << " ";
				debug_file << opt_cost_mat[rob_index][goal_index];

				if((goal_index + 1) != num_of_goals)
					debug_file << ",";
			}

			cout << endl;
			debug_file << endl;
		}

		debug_file.close();
	#endif
	//================================================== 1. Optimal Costs : END

	//================================================== 2. Optimal Goals : START
	MUNKRES_ALGO ma_obj;
	int_vec opt_goal_vec = ma_obj.munkres(opt_cost_mat, num_of_robs, num_of_goals);		//Optimal goal assignments
	cout << "\nOptimal Goal Vector...\n";

	#ifdef DEBUG_OG
		debug_filename = ondemcpp_pkg_path + OUTPUT_DIR + DEBUG_OG_FILENAME + boost::lexical_cast<string>(hor_id) + TXT_EXT;
		debug_file.open(debug_filename.c_str(), ios::out);

		for(int i = 0; i < num_of_robs; i++)
		{
			cout << "R_" << i << "\t\tG_" << opt_goal_vec[i] << endl;
			debug_file << opt_goal_vec[i];

			if((i + 1) != num_of_robs)
				debug_file << ",";
		}

		cout << endl;
		debug_file.close();
	#endif
	//================================================== 2. Optimal Goals : END
	uint active_count = 0;

	//================================================== 3. Optimal Paths : START
	robots_states_mat opt_paths = compute_optimal_paths(ws_graph, robs_states, goals_locs, opt_goal_vec, S_r, paths_old, active_count);		// Optimal Paths
	cout << "\nOptimal Paths...\n";

	#ifdef DEBUG_OP
		debug_filename = ondemcpp_pkg_path + OUTPUT_DIR + DEBUG_OP_FILENAME + boost::lexical_cast<string>(hor_id) + CSV_EXT;
		debug_file.open(debug_filename.c_str(), ios::out);

		for(int i = 0; i < rob_count; i++)
		{
			cout << "Path_" << i << " ";

			// loc_vec path_temp = opt_paths[i];
			loc_vec path_temp = opt_paths[i];

			for(int j = 0; j < path_temp.size(); j++)
			{
				cout << "(" << path_temp[j].x << ", " << path_temp[j].y << ", " << path_temp[j].theta << ") ";
				debug_file << "(" << path_temp[j].x << " " << path_temp[j].y << " " << path_temp[j].theta << "),";
			}

			cout << endl;
			debug_file << endl;
		}
		
		debug_file.close();
	#endif
	//================================================== 3. Optimal Paths : END
	uint iter_id = 0;
	bool_mat partial_order;
	bool flag_to_found;		//Is Total Order found?
	int_vec total_order, start_time_offsets;
	robots_states_mat feasible_paths;

	while(true)
	{
		//================================================== 4. Feasible Paths : START
		uint killed_count = 0;		// Number of killed robots
		uint revived_count = 0;		// Number of revived robots
		uint revived_goal_count = 0;		// Number of goals of revived robots that will be visited. revived_count <= revived_goal_count <= killed_count
		feasible_paths = get_feasible_paths(opt_goal_vec, opt_paths, killed_count, revived_count, revived_goal_count, ws_size_x, ws_size_y, S_r);

		#ifdef DEBUG_FP
			cout << "\nFeasible Paths...\n";
			debug_filename = ondemcpp_pkg_path + OUTPUT_DIR + DEBUG_FP_FILENAME + to_string(hor_id) + "_i" + to_string(iter_id) + CSV_EXT;
			debug_file.open(debug_filename.c_str(), ios::out);

			for(int i = 0; i < rob_count; i++)
			{
				cout << "Path_" << i << " ";

				// loc_vec path_temp = feasible_paths[i];
				loc_vec path_temp = feasible_paths[i];

				for(int j = 0; j < path_temp.size(); j++)
				{
					cout << "(" << path_temp[j].x << ", " << path_temp[j].y << ", " << path_temp[j].theta << ") ";
					debug_file << "(" << path_temp[j].x << " " << path_temp[j].y << " " << path_temp[j].theta << "),";
				}

				cout << endl;
				debug_file << endl;
			}

			debug_file.close();
		#endif

		debug_filename = ondemcpp_pkg_path + OUTPUT_DIR + DEBUG_FP_STAT_FILENAME + TXT_EXT;
		debug_file.open(debug_filename.c_str(), ios::app);
		debug_file << hor_id << "," << iter_id << "," << active_count << "," << killed_count << "," << revived_count << "," << revived_goal_count << endl;
		debug_file.close();
		//================================================== 4. Feasible Paths : END
		//================================================== 5. Partial Orders : START
		partial_order = compute_partial_orders(rob_count, feasible_paths, S_r);
		cout << "\nPartial Orders...\n";

		#ifdef DEBUG_PO
			debug_filename = ondemcpp_pkg_path + OUTPUT_DIR + DEBUG_PO_FILENAME + boost::lexical_cast<string>(hor_id) + CSV_EXT;
			debug_file.open(debug_filename.c_str(), ios::out);

			for(int i = 0; i < rob_count; i++)
			{
				bool_vec v = partial_order[i];

				for(int j = 0; j < rob_count; j++)
				{
					if(v[j])
					{
						cout << "1 ";
						debug_file << "1";
					}
					else
					{
						cout << "0 ";
						debug_file << "0";
					}


					if((j + 1) != rob_count)
						debug_file << v[j] << ",";
				}

				cout << endl;
				debug_file << endl;
			}

			debug_file.close();
		#endif
		//================================================== 5. Partial Orders : END

		//================================================== 6. Total Order : START
		bool_mat po_residue = partial_order;		//Residue of PO
		total_order = compute_total_order(rob_count, partial_order, po_residue);

		flag_to_found = true;

		for(uint rob_id = 0; rob_id < rob_count; rob_id++)
			if(total_order[rob_id] == -1)
			{
				flag_to_found = false;
				break;
			}

		active_count = rob_count;

		if(!flag_to_found)
		{
			cout << "\nInvalid TO\n";

			#ifdef DEBUG_TO
				debug_filename = ondemcpp_pkg_path + OUTPUT_DIR + DEBUG_TO_FILENAME + boost::lexical_cast<string>(hor_id) + CSV_EXT;
				debug_file.open(debug_filename.c_str(), ios::out);

				for(int i = 0; i < rob_count; i++)
				{
					bool_vec v = po_residue[i];

					for(int j = 0; j < rob_count; j++)
					{
						if(v[j])
						{
							cout << "1 ";
							debug_file << "1";
						}
						else
						{
							cout << "0 ";
							debug_file << "0";
						}

						if((j + 1) != rob_count)
							debug_file << v[j] << ",";
					}

					cout << endl;
					debug_file << endl;
				}

				debug_file.close();
			#endif

			break_dependency_cycles(opt_goal_vec, po_residue, S_r);
			adjust_dependent_paths(opt_goal_vec, feasible_paths, S_r, active_count);
		}
		else
		{
			cout << "\nTotal Order...\n";

			#ifdef DEBUG_TO
				debug_filename = ondemcpp_pkg_path + OUTPUT_DIR + DEBUG_TO_FILENAME + boost::lexical_cast<string>(hor_id) + TXT_EXT;
				debug_file.open(debug_filename.c_str(), ios::out);

				for(int i = 0; i < rob_count; i++)
				{
					cout << total_order[i] << " ";
					debug_file << total_order[i] << " ";
				}

				cout << endl;
				debug_file.close();
			#endif

			//================================================== 6. Start-time offsets : START
			bool flag_inactivated_robot = false;
			start_time_offsets = compute_start_time_offsets(total_order, feasible_paths, S_r, opt_goal_vec, flag_inactivated_robot);
			
			if(flag_inactivated_robot)
				adjust_dependent_paths(opt_goal_vec, feasible_paths, S_r, active_count);
			else
			{
				cout << "\nStart-Time Offsets...\n";

				#ifdef DEBUG_SO
					debug_filename = ondemcpp_pkg_path + OUTPUT_DIR + DEBUG_SO_FILENAME + boost::lexical_cast<string>(hor_id) + TXT_EXT;				
					debug_file.open(debug_filename.c_str(), ios::out);

					for(int i = 0; i < rob_count; i++)
					{
						uint sto = start_time_offsets[i];

						if(sto)
							cout << "R_" << i << "=" << start_time_offsets[i] << " ";
						else	
							cout << sto << " ";

						debug_file << start_time_offsets[i];

						if((i + 1) != rob_count)
							debug_file << " ";
					}

					cout << endl;
					debug_file.close();
				#endif

				break;
			}
			//================================================== 7. Start-time offsets : END
		}

		opt_paths = feasible_paths;
		cout << "\nAdjusted dependent paths...\n";

		#ifdef DEBUG_DP
			debug_filename = ondemcpp_pkg_path + OUTPUT_DIR + DEBUG_DP_FILENAME + boost::lexical_cast<string>(hor_id) + CSV_EXT;
			debug_file.open(debug_filename.c_str(), ios::out);

			for(uint i = 0; i < rob_count; i++)
			{
				cout << "Path_" << i << " ";

				// loc_vec path_temp = opt_paths[i];
				loc_vec path_temp = opt_paths[i];

				for(uint j = 0; j < path_temp.size(); j++)
				{
					cout << "(" << path_temp[j].x << ", " << path_temp[j].y << ", " << path_temp[j].theta << ") ";
					debug_file << "(" << path_temp[j].x << " " << path_temp[j].y << " " << path_temp[j].theta << "),";
				}

				cout << endl;
				debug_file << endl;
			}
		#endif
		//================================================== 7. Total Order : END

		iter_id++;
	}

	//================================================== 8. Collision Averted Paths : START
	robots_states_mat trajectories = compute_optimal_trajectories(num_of_robs, start_time_offsets, feasible_paths, S_r, hor_id);
	cout << "\nCollision Averted Paths...\n";

	#ifdef DEBUG_CAP
		debug_filename = ondemcpp_pkg_path + OUTPUT_DIR + DEBUG_CAP_FILENAME + boost::lexical_cast<string>(hor_id) + CSV_EXT;
		debug_file.open(debug_filename.c_str(), ios::out);

		for(int i = 0; i < rob_count; i++)
		{
			cout << "Path_" << i << " ";
			robots_states_vec t = trajectories[i];

			for(int j = 0; j < t.size(); j++)
			{
				ondemcpp_pkg::State robot_loc = t[j];
				cout << "(" << robot_loc.x << ", " << robot_loc.y << ", " << robot_loc.theta << ") ";
				debug_file << "(" << robot_loc.x << " " << robot_loc.y << " " << robot_loc.theta << "),";
			}

			cout << endl;
			debug_file << endl;
		}

		debug_file.close();
	#endif
	//================================================== 8. Collision Averted Paths : END
	#ifdef MONITOR_PATHS
		monitor_paths(ws_size_x, ws_size_y, rob_count, trajectories);
	#endif

	return trajectories;
}