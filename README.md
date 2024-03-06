# OnDemCPP (Online On-Demand Multi-Robot Coverage Path Planning)
Implementation of **Online On-Demand Multi-Robot Coverage Path Planning** [ICRA 2024] in **ROS Noetic Ninjemys** using **C++**.

Paper's URL: [CoRR](https://arxiv.org/abs/2303.00047)<br/>
Video's URL: [YouTube](https://www.youtube.com/watch?v=5nhysTTp2Fw)

###### Instructions:

1.  Download the source code package:<br/> 
    `cd ~/catkin_ws/src/`<br/> 
    `git clone https://github.com/iitkcpslab/OnDemCPP.git`
2.  Understand the directory structure:<br/> 
    1.  include: Contains the header files.<br/> 
        By default, the package is configured for the motions of a *Quadcopter in a 2D workspace*. However, this can be reconfigured for the motions of a *Turtlebot* in *config.h* file by uncommenting `#define TURTLEBOT`. 
    2.  input: *ws_obs_robs.txt* represents the 2D workspace grid with obstacles and initial locations of the robots. 
         | Value      | Meaning                              |
         | -----      | ------------------------------------ |
         | 0.0        | Obstacle-occupied cell               |
         | 0.5        | Obstacle-free cell                   |
         | i (&ge; 1) | Initial location of **Robot-i**      |

        *E.g.*, the given file represents a *10 x 10* grid with 3 robots - Robot-1, Robot-2, and Robot-3, having initial locations (1,0), (2, 1), and (3,2), respectively. 
    3.  msg: Contains message files. 
    4.  output: Contains the obtained results when run with the input.
        * *j_capl.txt* file stores the collision-free path lengths of the participants in the *j*-th horizon. The format of each row is as follows -

        | Robot ID | Path Length | Start-Time Offset | Collision-Free Path Length | 
        | ---------| ------------|-------------------|--------------------------- |
        * *cap_j.csv* file stores the collision-free paths of all the robots in the *j*-th horizon. 
        * *fp_stat.txt* file stores the performance of algorithm, which computes feasible paths. The format of each row is as follows -

        | Horizon ID | Iteration ID of the inner WHILE loop | Active robot count | Killed robot count | Revived robot count | Visited goal count |
        | ---------- | ------------------------------------ | ------------------ | ------------------ | ------------------- | ------------------ |
        * *path_len.txt* file stores the total collision-free path lengths of all the robots.
        * *post_comp.txt* file stores the times spent on sending the paths to all the active participants in a particular horizon. 
        * *pre_comp.txt* file stores the times spent on updating the global view in a particular horizon.
        * *r_i.txt* file stores the time spent on updating the global view with *Robot-i*'s localview.
        * *result.txt* file stores the overall computation time, horizon length, and mission time. 
        * *resultPerHorizon.txt* file stores horizon-wise computation times and horizon lengths. 
    6.  src: Contains the source files corresponding to the header files. 
        * Robot side: *robot.cpp* emulates a robot. 
        * Coverage Planner side: The rest of the source files. 
    7.  srv: Contains the service files. 
3.  Build the package:<br/> 
    `cd ~/catkin_ws && catkin_make clean && catkin_make`<br/>
    `echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`
4.  Run the package in a Terminal:
    -   Tab 1:
        `rosclean purge -y && pkill roscore ; roscore`
    -   Tab 2:
        `rm ~/catkin_ws/src/OnDemCPP/output/*`<br/> 
        `rosrun ondemcpp_pkg onDemCPPExe _ws_x:=<Workspace size along the +x axis> _ws_y:=<Workspace size along the +y axis> _rc:=<Robot count>`
    -   Tab 3:
        `cp ~/<Workspace directory>/ws_obs_robs.txt ~/catkin_ws/src/OnDemCPP/input/`<br/>
        `rosrun ondemcpp_pkg robotExe __name:=robot_<Robot ID> _rid:=<Robot ID> # Run for each Robot-i`
