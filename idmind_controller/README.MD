# idmind_controller

This package contains a controller for robot planned navigation.

####Files
- **controller_interface.py** - This node receives the requests
- **idmind_lasers.py** - This node reads lasers and publishes closest obstacles
- **idmind_controller** - This node contains the Action server and the controller for trajectory following
- **idmind_recovery** - Contains some methods for obstacle avoidance, called by the controller

##Action Server
- **idmind_controller**

    This package provides an action server, compatible with move_base messages. Should be used via the `/idmind_controller/follow_path` service.

##Services Provided
- **/idmind_controller/follow_path (FollowPath)**

    By providing a list of poses, the controller will iteratively move from pose to pose. 
    Planning is done via the move_base packages.
    Navigation is done via PID trajectory follower, with some obstacle avoidance abilities.
    
    *TODO*: Implement PWA or MPC controller, to improve from reaction control to planned.
    
- **/idmind_controller/action (DoAction)**
    
    Not yet implemented. It will perform an action.
    
- **/idmind_controller/cancel_navigation (Trigger)**
    
    Cancels navigation and stops the robot.

- **/idmind_recovery/obstacle_handler (Trigger)**  
    
    This service will move the robot away from obstacles around him.
    *TODO* - Detect wall and corridors
   
- **/idmind_controller/list_obstacles (Obstacles)** - Return a list with [x1, y1, x2, y2,... xn, yn] positions of obstacles detected by lasers  

##Services Required
- **/move_base/NavfnROS/make_plan (MakePlan)**
    
    The planner provided by move_base package
##Topics Subscribed
- **/idmind_logging (/idmind_robot/Log)**

    Logging only

- **laser topics** - Listens to all laser topics, as described in the move_base local costmap definitions for the obstacle layer

##Topics Published
- **/idmind_controller/rviz_goal (PoseStamped)** - When giving a 2D navigation Goal in RViz, if using the correct namespace, it will be used for navigation.
- **/idmind_controller/checkpoint (PoseStamped)** - Current checkpoint, used mainly for visualization
- **/cmd_vel_idmind (Twist)** - Velocity to be handled by a navigation node