# `secbot_bringup` Documentation
- Holds launches and files for the bringup of the robot.
## Directory: `src`
- Contains c++ executables that are called by the src_ws/full_bringup files.
- **ALL** current src files depend on the topic `/rosout` to listen for succesful startup and error msgs.
### launch_bringup
- Starts everything pre-nav2.
- It will reset itself until it believes that the system has started correctly.
### nav_bringup
- Starts localization and navigation.
- It will reset itself and launch_bringup until it believes that navigation and localization have started correctly.
### waypoint_bringup
- Starts waypoint following.
- It will endlessely listen for implementation-defined errors, and will end all processes if an error is found.
- Ends all proccess if amcl takes too long to start, allowing the system to restart if the user puts **all** bringup calls in a while loop.
### Important Functions
-  *end_all_nodes*
    1. Sends a command to the terminal to **peacefully** terminate the process of all ros2 nodes.
    2. Must be updated with the addition of more packages and nodes added to the system.
    3. If the user **doesn't** update the proccesses to be killed, **proccesses will run indefinitely**.

