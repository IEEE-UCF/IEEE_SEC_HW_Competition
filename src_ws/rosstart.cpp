#include <iostream>
#include <filesystem>
#include <string>
#include <cstdlib>


int main() {

    const int counter_max = 1;

    int counter = 0;

    while(counter<counter_max){
        std::system("ros2 run secbot_bring_up gazebo_bringup");
        std::system("ros2 run secbot_bring_up nav_bringup");
        std::system("ros2 run secbot_bring_up waypoint_bringup");
        counter++;
    }

    std::cout<<"node_checker should've ended, go check\n";


    return 0;

}