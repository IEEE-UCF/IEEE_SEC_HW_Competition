#include <iostream>
#include <filesystem>
#include <string>
#include <cstdlib>


int main() {

    std::system("ros2 launch secbot_simulation launch_sim.launch.py &");

    run a node that listens to rosout. if rosout msgs doesnt match right msg, wait one second, and try again.

    Return 0 if msgs match, or try five times and return 1(while i<5, wait sec, check, i++).

    if return value is 1 restart, otherwise continue

    do the same for every launch file.


    continous launch file

    timer that will reset the node if the time exceeds 10 seconds.

    non continuos topic that listens, then runs detsroy, rcl:shut, pkill


    return 0;

}