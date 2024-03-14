#include <iostream>
#include <filesystem>
#include <string>
#include <cstdlib>


int main() {

    std::system("ros2 run secbot_bring_up node_checker");



    std::cout<<"node_checker should've ended, go check\n";


    return 0;

}