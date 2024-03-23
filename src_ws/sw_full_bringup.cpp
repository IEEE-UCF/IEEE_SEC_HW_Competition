#include <iostream>
#include <filesystem>
#include <string>
#include <thread>
#include <fstream>

bool isFileEmpty(const std::string& filename) {
    std::ifstream file(filename);
    return file.peek() == std::ifstream::traits_type::eof();
}


int main() {
    
    std::cout<< "Starting to search for signal\n";


    //THE GREEN LIGHT DETECTOR SHOULD SEND DATA TO A FILE WHEN COMPLETE. THE FILE SHOULD BE HERE:
    const std::string filename = "green_detector/greenlightstart.txt";

    while (isFileEmpty(filename)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout<<"Havent recieved green light signal, waiting..\n";
    };

    std::cout<< "Recived green light signal, starting ros2..\n";

    //CHECK THAT EACH BRINGUP USES SIM TIME IF NEEDED IN std::system LAUNCHES FUNCTIONS
    std::system("ros2 run secbot_bring_up gazebo_bringup");
    std::system("ros2 run secbot_bring_up nav_bringup");
    std::system("ros2 run secbot_bring_up waypoint_bringup");

    std::cout<< "node_checker should've ended, go check\n";


    return 0;

}