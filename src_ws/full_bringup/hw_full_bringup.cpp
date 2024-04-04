#include <iostream>
#include <filesystem>
#include <string>
#include <thread>
#include <fstream>

bool isFileEmpty(const std::string& filename) {
    std::ifstream file(filename);
    return file.peek() == std::ifstream::traits_type::eof();
}

void checkGreenLight(){
    std::cout<< "Searching for Green Light Signal\n";

    //THE GREEN LIGHT DETECTOR SHOULD SEND DATA TO A FILE WHEN COMPLETE. THE FILE SHOULD BE HERE:
    const std::string filename = "green_detector/greenlightstart.txt";

    while (isFileEmpty(filename)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout<<"No green light signal recieved, waiting..\n";
    };

    std::cout<< "Recived Green Light Signal - Starting Bringup..\n";
}

int main() {

    //Uncomment this function call if you plan to check for a greenlight before launching the software bringup
    //checkGreenLight();

    std::system("ros2 run secbot_bring_up hw_launch_bringup");
    std::system("ros2 run secbot_bring_up hw_nav_bringup&");
    std::system("ros2 run secbot_bring_up hw_waypoint_bringup");


    return 0;

}