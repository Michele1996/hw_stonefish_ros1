/*    
    This file is a part of stonefish_ros.

    stonefish_ros is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    stonefish_ros is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  parsed_simulator_nogpu.cpp
//  stonefish_ros
//
//  Created by Patryk Cieslak on 16/12/20.
//  Copyright (c) 2020 Patryk Cieslak. All rights reserved.
//

#include <ros/ros.h>
#include <Stonefish/core/ConsoleSimulationApp.h>
#include <Stonefish/utils/SystemUtil.hpp>
#include "stonefish_ros/ROSSimulationManager.h"
#include <std_srvs/Empty.h>
#include <SDL2/SDL.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "parsed_simulator_nogpu", ros::init_options::NoSigintHandler);

    //Check number of command line arguments
	if(argc < 4)
	{
		ROS_FATAL("Not enough command line arguments provided!");
		return 1;
	}

    //Parse arguments
    std::string dataDirPath = std::string(argv[1]) + "/";
    std::string scenarioPath(argv[2]);
    sf::Scalar rate = atof(argv[3]);
	
	sf::ROSSimulationManager manager(rate, scenarioPath);
    sf::ConsoleSimulationApp app("Stonefish Simulator", dataDirPath, &manager); 
	app.Run();

	return 0;
}

namespace {
    // Static variable to store the GraphicalSimulationApp instance
    static sf::ConsoleSimulationApp* simulatorApp = nullptr;
    static sf::ROSSimulationManager* manager = nullptr;
}
// Function to stop the simulator
void stopSimulator() {
    if (simulatorApp) {
        simulatorApp->StopSimulationWrapper();
        ROS_INFO("Simulator stopped.");
    } else {
        ROS_WARN("Simulator not running.");
    }
}

// Function to resume the simulator
void resumeSimulator() {
    if (simulatorApp) {
        simulatorApp->ResumeSimulationWrapper();
        ROS_INFO("Simulator resumed.");
    } else {
        ROS_WARN("Simulator not running.");
    }
}

// Function to resume the simulator
void destSimulator() {
 
    // Simulate pressing the space bar
    SDL_Event event;
    event.type = SDL_KEYDOWN;
    event.key.keysym.sym = SDLK_SPACE;
    event.key.state = SDL_PRESSED;

    SDL_PushEvent(&event);
  
     
}

// Service callback to stop the simulator
bool stopSimulationCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    stopSimulator();
    return true;
}

// Service callback to resume the simulator
bool resumeSimulationCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    resumeSimulator();
    return true;
}

// Service callback to resume the simulator
bool restartSimulationCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    destSimulator();
    return true;
}

// Function to run the simulator
void runSimulatorROS(const std::string& dataDirPath, const std::string& scenarioPath, double rate) {
    // Initialize ROS node
    int argc = 1;  
    
    char* argv[] = {"parsed_simulator"};
    ros::init(argc, argv, "parsed_simulator", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::ServiceServer stopService = nh.advertiseService("stop_simulation", stopSimulationCallback);
    ros::ServiceServer resumeService = nh.advertiseService("resume_simulation", resumeSimulationCallback);
    ros::ServiceServer destService = nh.advertiseService("restart_simulation", restartSimulationCallback);
    // Check if ROS is initialized successfully
    if (!ros::isInitialized()) {
        ROS_FATAL("Failed to initialize ROS. Exiting...");
        return;
    }
    

    sf::Scalar sfRate = rate;
    sf::ROSSimulationManager manag(sfRate, scenarioPath);
    sf::ConsoleSimulationApp app("Stonefish Simulator", dataDirPath, &manag); 

    // Assign the app to the static variable
    simulatorApp = &app;
    manager=&manag;

    

    // Run the simulator
    app.Run();
    
}

