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
//  parsed_simulator.cpp
//  stonefish_ros
//
//  Created by Patryk Cieslak on 12/06/19.
//  Copyright (c) 2019-2020 Patryk Cieslak. All rights reserved.
//

#include <ros/ros.h>
#include <Stonefish/core/GraphicalSimulationApp.h>
#include <Stonefish/utils/SystemUtil.hpp>
#include "stonefish_ros/ROSSimulationManager.h"
#include <std_srvs/Empty.h>
#include <SDL2/SDL.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "parsed_simulator", ros::init_options::NoSigintHandler);

    //Check number of command line arguments
	if(argc < 7)
	{
		ROS_FATAL("Not enough command line arguments provided!");
		return 1;
	}

    //Parse arguments
    std::string dataDirPath = std::string(argv[1]) + "/";
    std::string scenarioPath(argv[2]);
    sf::Scalar rate = atof(argv[3]);

	sf::RenderSettings s;
    s.windowW = atoi(argv[4]);
    s.windowH = atoi(argv[5]);

    std::string quality(argv[6]);
    if(quality == "low")
    {
        s.shadows = sf::RenderQuality::LOW;
        s.ao = sf::RenderQuality::DISABLED;
        s.atmosphere = sf::RenderQuality::LOW;
        s.ocean = sf::RenderQuality::LOW;
        s.aa = sf::RenderQuality::LOW;
        s.ssr = sf::RenderQuality::DISABLED;
    }
    else if(quality == "high")
    {
        s.shadows = sf::RenderQuality::HIGH;
        s.ao = sf::RenderQuality::HIGH;
        s.atmosphere = sf::RenderQuality::HIGH;
        s.ocean = sf::RenderQuality::HIGH;
        s.aa = sf::RenderQuality::HIGH;
        s.ssr = sf::RenderQuality::HIGH;
    }
    else // "medium"
    {
        s.shadows = sf::RenderQuality::MEDIUM;
        s.ao = sf::RenderQuality::MEDIUM;
        s.atmosphere = sf::RenderQuality::MEDIUM;
        s.ocean = sf::RenderQuality::MEDIUM;
        s.aa = sf::RenderQuality::MEDIUM;
        s.ssr = sf::RenderQuality::MEDIUM;
    }
    
    std::string turbidity_level(argv[7]);
    std::cout<<"TURBIDITY LEVEL " << turbidity_level <<std::endl;

    sf::HelperSettings h;
    h.showFluidDynamics = false;
    h.showCoordSys = false;
    h.showBulletDebugInfo = false;
    h.showSensors = false;
    h.showActuators = false;
    h.showForces = false;
    // Initialize OpenGL window
    /**glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1000, 800);
    glutCreateWindow("Event-based Camera Simulation");
    **/
	sf::ROSSimulationManager manager(rate, scenarioPath);
	//    glutDisplayFunc(manager->EventCameraImageReady);
    sf::GraphicalSimulationApp app("Stonefish Simulator", dataDirPath, s, h, turbidity_level, &manager); 
	app.Run();

	return 0;
}

namespace {
    // Static variable to store the GraphicalSimulationApp instance
    static sf::GraphicalSimulationApp* simulatorApp = nullptr;
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
void runSimulatorROS(const std::string& dataDirPath, const std::string& scenarioPath, double rate, int windowW, int windowH, const std::string& quality) {
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
    
    bool flag=false;
    sf::Scalar sfRate = rate;
    sf::RenderSettings s;
    sf::HelperSettings h;
    h.showFluidDynamics = false;
    h.showCoordSys = false;
    h.showBulletDebugInfo = false;
    h.showSensors = false;
    h.showActuators = false;
    h.showForces = false;

    // Create the GraphicalSimulationApp instance
    sf::ROSSimulationManager manag(sfRate, scenarioPath);
    sf::GraphicalSimulationApp app("Stonefish Simulator", dataDirPath, s, h, "10", &manag);

    // Assign the app to the static variable
    simulatorApp = &app;
    manager=&manag;

    

    // Run the simulator
    app.Run();
    
}
