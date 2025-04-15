#include <pybind11/pybind11.h>
#include "parsed_simulator.cpp"

namespace py = pybind11;


// Wrapper function to run the simulator with ROS
void runSimulatorROSWrapper(const std::string& dataDirPath, const std::string& scenarioPath, double rate, int windowW, int windowH, const std::string& quality) {
    runSimulatorROS(dataDirPath, scenarioPath, rate, windowW, windowH, quality);
}


PYBIND11_MODULE(parsed_simulator_bindings, m) {
    m.def("run_simulator_ros", &runSimulatorROSWrapper, "Run the Stonefish Simulator with ROS");

}

