#include <pybind11/pybind11.h>
#include "parsed_simulator_nogpu.cpp"

namespace py = pybind11;


// Wrapper function to run the simulator with ROS
void runSimulatorROSWrapper(const std::string& dataDirPath, const std::string& scenarioPath, double rate) {
    runSimulatorROS(dataDirPath, scenarioPath, rate);
}


PYBIND11_MODULE(parsed_simulator_bindings_nogpu, m) {
    m.def("run_simulator_ros", &runSimulatorROSWrapper, "Run the Stonefish Simulator with ROS");

}

