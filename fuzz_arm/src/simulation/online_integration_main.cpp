#include "simulation/core/simulation_app.hpp"
#include <iostream>

int main(int argc, char **argv) {
  try {
    robot_sim::simulation::SimulationApp::instance().run(argc, argv);
  } catch (const std::exception &e) {
    std::cerr << "Unhandled exception: " << e.what() << std::endl;
    return 1;
  } catch (...) {
    std::cerr << "Unknown exception occurred." << std::endl;
    return 1;
  }
  return 0;
}
