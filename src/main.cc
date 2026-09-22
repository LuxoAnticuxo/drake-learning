// The file uses .cc since that's how drake does it.

#include <iostream>
#include <memory>
#include <fstream>

// browser visualizer
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>

// parser to read the robot
#include <drake/multibody/parsing/parser.h>

// Plant is the system.
// Diagram holds various systems.
// Simulator will apply integrators.
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/analysis/simulator.h>

// Logger
#include <drake/systems/primitives/vector_log_sink.h>

using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::math::RigidTransformd;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::LogVectorOutput;
using drake::systems::Simulator;

int main() {

    // Makes the builder, which has the plant and the scene graph connected to it.
    DiagramBuilder<double> builder;
    auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);

    // The parser reads an XML file to know the links, joints and actuators of the robot.
    Parser parser(&plant);
    parser.AddModels("../models/Acrobot.urdf"); // copied from the drake example models folder

    // The robot system is done after the parsing, so we finish it for drake to create the states going to be used for the sim.
    plant.Finalize();

    // The input port is inside the builder, so we make it visible in this scope with this function.
    builder.ExportInput(plant.get_actuation_input_port(), "actuation");

    // makes the meshcat and its visualizer, then connect it.
    auto meshcat = std::make_shared<Meshcat>(); // Shared pointer since it will be used by this main function and the visualizer.
    MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat);

    // makes the camera orthographic and sets the window size.
    meshcat->Set2dRenderMode(RigidTransformd(Eigen::Vector3d(0.0, -1.0, 0.0)),-3.0,3.0,-3.0, 3.0); // The vector is the camera direction.

    // gets the data of the system.
    auto* logger = LogVectorOutput(plant.get_state_output_port(), &builder);

    // finishes the build and passes it to the diagram.
    auto diagram = builder.Build();

    // Sets the simulator and run it in real time (1.0)
    Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(1.0);

    // getting the address of the context to edit it.
    auto& context = simulator.get_mutable_context();
    context.SetContinuousState(Eigen::Vector4d(1.0, 1.0, 0.0, 0.0)); // theta1, theta2, theta1dot, theta2dot
    const Eigen::VectorXd u = Eigen::VectorXd::Zero(plant.num_actuated_dofs()); // VectorXd size gets decided at runtime, u with zeroes means zero torque actuated.

    // assigns u to the input port.
    diagram->GetInputPort("actuation").FixValue(&context, u);


    // simulation
    std::cout << "Open this in your browser to see meshcat: " << meshcat->web_url();
    simulator.AdvanceTo(10.0); // 10 seconds of simulation

    // Logging data
    const auto& log = logger->FindLog(context);

    std::ofstream csv("../build/data/pendulum_log.csv");
    csv << "t,theta1,theta2,theta1dot,theta2dot\n";

    for (int i = 0; i < log.num_samples(); ++i) {
        csv << log.sample_times()(i); // returns a vector and indexes it.


        for (int row = 0; row < log.data().rows(); ++row) {
            csv <<  "," << log.data()(row, i);
        }
        csv << "\n";
    }
    return 0;
}