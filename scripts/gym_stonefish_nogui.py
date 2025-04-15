import rospy
import parsed_simulator_bindings_nogpu

def main():
    # Initialize the ROS node
    rospy.init_node('stonefish_simulator')

    rospy.loginfo("CHECK")
    # Get parameters using rospy
    simulation_data = rospy.get_param('~simulation_data', '')
    scenario_description = rospy.get_param('~scenario_description', '')
    simulation_rate = rospy.get_param('~simulation_rate', 100)

    # Call the exposed function
    
    parsed_simulator_bindings_nogpu.run_simulator_ros(simulation_data, scenario_description,
simulation_rate)
                                                
                                                


if __name__ == "__main__":
    main()

