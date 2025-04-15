import rospy
import parsed_simulator_bindings

def main():
    # Initialize the ROS node
    rospy.init_node('stonefish_simulator')

    # Get parameters using rospy
    simulation_data = rospy.get_param('~simulation_data', '')
    scenario_description = rospy.get_param('~scenario_description', '')
    simulation_rate = rospy.get_param('~simulation_rate', 100)
    graphics_resolution = rospy.get_param('~graphics_resolution', '1200 800')
    graphics_quality = rospy.get_param('~graphics_quality', 'medium')

    # Call the exposed function
    
    parsed_simulator_bindings.run_simulator_ros(simulation_data, scenario_description,
simulation_rate, *map(int, graphics_resolution.split()),graphics_quality)
                                                
                                                


if __name__ == "__main__":
    main()

