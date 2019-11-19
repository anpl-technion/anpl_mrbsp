/* ---------------------------------------------------------------------------
 *
 * Autonomous Navigation and Perception Lab (ANPL),
 * Technion, Israel Institute of Technology,
 * Faculty of Aerospace Engineering,
 * Haifa, Israel, 32000
 * All Rights Reserved
 *
 * See LICENSE for the license information
 *
 * -------------------------------------------------------------------------- */
/**
 * @file: action_generator.cpp
 * @brief: ROS node main
 * @author: Andrej Kitanov
 *
 */

#include <action_generator/action_generator.hpp>


int main(int argc, char **argv) {

    ros::init(argc, argv, "action_generator", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    ActionGenerator ag(nh);

    //ros::spin();
    ros::Rate r(10);
    while (ros::ok())
    {
        //ag.Draw(); // Handle Drawing events
        ros::spinOnce();                   // Handle ROS events
        r.sleep();
        //std::cout << "main" << std::endl;
    }


    std::cout << "Shutting down service..." << std::endl;

    sleep(1);

    return 0;
}