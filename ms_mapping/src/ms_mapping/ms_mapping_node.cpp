#include "ms_mapping.h"

int main(int argc, char **argv)
{
    // Initialize the ROS node with custom signal handling (NoSigintHandler allows custom SIGINT handling)
    ros::init(argc, argv, "MS-Mapping Start!", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    // Create a shared pointer to the MSMapping object, which encapsulates the mapping functionalities.
    auto pgo_wrapper = std::make_shared<MSMapping>(nh);

    // Use lambda functions to create threads for different tasks concurrently.
    // Thread for pose graph SLAM.
    std::thread posegraph_slam([&]()
                               { pgo_wrapper->pose_slam(); });

    // Thread for visualization.
    std::thread viz_map([&]()
                        {
                            pgo_wrapper->VisualizationThread(); // Note: Ensure the method name matches your class definition.
                        });

    // Thread for loop closure detection; only start if loop closure is enabled.
    std::thread lc_detection;
    if (useLoopCloser)
    {
        lc_detection = std::thread([&]()
                                   {
                                       pgo_wrapper->LoopDetection(); 
                                   });
    }

    // Create a MultiThreadedSpinner to process ROS callbacks concurrently using 8 threads.
    ros::MultiThreadedSpinner spinner(8);
    spinner.spin();

    // After the spinner stops (typically on shutdown), notify all threads to stop gracefully.
    pgo_wrapper->requestStop();

    // Join all threads to ensure they finish execution before the program exits.
    if (posegraph_slam.joinable())
        posegraph_slam.join();
    if (viz_map.joinable())
        viz_map.join();
    if (lc_detection.joinable())
        lc_detection.join();

    return EXIT_SUCCESS;
}
