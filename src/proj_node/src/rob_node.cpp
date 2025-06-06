#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <tinyxml2.h>
#include "grid_map.h"
#include "world_item.h"
#include "laser_scanner.h"
#include "draw_helpers.h"


// Define simulation parameters
const float GRID_RESOLUTION = 0.1; // 10 cm per cell

// Function to parse XML and create robot objects
// Returns: a vector of UnicyclePlatform pointers
std::vector<UnicyclePlatform*> parseRobotsFromXML(World& world, const std::string& file_path) {
    // Define a vector to hold the robots from Grisetti's UnicyclePlatform class
    std::vector<UnicyclePlatform*> robots;

    // Load the XML file using TinyXML2
    tinyxml2::XMLDocument doc;

    tinyxml2::XMLError loadResult = doc.LoadFile(file_path.c_str());
    if (loadResult != tinyxml2::XML_SUCCESS) {
        ROS_ERROR("Failed to load XML file: %s", doc.ErrorIDToName(loadResult));
        return robots; // Return empty vector if loading fails
    }

    // Get the first element of the 'simulation' tag
    tinyxml2::XMLElement* root = doc.FirstChildElement("simulation");

    // Take the robots element
    tinyxml2::XMLElement* robotElement = root->FirstChildElement("robots")->FirstChildElement("robot");


    // Loop through all robot elements
    while (robotElement) {
        // Get robot id
        std::string id = robotElement->Attribute("id");

        // Get robot max linear and angular velocities
        float max_linear = robotElement->FirstChildElement("max_velocities")->FloatAttribute("linear");
        float max_angular = robotElement->FirstChildElement("max_velocities")->FloatAttribute("angular");

        // Get robot radius and initial position
        float radius = std::stof(robotElement->FirstChildElement("radius")->GetText());
        float x = 0.0f, y = 0.0f, theta = 0.0f;
        tinyxml2::XMLElement* posElement = robotElement->FirstChildElement("position");
        if (posElement) {
            posElement->QueryFloatAttribute("x", &x);
            posElement->QueryFloatAttribute("y", &y);
            posElement->QueryFloatAttribute("theta", &theta);
        }

        // Create initial pose as an Isometry2f object
        Isometry2f initial_pose = Isometry2f::Identity();
        initial_pose.translation() << x, y;
        initial_pose.linear() = Eigen::Rotation2Df(theta).toRotationMatrix();

        // Create a new UnicyclePlatform object
        UnicyclePlatform* robot = new UnicyclePlatform(world, initial_pose);
        robot->tv = 0; // Initial linear velocity
        robot->rv = 0; // Initial angular velocity
        robot->radius = radius; // Set the robot's radius

        // Parse of lidar parameters
        tinyxml2::XMLElement* lidarElement = robotElement->FirstChildElement("lidar");

        // If the lidar element exists, parse its parameters
        if (lidarElement) {
            // Get the number of beams (or ranges)
            int num_beams = std::stoi(lidarElement->FirstChildElement("num_beams")->GetText());

            // Get the range parameters (min and max), defaulting to 0.1 and 10.0 if not specified
            float range_min = 0.1f, range_max = 10.0f;
            tinyxml2::XMLElement* rangeElement = lidarElement->FirstChildElement("range");
            if (rangeElement) {
                rangeElement->QueryFloatAttribute("min", &range_min);
                rangeElement->QueryFloatAttribute("max", &range_max);
            }

            // Get the frame ID for the lidar, defaulting to "laser" if not specified
            std::string lidar_frame = "laser";
            tinyxml2::XMLElement* frameElement = lidarElement->FirstChildElement("frame_id");
            if (frameElement && frameElement->GetText()) {
                lidar_frame = frameElement->GetText();
            }

            // Now create the LaserScan and LaserScanner objects
            LaserScan* scan = new LaserScan(range_min, range_max, -M_PI, M_PI, num_beams);

            // This is a lidar from grisetti, it has a fixed number of beams and a fixed range
            // and modifies the scan object with the new readings
            LaserScanner* scanner = new LaserScanner(*scan, *robot, Isometry2f::Identity());

            // set the robot laser scanner and scan [ADDED BY ME]
            robot->laser_scanner = scanner;
            robot->laser_scan = scan;

        }

        // Put the robot in the robots vector
        robots.push_back(robot);

        // Move to the next robot element in the XML
        robotElement = robotElement->NextSiblingElement("robot");
    }
    return robots;
}

int main(int argc, char **argv) {
    // Inizialize Ros Node called "robot_simulator"
    ros::init(argc, argv, "robot_simulator");

    // Create a NodeHandle to manage the node
    ros::NodeHandle n;

    // ROS publishers for the different topics:
    std::vector<ros::Publisher> odom_pubs;      // For publishing odometry (nav_msgs::Odometry)
    std::vector<ros::Publisher> scan_pubs;      // For publishing laser scans (sensor_msgs::LaserScan)
    std::vector<ros::Publisher> pose_pubs;      // For publishing robot pose (PoseStamped)
    std::vector<ros::Publisher> cmd_vel_pubs;   // For publishing velocity commands (TwistStamped)

    // Create a single tf2 transform broadcaster (used to send many transforms)
    tf2_ros::TransformBroadcaster tf_broadcaster;

    // Load the grid map from an image file using Grisetti's GridMap class
    cv::Mat img = cv::imread("map.png", cv::IMREAD_GRAYSCALE);
    int rows = img.rows, cols = img.cols;
    GridMap grid_map(GRID_RESOLUTION, rows, cols);
    grid_map.loadFromImage("map.png", GRID_RESOLUTION);

    // Create a World object to manage the grid map from Grisetti's World class
    World world(grid_map);

    // Create and publish the static map as an OccupancyGrid
    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

    // Create the OccupancyGrid message (a ROS message type for maps)
    nav_msgs::OccupancyGrid map_msg;

    // Fill the OccupancyGrid message with data from the grid map
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = "map";
    map_msg.info.resolution = grid_map.resolution();
    map_msg.info.width = grid_map.cols;
    map_msg.info.height = grid_map.rows;
    map_msg.data.resize(grid_map.rows * grid_map.cols);
    for (int row = 0; row < grid_map.rows; ++row) {
        for (int col = 0; col < grid_map.cols; ++col) {
            uint8_t value = grid_map(row, col);
            if (value < 127) {
                map_msg.data[row * grid_map.cols + col] = 100; // Occupied
            } else {
                map_msg.data[row * grid_map.cols + col] = 0;   // Free
            }
        }
    }
    map_msg.info.origin.position.x = grid_map.origin().x(); // From Grisetti's GridMap class
    map_msg.info.origin.position.y = grid_map.origin().y(); // From Grisetti's GridMap class
    map_msg.info.origin.position.z = 0.0;
    map_msg.info.origin.orientation.x = 0.0;
    map_msg.info.origin.orientation.y = 0.0;
    map_msg.info.origin.orientation.z = 0.0;
    map_msg.info.origin.orientation.w = 1.0;
    
    // Publish the map
    map_pub.publish(map_msg);

    // Optionally, compute the center of the grid in world coordinates
    //Vector2f grid_middle(GRID_COLS / 2, GRID_ROWS / 2);
    //Vector2f world_middle = grid_map.grid2world(grid_middle);

    // Print to console
    //std::cout << "Grid center in world coordinates: " << world_middle.transpose() << std::endl;
    //std::cout << "Grid center in grid coordinates: " << grid_middle.transpose() << std::endl;
    
    // Load robots from configuration file and put them in a vector of UnicyclePlatform pointers
    std::vector<UnicyclePlatform*> robots = parseRobotsFromXML(world, "config.xml");
    if (robots.empty()) {
        ROS_ERROR("No robots loaded from configuration.");
        return 1;
    }

    // Create visualization canvas
    Canvas canvas(rows, cols, CV_8UC1);
    std::cout << "Debug 8 " << std::endl;
    // Create publishers for each robot and put them in the vectors of publishers
    for (size_t i = 0; i < robots.size(); i++) {
        std::string robot_ns = "robot_" + std::to_string(i+1);
        odom_pubs.push_back(n.advertise<nav_msgs::Odometry>(robot_ns + "/odom", 1000));
        scan_pubs.push_back(n.advertise<sensor_msgs::LaserScan>(robot_ns + "/scan", 1000));
        pose_pubs.push_back(n.advertise<geometry_msgs::PoseStamped>(robot_ns + "/robot_pose", 1000));
        cmd_vel_pubs.push_back(n.advertise<geometry_msgs::TwistStamped>(robot_ns + "/cmd_vel", 1000));
    }

    // Set up the ros loop rate
    ros::Rate loop_rate(10);
    std::cout << "Debug 9 " << std::endl;

    // While the node is running, we will draw the robots and publish their data
    while (ros::ok()) {
        int key;
        // Draw the grid map and robots on the canvas
        grid_map.draw(canvas);

        // Loop through each robot, update its state, draw it, and publish its data
        for (size_t i = 0; i < robots.size(); i++) {
            // Get the robot i from the vector of robots
            UnicyclePlatform* robot = robots[i];

            float yaw;
            // Tell the Grisetti's UnicyclePlatform to tick and update its state
            robot->tick(0.1);
            robot->draw(canvas);

            ros::Time current_time = ros::Time::now();

            // Publish odometry message
            nav_msgs::Odometry odom_msg;
            odom_msg.header.stamp = current_time;
            odom_msg.header.frame_id = "map";
            odom_msg.child_frame_id = "base_link";
            odom_msg.pose.pose.position.x = robot->globalPose().translation()[0];
            odom_msg.pose.pose.position.y = - robot->globalPose().translation()[1];
            yaw = std::atan2(robot->globalPose().linear()(1,0), robot->globalPose().linear()(0,0));
            odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

            odom_pubs[i].publish(odom_msg);

            // Publish PoseStamped message with full 2D pose information
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.stamp = current_time;
            pose_msg.header.frame_id = "map";
            pose_msg.pose.position.x = robot->globalPose().translation()[0];
            pose_msg.pose.position.y = - robot->globalPose().translation()[1];
            pose_msg.pose.position.z = 0.0;
            yaw = std::atan2(robot->globalPose().linear()(1,0), robot->globalPose().linear()(0,0));
            pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            pose_pubs[i].publish(pose_msg);

            // Publish TF transform from "map" to the robot's frame using tf2
            geometry_msgs::TransformStamped robotTransform;
            robotTransform.header.stamp = current_time;
            robotTransform.header.frame_id = "map";
            robotTransform.child_frame_id = "robot_" + std::to_string(i+1);
            robotTransform.transform.translation.x = robot->globalPose().translation()[0];
            robotTransform.transform.translation.y = - robot->globalPose().translation()[1];
            robotTransform.transform.translation.z = 0.0;
            robotTransform.transform.rotation = tf::createQuaternionMsgFromYaw(yaw);
            tf_broadcaster.sendTransform(robotTransform);

            // Publish transform from the robot frame to its laser frame <<<
            geometry_msgs::TransformStamped laserTransform;
            laserTransform.header.stamp = current_time;
            laserTransform.header.frame_id = "robot_" + std::to_string(i+1);
            // We name the laser frame as "robot_X/laser" so each robot's laser is uniquely identified
            laserTransform.child_frame_id = "robot_" + std::to_string(i+1) + "/laser";
            // Identity transform: laser is at the same position as the robot
            laserTransform.transform.translation.x = 0.0;
            laserTransform.transform.translation.y = 0.0;
            laserTransform.transform.translation.z = 0.0;
            laserTransform.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);
            tf_broadcaster.sendTransform(laserTransform);

            // Take the current robot's laser scanner and publish its scan data
            LaserScanner* scanner = robot->laser_scanner;

            // If the robot has a laser scanner, we publish its scan data
            if (scanner) {
                // Make sure to update the scanner before publishing
                scanner->tick(0.1f);

                // Create a LaserScan message to publish
                sensor_msgs::LaserScan scan_msg;
                // Fill the LaserScan message with data from the scanner
                scan_msg.header.stamp = current_time;
                scan_msg.header.frame_id = "robot_" + std::to_string(i+1) + "/laser";  // O usa lidar_frame se lo memorizzi
                scan_msg.angle_min = scanner->scan.angle_min;
                scan_msg.angle_max = scanner->scan.angle_max;
                scan_msg.angle_increment = (scanner->scan.angle_max - scanner->scan.angle_min) / scanner->scan.ranges.size();
                scan_msg.range_min = scanner->scan.range_min;
                scan_msg.range_max = scanner->scan.range_max;
                scan_msg.ranges = scanner->scan.ranges;
                // Publish the LaserScan message
                scan_pubs[i].publish(scan_msg);
            }


            // Publish Velocity Command (TwistStamped) message; here we publish the current velocities
            geometry_msgs::TwistStamped cmd_vel_msg;
            cmd_vel_msg.header.stamp = current_time;
            cmd_vel_msg.header.frame_id = "robot_" + std::to_string(i+1);
            cmd_vel_msg.twist.linear.x = robot->tv; // Linear velocity in the x direction
            cmd_vel_msg.twist.angular.z = robot->rv; // Angular velocity around the z axis
            cmd_vel_pubs[i].publish(cmd_vel_msg);
        }

        // Set keyboard input for controlling the robots
        if (key != -1) {
            switch (key) {
                // Arrows are used for robot 1
                case 81: // left
                    robots[0]->rv += 0.1;
                    std::cout << "robot 1 rv: " << robots[0]->rv << std::endl;
                    break;
                case 82: // up
                    robots[0]->tv += 0.1;
                    std::cout << "robot 1 tv: " << robots[0]->tv << std::endl;
                    break;
                case 83: // right
                    robots[0]->rv -= 0.1;
                    std::cout << "robot 1 rv: " << robots[0]->rv << std::endl;
                    break;
                case 84: // down
                    robots[0]->tv -= 0.1;
                    std::cout << "robot 1 tv: " << robots[0]->tv << std::endl;
                    break;
        
                // WASD are used for robot 2
                case 'a': // left
                    robots[1]->rv += 0.1;
                    std::cout << "robot 2 rv: " << robots[1]->rv << std::endl;
                    break;
                case 'w': // up
                    robots[1]->tv += 0.1;
                    std::cout << "robot 2 tv: " << robots[1]->tv << std::endl;
                    break;
                case 'd': // right
                    robots[1]->rv -= 0.1;
                    std::cout << "robot 2 rv: " << robots[1]->rv << std::endl;
                    break;
                case 's': // down
                    robots[1]->tv -= 0.1;
                    std::cout << "robot 2 tv: " << robots[1]->tv << std::endl;
                    break;
                // If i press space, set all velocities to 0
                case ' ':
                    for (auto* robot : robots) {
                        robot->tv = 0;
                        robot->rv = 0;
                    }
                    std::cout << "All robots stopped." << std::endl;
                    break;
            }
        }

        
        key = showCanvas(canvas, 50);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Clean up dynamically allocated robots
    for (auto* robot : robots) {
        delete robot;
    }

    return 0;
}
