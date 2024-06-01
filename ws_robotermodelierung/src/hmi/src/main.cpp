#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> 
#include <iostream>
#include <string>
#include <regex>

//ROS and Moveit
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>




#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif
   

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

void saveJointPosition(moveit::planning_interface::MoveGroupInterface& move_group, std::string position, std::string robot_name, int position_number) {
    
    // Get current joint values
    std::vector<double> joint_values = move_group.getCurrentJointValues();

    // File path where you want to save the joint positions
    std::string file_path = "/home/cocokayya18/robotermodelierung/ws_robotermodelierung/src/hmi/SaveFiles/joint_positions_" + robot_name + ".yaml";

    // Open file in append mode
    std::ofstream file(file_path, std::ios::app);

    if (file.is_open()) {
        // Write joint values to file in YAML format
        file << "position_" << position_number << ":\n";
        file << "    pose_name: \"" << position << "\"\n";
        file << "    joints: [";
        for (size_t i = 0; i < joint_values.size(); ++i) {
            file << std::fixed << std::setprecision(3) << joint_values[i];
            if (i != joint_values.size() - 1) {
                file << ", ";
            }
        }
        file << "]\n\n"; 
        file.close();
        std::cout << "Joint positions saved successfully to " << file_path << std::endl;
    } else {
        std::cerr << "Failed to open file at " << file_path << std::endl;
    }
}

void saveAbsolutePosition(moveit::planning_interface::MoveGroupInterface& move_group, std::string position, std::string robot_name, int position_number) {
    
    // Get current joint values
    geometry_msgs::PoseStamped absolut_values = move_group.getCurrentPose();

    // File path where you want to save the joint positions
    std::string file_path = "/home/cocokayya18/robotermodelierung/ws_robotermodelierung/src/hmi/SaveFiles/absolute_positions_" + robot_name + ".yaml";

    // Open file in append mode
    std::ofstream file(file_path, std::ios::app);

    int precision = 3;

    if (file.is_open()) {
        // Write the position data in YAML format
        file << "position_" << position_number << ":\n";
        file << "    pose_name: \"" << position << "\"\n";
        file << "    pose:\n";
        file << "        position:\n";
        file << "            x: " << std::fixed << std::setprecision(precision) << absolut_values.pose.position.x << "\n";
        file << "            y: " << std::fixed << std::setprecision(precision) << absolut_values.pose.position.y << "\n";
        file << "            z: " << std::fixed << std::setprecision(precision) << absolut_values.pose.position.z << "\n";
        file << "        orientation:\n";
        file << "            x: " << std::fixed << std::setprecision(precision) << absolut_values.pose.orientation.x << "\n";
        file << "            y: " << std::fixed << std::setprecision(precision) << absolut_values.pose.orientation.y << "\n";
        file << "            z: " << std::fixed << std::setprecision(precision) << absolut_values.pose.orientation.z << "\n";
        file << "            w: " << std::fixed << std::setprecision(precision) << absolut_values.pose.orientation.w << "\n";
        file << "\n"; // Ensure there's a newline at the end of the entry for better readability
        file.close();
        std::cout << "Absolute positions saved successfully to " << file_path << std::endl;
    } else {
        std::cerr << "Failed to open file at " << file_path << std::endl;
    }
}

void adjustOrientation(moveit::planning_interface::MoveGroupInterface& move_group, double roll, double pitch, double yaw) {
    const auto& end_effector_link = move_group.getEndEffectorLink();
    auto current_pose = move_group.getCurrentPose(end_effector_link).pose;

    tf2::Quaternion q_orig, q_rot, q_new;

    // Manually converting from geometry_msgs::Quaternion to tf2::Quaternion
    q_orig.setX(current_pose.orientation.x);
    q_orig.setY(current_pose.orientation.y);
    q_orig.setZ(current_pose.orientation.z);
    q_orig.setW(current_pose.orientation.w);

    // Set the rotation
    q_rot.setRPY(roll, pitch, yaw);

    // Combine the rotations
    q_new = q_orig * q_rot;
    q_new.normalize();

    // Manually converting from tf2::Quaternion back to geometry_msgs::Quaternion
    geometry_msgs::Quaternion new_orientation;
    new_orientation.x = q_new.x();
    new_orientation.y = q_new.y();
    new_orientation.z = q_new.z();
    new_orientation.w = q_new.w();

    current_pose.orientation = new_orientation;

    move_group.setPoseTarget(current_pose);
    move_group.move();
}

/* void saveTrajectoryPosition()
{
    moveit_msgs::RobotTrajectory trajectory;

     // File path where you want to save the joint positions
    std::string file_path = "/home/cocokayya18/robotermodelierung/ws_robotermodelierung/src/hmi/SaveFiles/trajectories_" + robot_name + ".yaml";

    std::ofstream file(file_path, std::ios::app);

    if (file.is_open()) {
        file << "trajectory_name: \"" << trajectory_name << "\"\n";
        file << "joint_trajectory:\n";
        file << "  joint_names: [";
        for (size_t i = 0; i < trajectory.joint_trajectory.joint_names.size(); ++i) {
            file << "\"" << trajectory.joint_trajectory.joint_names[i] << "\"";
            if (i != trajectory.joint_trajectory.joint_names.size() - 1) file << ", ";
        }
        file << "]\n";
        file << "  points:\n";

        for (const auto& point : trajectory.joint_trajectory.points) {
            file << "    - positions: [";
            for (size_t i = 0; i < point.positions.size(); ++i) {
                file << std::fixed << std::setprecision(3) << point.positions[i];
                if (i != point.positions.size() - 1) file << ", ";
            }
            file << "]\n";
            file << "      velocities: [";
            for (size_t i = 0; i < point.velocities.size(); ++i) {
                file << std::fixed << std::setprecision(3) << point.velocities[i];
                if (i != point.velocities.size() - 1) file << ", ";
            }
            file << "]\n";
            file << "      accelerations: [";
            for (size_t i = 0; i < point.accelerations.size(); ++i) {
                file << std::fixed << std::setprecision(3) << point.accelerations[i];
                if (i != point.accelerations.size() - 1) file << ", ";
            }
            file << "]\n";
            file << "      effort: [";
            for (size_t i = 0; i < point.effort.size(); ++i) {
                file << std::fixed << std::setprecision(3) << point.effort[i];
                if (i != point.effort.size() - 1) file << ", ";
            }
            file << "]\n";
            file << "      time_from_start: " << point.time_from_start.toSec() << "s\n";
        }

        file << "\n";  // Add a newline for better readability
        file.close();
        std::cout << "Trajectory '" << trajectory_name << "' saved successfully to " << file_path << std::endl;
    } else {
        std::cerr << "Failed to open file at " << file_path << std::endl;
    }
} */

void displayRobotControlPanel(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::vector<geometry_msgs::Pose>& waypoints, moveit_msgs::RobotTrajectory& trajectory, const std::string& robot_name, int &speed, GLFWwindow* window, std::vector<int>& positionNumbers) {

    ImGui::Text("Control panel for %s", robot_name.c_str());
    
    ImGui::SetCursorPos(ImVec2(412, 50));

    int button_size = 50;
    int padding = 20;
    int startX = 120; 
    int startY = 150;

    static char positionBuffer[256] = "";
    std::string positionStr = std::string(positionBuffer);
    
    //X+ Button
    ImGui::SetCursorPos(ImVec2(startX, startY));
    if(ImGui::Button("X+", ImVec2(button_size, button_size)))
    {
        geometry_msgs::PoseStamped cp = move_group_interface.getCurrentPose();
        waypoints.clear();
        geometry_msgs::Pose target = cp.pose;
        target.position.x += 0.001 * speed;
        waypoints.push_back(target);
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        move_group_interface.asyncExecute(trajectory);          
    }

    //X- Button
    ImGui::SetCursorPos(ImVec2(startX, startY + button_size + padding));
    if(ImGui::Button("X-", ImVec2(button_size, button_size)))
    {
        geometry_msgs::PoseStamped cp = move_group_interface.getCurrentPose();
        waypoints.clear();
        geometry_msgs::Pose target = cp.pose;
        target.position.x -= 0.001 * speed;
        waypoints.push_back(target);
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        move_group_interface.asyncExecute(trajectory);  
    }
    
    //Y+ Button
    ImGui::SameLine();
    ImGui::SetCursorPos(ImVec2(startX + button_size + padding, startY));
    if(ImGui::Button("Y+", ImVec2(button_size, button_size)))
    {
        geometry_msgs::PoseStamped cp = move_group_interface.getCurrentPose();
        waypoints.clear();
        geometry_msgs::Pose target = cp.pose;
        target.position.y += 0.001 * speed;
        waypoints.push_back(target);
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        move_group_interface.asyncExecute(trajectory); 
    }

    //Y- Button
    ImGui::SetCursorPos(ImVec2(startX + button_size + padding, startY + button_size + padding));
    if(ImGui::Button("Y-", ImVec2(button_size, button_size)))
    {
        geometry_msgs::PoseStamped cp = move_group_interface.getCurrentPose();
        waypoints.clear();
        geometry_msgs::Pose target = cp.pose;
        target.position.y -= 0.001 * speed;
        waypoints.push_back(target);
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        move_group_interface.asyncExecute(trajectory);   
    }                        
    
    //Z+ Button
    ImGui::SetCursorPos(ImVec2(startX + 2 * (button_size + padding), startY));
    if(ImGui::Button("Z+", ImVec2(button_size, button_size)))
    {
        geometry_msgs::PoseStamped cp = move_group_interface.getCurrentPose();
        waypoints.clear();
        geometry_msgs::Pose target = cp.pose;
        target.position.z += 0.001 * speed;
        waypoints.push_back(target);
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        move_group_interface.asyncExecute(trajectory);  
    }
    
    //Z- Button
    ImGui::SetCursorPos(ImVec2(startX + 2 * (button_size + padding), startY + button_size + padding));
    if(ImGui::Button("Z-", ImVec2(button_size, button_size)))
    {
        geometry_msgs::PoseStamped cp = move_group_interface.getCurrentPose();
        waypoints.clear();
        geometry_msgs::Pose target = cp.pose;
        target.position.z -= 0.001 * speed;
        waypoints.push_back(target);
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        move_group_interface.asyncExecute(trajectory);  
    }

    ImGui::SetCursorPos(ImVec2(startX + 3 * (button_size + padding), startY));
    if (ImGui::Button("Roll +", ImVec2(button_size + 10, button_size))) 
    {
        adjustOrientation(move_group_interface, 0.05 * speed, 0.0, 0.0); 
    }

    ImGui::SetCursorPos(ImVec2(startX + 3 * (button_size + padding), startY + button_size + padding));
    if (ImGui::Button("Roll -", ImVec2(button_size + 10, button_size))) 
    {
        adjustOrientation(move_group_interface, -0.05 * speed, 0.0, 0.0); 
    }

    ImGui::SetCursorPos(ImVec2(startX + 4 * (button_size + padding) + 10, startY));
    if (ImGui::Button("Pitch +", ImVec2(button_size + 20, button_size))) 
    {
        adjustOrientation(move_group_interface, 0.0, 0.05 * speed, 0.0); 
    }

    ImGui::SetCursorPos(ImVec2(startX + 4 * (button_size + padding) + 10, startY + button_size + padding));
    if (ImGui::Button("Pitch -", ImVec2(button_size + 20, button_size))) 
    {
        adjustOrientation(move_group_interface, 0.0, -0.05 + speed, 0.0); 
    }

    ImGui::SetCursorPos(ImVec2(startX + 30 + 5 * (button_size + padding), startY));
    if (ImGui::Button("Yaw +", ImVec2(button_size + 15, button_size))) 
    {
        adjustOrientation(move_group_interface, 0.0, 0.0, 0.05 * speed); 
    }

    ImGui::SetCursorPos(ImVec2(startX + 30 + 5 * (button_size + padding), startY + button_size + padding));
    if (ImGui::Button("Yaw -", ImVec2(button_size + 15, button_size))) 
    {
        adjustOrientation(move_group_interface, 0.0, 0.0, -0.05 * speed); 
    }

    ImGui::SetCursorPos(ImVec2(110, 515));
    if (!ImGui::InputText("Position Name", positionBuffer, sizeof(positionBuffer))) 
    {
        // ROS_INFO_STREAM("InputText Modified: " << sizeof(positionBuffer));
    }

    // Save joint configuration button
    ImGui::SetCursorPos(ImVec2(150, 350)); 
    if (ImGui::Button("Save Joint Position", ImVec2(210, 70))) 
    {
        saveJointPosition(move_group_interface, positionStr, robot_name, positionNumbers[0]);
        positionNumbers[0] = positionNumbers[0] + 1; 
    }

    // Save absolute position button
    ImGui::SetCursorPos(ImVec2(150, 430)); 
    if (ImGui::Button("Save Absolute Position", ImVec2(210, 70))) {
        saveAbsolutePosition(move_group_interface, positionStr, robot_name, positionNumbers[1]); 
        positionNumbers[1] = positionNumbers[1] + 1;
    }

/*    Save Trajectory button
    ImGui::SetCursorPos(ImVec2(360, 350)); 
    if (ImGui::Button("Save Trajectory", ImVec2(150, 70))) {
        saveTrajectoryPosition(); 
    } */

    ImGui::SetCursorPos(ImVec2(470, 50));
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(139.0f/255.0f, 0.0f, 0.0f, 1.0f));  
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));            
    if (ImGui::Button("Stop Robot", ImVec2(210, 70))) 
    {   
        move_group_interface.stop();
    }
    ImGui::PopStyleColor(2);

    //Stepsize Slider
    ImGui::SetCursorPos(ImVec2(115, 300)); 
    ImGui::SliderInt("Stepsize (%)", &speed, 1, 100);   
}


// Main code
int main(int argc, char** argv)
{
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0 + GLSL 100
    const char* glsl_version = "#version 100";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
    // GL 3.2 + GLSL 150
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

    // Create window with graphics context
    GLFWwindow* window = glfwCreateWindow(1024, 768, "HMI for Robotermodellierung", NULL, NULL);
    if (window == NULL)
        return 1;
    glfwSetWindowSizeLimits(window, 700, 700, 700, 700);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.FontGlobalScale = 1.3f;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
    ImVec4 clear_color = ImVec4(0.40f, 0.50f, 0.60f, 0.45f);
    
    ros::init(argc, argv, "hmi");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    static const std::string PLANNING_GROUP_1 = "yuzi_robot";
    static const std::string PLANNING_GROUP_2 = "alex_robot";
    static const std::string PLANNING_GROUP_3 = "coco_robot_eef1";
    static const std::string PLANNING_GROUP_4 = "coco_robot_eef2";
    moveit::planning_interface::MoveGroupInterface move_group_interface_1(PLANNING_GROUP_1);
    moveit::planning_interface::MoveGroupInterface move_group_interface_2(PLANNING_GROUP_2);
    moveit::planning_interface::MoveGroupInterface move_group_interface_3(PLANNING_GROUP_3);
    moveit::planning_interface::MoveGroupInterface move_group_interface_4(PLANNING_GROUP_4);
    std::vector<geometry_msgs::Pose> waypoints_1, waypoints_2, waypoints_3, waypoints_4;
    moveit_msgs::RobotTrajectory trajectory_1, trajectory_2, trajectory_3, trajectory_4;
    int speed = 1;
    std::vector<int> yuzi_robot_positionNumbers = {1, 1, 1};
    std::vector<int> alex_robot_positionNumbers = {1, 1, 1};
    std::vector<int> coco_robot_eef1_positionNumbers = {1, 1, 1};
    std::vector<int> coco_robot_eef2_positionNumbers = {1, 1, 1};
   
    ///////////////
    // Main loop //    
    ///////////////
    
    while (!glfwWindowShouldClose(window))

    {
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowSize(ImVec2(700,550));
	    ImGui::SetNextWindowPos(ImVec2(50,50));
        
        //Start of the Window
        if (ImGui::Begin("Robot HMI" , NULL, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | 4 | 256 | 1))
        {
            if (ImGui::BeginTabBar("Robots Control")) 
            {
                if (ImGui::BeginTabItem("Robot 1")) 
                {
                    displayRobotControlPanel(move_group_interface_1, waypoints_1, trajectory_1, "yuzi_robot", speed, window, yuzi_robot_positionNumbers);
                    ImGui::EndTabItem();  
                }
                if (ImGui::BeginTabItem("Robot 2")) 
                {
                    displayRobotControlPanel(move_group_interface_2, waypoints_2, trajectory_2, "alex_robot", speed, window, alex_robot_positionNumbers);
                    ImGui::EndTabItem();  
                }
                if (ImGui::BeginTabItem("Robot 3")) 
                {
                    displayRobotControlPanel(move_group_interface_3, waypoints_3, trajectory_3, "coco_robot_eef1", speed, window, coco_robot_eef1_positionNumbers);
                    ImGui::EndTabItem();  
                }
                if (ImGui::BeginTabItem("Robot 4")) 
                {
                    displayRobotControlPanel(move_group_interface_4, waypoints_4, trajectory_4, "coco_robot_eef2", speed, window, coco_robot_eef2_positionNumbers);
                    ImGui::EndTabItem();  
                }
                ImGui::EndTabBar();  
            }

            ImGui::SetCursorPos(ImVec2(390, 400)); 
            if(ImGui::Button("Close", ImVec2(150, 70))) {
                glfwWindowShouldClose(window);
                break;
            }
            ImGui::End(); 
        }

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
        
    }

    // Cleanup
    ros::shutdown();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}