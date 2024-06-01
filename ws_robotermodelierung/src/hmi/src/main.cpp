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

void displayRobotControlPanel(moveit::planning_interface::MoveGroupInterface& move_group_interface, std::vector<geometry_msgs::Pose>& waypoints, moveit_msgs::RobotTrajectory& trajectory, const std::string& robot_name, int &speed, GLFWwindow* window) {

    ImGui::Text("Control panel for %s", robot_name.c_str());
    
    ImGui::SetCursorPos(ImVec2(412, 50));

    int button_size = 50;
    int padding = 20;
    int startX = 250; 
    int startY = 150;
    
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
    
    
    //Stepsize Slider
    ImGui::SetCursorPos(ImVec2(100, 350)); // (1024/2 - slider width/2, desired vertical position)
    ImGui::SliderInt("Stepsize (%)", &speed, 1, 100);   
    // ImGui::SetCursorPos(ImVec2(5,5));
    // ImGui::Text("%.2f",ImGui::GetIO().Framerate);
    // ROS_INFO_STREAM("ALL OKAY HERE IN FUNCTOIN");
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
    GLFWwindow* window = glfwCreateWindow(1024, 768, "Example HMI for Robotermodellierung", NULL, NULL);
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
    io.FontGlobalScale = 1.5f;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
    ImVec4 clear_color = ImVec4(0.40f, 0.50f, 0.60f, 0.45f);
    
    
    //ROS Stuff
    
    ros::init(argc, argv, "hmi");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    static const std::string PLANNING_GROUP_1 = "yuzi_robot";
    static const std::string PLANNING_GROUP_2 = "coco_robot";
    // static const std::string PLANNING_GROUP_3 = "alex_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface_1(PLANNING_GROUP_1);
    moveit::planning_interface::MoveGroupInterface move_group_interface_2(PLANNING_GROUP_2);
    // moveit::planning_interface::MoveGroupInterface move_group_interface_3(PLANNING_GROUP_3);
    std::vector<geometry_msgs::Pose> waypoints_1, waypoints_2, waypoints_3;
    moveit_msgs::RobotTrajectory trajectory_1, trajectory_2, trajectory_3;
    int speed = 1;
   
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

        ImGui::SetNextWindowSize(ImVec2(700,500));
	    ImGui::SetNextWindowPos(ImVec2(50,50));
        
        //Start of the Window
        if (ImGui::Begin("Robot HMI" , NULL, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | 4 | 256 | 1))
        {
            if (ImGui::BeginTabBar("Robots Control")) 
            {
                if (ImGui::BeginTabItem("Robot 1")) 
                {
                    displayRobotControlPanel(move_group_interface_1, waypoints_1, trajectory_1, "yuzi_robot", speed, window);
                    ImGui::EndTabItem();  
                }
                if (ImGui::BeginTabItem("Robot 2")) 
                {
                    displayRobotControlPanel(move_group_interface_2, waypoints_2, trajectory_2, "coco_robot", speed, window);
                    ImGui::EndTabItem();  
                }
                if (ImGui::BeginTabItem("Robot 3")) 
                {
                    // displayRobotControlPanel(move_group_interface_3, waypoints_3, trajectory_3, "alex_robot", speed, window);
                    ImGui::EndTabItem();  
                }
                ImGui::EndTabBar();  
            }

            ImGui::SetCursorPos(ImVec2(600, 400)); 
            if(ImGui::Button("Close", ImVec2(70, 70))) {
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