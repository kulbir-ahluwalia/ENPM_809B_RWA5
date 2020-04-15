//
// Created by zeid on 2/27/20.
//
#include "robot_controller.h"

/**
 * Constructor for the robot
 * Class attributes are initialized in the constructor init list
 * You can instantiate another robot by passing the correct parameter to the constructor
 */




RobotController::RobotController(std::string arm_id) :
robot_controller_nh_("/ariac/"+arm_id),
robot_controller_options("manipulator",
        "/ariac/"+arm_id+"/robot_description",
        robot_controller_nh_),
robot_move_group_(robot_controller_options) 



{   arm_id_=arm_id;
    //ROS_WARN(">>>>> RobotController");
    if (arm_id=="arm1")
        {kit_tray_="kit_tray_1";}
    if (arm_id=="arm2")
        {kit_tray_="kit_tray_2";}

    robot_move_group_.setPlanningTime(20);
    robot_move_group_.setNumPlanningAttempts(10);
    robot_move_group_.setPlannerId("RRTConnectkConfigDefault");
    robot_move_group_.setMaxVelocityScalingFactor(0.9);
    robot_move_group_.setMaxAccelerationScalingFactor(0.9);
    // robot_move_group_.setEndEffector("moveit_ee");
    robot_move_group_.allowReplanning(true);


    //--These are joint positions used for the home position
    // home_joint_pose_ = {0.0, 3.1, -1.1, 1.9, 3.9, 4.7, 0};
    //home_joint_pose_ = {0.5753406842498247, 5.933530060266838, -0.6150311670595325, 1.6234181541925308, 3.5494995694291624, 4.692497629212904, 2.819532850928649};
    home_joint_pose_ = {0.0,3.1,-1.1, 1.9, 3.9, 4.7, 0};
    conveyer_joint_pose_ = {-1.1800,0.0,-1.1, 1.9, 3.9, 4.7, 0};    
    
    if (arm_id_=="arm2")
        home_joint_pose_[0]-=0.5;
    
    flipping_joint_pose_ = {0.507,3.1,-1.1, 1.9, 3.9, 4.7, 0};

    //-- offset used for picking up parts
    //-- For the pulley_part, the offset is different since the pulley is thicker
    offset_ = 0.02;

    //--topic used to get the status of the gripper
    gripper_subscriber_ = gripper_nh_.subscribe(
            "/ariac/"+arm_id+"/gripper/state", 10, &RobotController::GripperCallback, this);

    quality_control_sensor_1_subscriber_ = gripper_nh_.subscribe(
        "/ariac/quality_control_sensor_1", 10, &RobotController::QualityControlSensor1Callback, this);
    
    //add arm_id over here
    joint_states_subscriber_ = gripper_nh_.subscribe("/ariac/arm1/joint_states", 10,
                                                &RobotController::JointStatesCallback, this);

    joint_states_subscriber_2_ = gripper_nh_.subscribe("/ariac/arm2/joint_states", 10,
                                                &RobotController::JointStatesCallback_2, this);

    //SendRobotHome();

    robot_tf_listener_.waitForTransform(arm_id+"_linear_arm_actuator", arm_id+"_ee_link",
                                            ros::Time(0), ros::Duration(10));
    robot_tf_listener_.lookupTransform("/"+arm_id+"_linear_arm_actuator", "/"+arm_id+"_ee_link",
                                           ros::Time(0), robot_tf_transform_);


    fixed_orientation_.x = robot_tf_transform_.getRotation().x();
    fixed_orientation_.y = robot_tf_transform_.getRotation().y();
    fixed_orientation_.z = robot_tf_transform_.getRotation().z();
    fixed_orientation_.w = robot_tf_transform_.getRotation().w();

    tf::quaternionMsgToTF(fixed_orientation_,q);
    tf::Matrix3x3(q).getRPY(roll_def_,pitch_def_,yaw_def_);


    end_position_ = home_joint_pose_;
    end_position_[0] = 2.2;             //This is for agv1
    //end_position_[1] = 1.55;             //This is for agv1
//    end_position_[1] = 4.5;
//    end_position_[2] = 1.2;


    robot_tf_listener_.waitForTransform("world", arm_id+"_ee_link", ros::Time(0),
                                            ros::Duration(10));
    robot_tf_listener_.lookupTransform("/world", "/"+arm_id+"_ee_link", ros::Time(0),
                                           robot_tf_transform_);

    home_cart_pose_.position.x = robot_tf_transform_.getOrigin().x();
    home_cart_pose_.position.y = robot_tf_transform_.getOrigin().y();
    home_cart_pose_.position.z = robot_tf_transform_.getOrigin().z();
    home_cart_pose_.orientation.x = robot_tf_transform_.getRotation().x();
    home_cart_pose_.orientation.y = robot_tf_transform_.getRotation().y();
    home_cart_pose_.orientation.z = robot_tf_transform_.getRotation().z();
    home_cart_pose_.orientation.w = robot_tf_transform_.getRotation().w();

    agv_tf_listener_.waitForTransform("world", kit_tray_,
                                      ros::Time(0), ros::Duration(10));
    agv_tf_listener_.lookupTransform("/world", "/"+kit_tray_,
                                     ros::Time(0), agv_tf_transform_);
    agv_position_.position.x = agv_tf_transform_.getOrigin().x();
    agv_position_.position.y = agv_tf_transform_.getOrigin().y();
    agv_position_.position.z = agv_tf_transform_.getOrigin().z() + 4 * offset_;

    gripper_client_ = robot_controller_nh_.serviceClient<osrf_gear::VacuumGripperControl>(
            "/ariac/"+arm_id+"/gripper/control");
    counter_ = 0;
    drop_flag_ = false;
}

RobotController::~RobotController() {}

/**
 *
 * @return
 */

void RobotController::JointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_states){
    //ROS_WARN(">>>>> OrderCallback");
    auto positions=joint_states->position;
    std::vector <int>indices={1,3,2,0,4,5,6};
    joint_states_.clear();
    for(int i=0;i<indices.size();i++){
        joint_states_.push_back(positions[indices[i]]);
    }
}

geometry_msgs::Pose RobotController::GetEEPose(){
    geometry_msgs::Pose p;
    robot_tf_listener_.waitForTransform("world", arm_id_+"_ee_link",
                                            ros::Time(0), ros::Duration(10));
    robot_tf_listener_.lookupTransform("/world", "/"+arm_id_+"_ee_link",
                                           ros::Time(0), robot_tf_transform_);


    p.position.x= robot_tf_transform_.getOrigin().x();
    p.position.y= robot_tf_transform_.getOrigin().y();
    p.position.z= robot_tf_transform_.getOrigin().z();
    
    return p;

}

std::vector<double> RobotController::GetJointStates(){
    ros::spinOnce();
    if (arm_id_=="arm1"){
        return joint_states_;

    }
    if (arm_id_=="arm2"){
        return joint_states_2_;
    }

}


void RobotController::JointStatesCallback_2(const sensor_msgs::JointState::ConstPtr& joint_states){
    //ROS_WARN(">>>>> OrderCallback");
    auto positions=joint_states->position;
    std::vector <int>indices={1,3,2,0,4,5,6};
    joint_states_2_.clear();
    for(int i=0;i<indices.size();i++){
        joint_states_2_.push_back(positions[indices[i]]);
    }
}


void RobotController::QualityControlSensor1Callback(const osrf_gear::LogicalCameraImage::ConstPtr & msg){

if((*msg).models.size()!=0)
    {quality_control_sensor_1_flag_=true;
    defected_part_=(*msg).models[0].type;}    //Bad Quality Detected

}

bool RobotController::GetQualityControl1Status(){
return quality_control_sensor_1_flag_;

}

bool RobotController::Planner() {
    //ROS_INFO_STREAM("Planning started...");
    if (robot_move_group_.plan(robot_planner_) ==
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        plan_success_ = true;
        //ROS_INFO_STREAM("Planner succeeded!");
    } else {
        plan_success_ = false;
        ROS_WARN_STREAM("Planner failed!");
    }

    return plan_success_;
}


void RobotController::Execute() {
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        //ros::Duration(0.5).sleep();
    }
}

void RobotController::GoToTarget(const geometry_msgs::Pose& pose) {
    target_pose_.orientation = fixed_orientation_;
    target_pose_.position = pose.position;
    ros::AsyncSpinner spinner(4);
    robot_move_group_.setPoseTarget(target_pose_);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        //ros::Duration(0.5).sleep();
    }
    //ROS_INFO_STREAM("Point reached...");
}

void RobotController::GoToTarget(
        std::initializer_list<geometry_msgs::Pose> list) {
    ros::AsyncSpinner spinner(4);
    spinner.start();

    std::vector<geometry_msgs::Pose> waypoints;
    for (auto i : list) {
        i.orientation.x = fixed_orientation_.x;
        i.orientation.y = fixed_orientation_.y;
        i.orientation.z = fixed_orientation_.z;
        i.orientation.w = fixed_orientation_.w;
        waypoints.emplace_back(i);
    }

    moveit_msgs::RobotTrajectory traj;
    auto fraction =
            robot_move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true);

    ROS_WARN_STREAM("Fraction: " << fraction * 100);
    //ros::Duration(5.0).sleep();
    //ros::Duration(0.5).sleep();

    robot_planner_.trajectory_ = traj;

    //if (fraction >= 0.3) {
        robot_move_group_.execute(robot_planner_);
        //ros::Duration(5.0).sleep();
        //ros::Duration(0.5).sleep();
//    } else {
//        ROS_ERROR_STREAM("Safe Trajectory not found!");
//    }
}


void RobotController::GoToTarget_with_orientation(const geometry_msgs::Pose& pose) {
    target_pose_.orientation = pose.orientation;
    target_pose_.position = pose.position;
    ros::AsyncSpinner spinner(4);
    robot_move_group_.setPoseTarget(target_pose_);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        //ros::Duration(0.5).sleep();
    }
    //ROS_INFO_STREAM("Point reached...");
}


void RobotController::GoToTarget_with_orientation(
        std::initializer_list<geometry_msgs::Pose> list) {
    ros::AsyncSpinner spinner(4);
    spinner.start();

    std::vector<geometry_msgs::Pose> waypoints;
    for (auto i : list) {
        i.orientation.x = i.orientation.x;
        i.orientation.y = i.orientation.y;
        i.orientation.z = i.orientation.z;
        i.orientation.w = i.orientation.w;
        waypoints.emplace_back(i);
    }

    moveit_msgs::RobotTrajectory traj;
    auto fraction =
            robot_move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true);

    ROS_WARN_STREAM("Fraction: " << fraction * 100);
    //ros::Duration(5.0).sleep();
    //ros::Duration(0.5).sleep();

    robot_planner_.trajectory_ = traj;

    //if (fraction >= 0.3) {
        robot_move_group_.execute(robot_planner_);
        //ros::Duration(5.0).sleep();
        //ros::Duration(0.5).sleep();
//    } else {
//        ROS_ERROR_STREAM("Safe Trajectory not found!");
//    }
}

void RobotController::SendRobotHome() {
    // ros::Duration(2.0).sleep();
    ROS_INFO_STREAM("Sending robot home");
    robot_move_group_.setJointValueTarget(home_joint_pose_);
    // this->execute();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(0.5).sleep();
    }

     //ros::Duration(2.0).sleep();
}


void RobotController::SendRobotJointPosition(std::vector<double> joint_states) {
    // ros::Duration(2.0).sleep();
    ROS_INFO_STREAM("Sending robot to specified joint_position");
    robot_move_group_.setJointValueTarget(joint_states);
    // this->execute();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(0.5).sleep();
    }

     //ros::Duration(2.0).sleep();
}


void RobotController::SendRobotConveyer() {
    // ros::Duration(2.0).sleep();
    ROS_INFO_STREAM("Sending robot Conveyer");
    robot_move_group_.setJointValueTarget(conveyer_joint_pose_);
    // this->execute();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        //ros::Duration(0.5).sleep();
    }

     //ros::Duration(2.0).sleep();
}


void RobotController::SendRobotFlipping() {
    // ros::Duration(2.0).sleep();
    ROS_INFO_STREAM("Sending robot Flipping");
    robot_move_group_.setJointValueTarget(flipping_joint_pose_);
    // this->execute();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(0.5).sleep();
    }

     //ros::Duration(2.0).sleep();
}

void RobotController::GripperToggle(const bool& state) {
    ROS_INFO_STREAM("Actuating the gripper :"<<state);
    gripper_service_.request.enable = state;
    gripper_client_.call(gripper_service_);
    ros::Duration(0.5).sleep();
    // if (gripper_client_.call(gripper_service_)) {
    if (gripper_service_.response.success) {
        //ROS_INFO_STREAM("Gripper activated!");
    } else {
        //ROS_WARN_STREAM("Gripper activation failed!");
    }
}

// bool RobotController::dropPart(geometry_msgs::Pose part_pose) {
//   counter_++;
//
//   pick = false;
//   drop = true;
//
//   ROS_WARN_STREAM("Dropping the part number: " << counter_);
//
//   // ROS_INFO_STREAM("Moving to end of conveyor...");
//   // robot_move_group_.setJointValueTarget(part_pose);
//   // this->execute();
//   // ros::Duration(1.0).sleep();
//   // this->gripper_state_check(part_pose);
//
//   if (drop == false) {
//     // ROS_INFO_STREAM("I am stuck here..." << object);
//     ros::Duration(2.0).sleep();
//     return drop;
//   }
//   ROS_INFO_STREAM("Dropping on AGV...");
//
//   // agv_position_.position.x -= 0.1;
//   // if (counter_ == 1) {
//   //   agv_position_.position.y -= 0.1;
//   // }
//   // if (counter_ >= 2) {
//   //   agv_position_.position.y += 0.1;
//   //   // agv_position_.position.x +=0.1;
//   // }
//
//   auto temp_pose = part_pose;
//   // auto temp_pose = agv_position_;
//   temp_pose.position.z += 0.35;
//   // temp_pose.position.y += 0.5;
//
//   // this->setTarget(part_pose);
//   // this->execute();
//   // ros::Duration(1.0).sleep();
//   this->goToTarget({temp_pose, part_pose});
//   ros::Duration(1).sleep();
//   ROS_INFO_STREAM("Actuating the gripper...");
//   this->gripperToggle(false);
//
//   // ROS_INFO_STREAM("Moving to end of conveyor...");
//   // robot_move_group_.setJointValueTarget(end_position_);
//   // this->execute();
//   // ros::Duration(1.0).sleep();
//
//   ROS_INFO_STREAM("Going to home...");
//   // this->sendRobotHome();
//   // temp_pose = home_cart_pose_;
//   // temp_pose.position.z -= 0.05;
//   this->goToTarget({temp_pose, home_cart_pose_});
//   return drop;
// }

int RobotController::DropPart(geometry_msgs::Pose part_pose) {
    // counter_++;

    drop_flag_ = true;

    ros::spinOnce();
    ROS_INFO_STREAM("Placing phase activated...");
    if (gripper_state_==false) return -1;
    if (gripper_state_){//--while the part is still attached to the gripper
        //--move the robot to the end of the rail
         
        while(1)
        {    ROS_INFO_STREAM("Moving towards AGV1...");
             robot_move_group_.setJointValueTarget(end_position_);
             this->Execute();
             ros::spinOnce();
             ROS_INFO_STREAM("diff  "<<abs(joint_states_[0]- 2.2 ));
             if (joint_states_[0]-2.2<0.1)
                break;
        }
         
        //ros::Duration(1).sleep();
         //ROS_INFO_STREAM("Actuating the gripper...");
         //this->GripperToggle(false);
        ros::spinOnce();
        if (gripper_state_==false) return -1; 
        auto temp_pose = part_pose;
        temp_pose.position.z += 0.5;
        ROS_INFO_STREAM("Going to keep part");
        this->GoToTarget({temp_pose, part_pose});
        ros::Duration(0.5).sleep();
        ros::spinOnce();

        if (gripper_state_==false) return -1;
        //Quality Control Sensor
        if (GetQualityControl1Status()==true)      //If true bad quality
            {return -1;

            }

        //ROS_INFO_STREAM("Actuating the gripper...");
        this->GripperToggle(false);

        ros::spinOnce();
        if (!gripper_state_) {
            ROS_INFO_STREAM("Going to home position...");
            this->GoToTarget({temp_pose, home_cart_pose_});
            //ros::Duration(1.0).sleep();
        }
    }
    // robot_move_group_.setJointValueTarget(home_joint_pose_);
    // this->Execute();
    ros::Duration(0.5).sleep();
    drop_flag_ = false;
    int temp=0;
    if (gripper_state_==1)
        {temp=1;}
    return temp;
}

void RobotController::GripperCallback(
        const osrf_gear::VacuumGripperState::ConstPtr& grip) {
    gripper_state_ = grip->attached;
}


bool RobotController::PickPart(geometry_msgs::Pose& part_pose) {
    // gripper_state = false;
    // pick = true;
    //ROS_INFO_STREAM("fixed_orientation_" << part_pose.orientation = fixed_orientation_);
    //ROS_WARN_STREAM("Picking the part...");

    
    ros::spinOnce();
       

        
    ROS_INFO_STREAM("Moving to part...");
    part_pose.position.z = part_pose.position.z + offset_;
    auto temp_pose_1 = part_pose;
    temp_pose_1.position.z += 0.3;

    this->GoToTarget({temp_pose_1, part_pose});

    //ROS_INFO_STREAM("Actuating the gripper..." << part_pose.position.z);
    this->GripperToggle(true);
    ros::spinOnce();
    while (!gripper_state_) {
        part_pose.position.z -= 0.005;
        this->GoToTarget({temp_pose_1, part_pose});
        //ROS_INFO_STREAM("Actuating the gripper...");
        this->GripperToggle(true);
        ros::spinOnce();
    }

    //ROS_INFO_STREAM("Going to waypoint...");
    this->GoToTarget(temp_pose_1);
    return gripper_state_;
}
