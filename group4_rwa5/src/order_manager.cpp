//
// Created by zeid on 2/27/20.
//

#include "order_manager.h"
#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>



//AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}
AriacOrderManager::AriacOrderManager()//: //arm1_("arm1")
{
    order_subscriber_ = order_manager_nh_.subscribe(
            "/ariac/orders", 10,
            &AriacOrderManager::OrderCallback, this);


    break_beam_subscriber_ = order_manager_nh_.subscribe(
            "/ariac/break_beam_1_change", 10,
            &AriacOrderManager::break_beam_callback, this);

    camera_4_subscriber_ = order_manager_nh_.subscribe("/ariac/logical_camera_4", 10,
                                                &AriacOrderManager::LogicalCamera4Callback, this);

    camera_1_subscriber_ = order_manager_nh_.subscribe("/ariac/logical_camera_1", 10,
                                                &AriacOrderManager::LogicalCamera1Callback, this);

    camera_5_subscriber_ = order_manager_nh_.subscribe("/ariac/logical_camera_8", 10,
                                                &AriacOrderManager::LogicalCamera5Callback, this);

    camera_5_2_subscriber_ = order_manager_nh_.subscribe("/ariac/logical_camera_9", 10,
                                                &AriacOrderManager::LogicalCamera5_2Callback, this);

    camera_repick_subscriber_ = order_manager_nh_.subscribe("/ariac/logical_camera_5", 10,
                                                &AriacOrderManager::LogicalCameraRepickCallback, this);


    joint_states_subscriber_ = order_manager_nh_.subscribe("/ariac/arm1/joint_states", 10,
                                                &AriacOrderManager::JointStatesCallback, this);

    joint_states_subscriber_2_ = order_manager_nh_.subscribe("/ariac/arm2/joint_states", 10,
                                                &AriacOrderManager::JointStatesCallback_2, this);

}


AriacOrderManager::~AriacOrderManager(){}


void AriacOrderManager::OrderCallback(const osrf_gear::Order::ConstPtr& order_msg) {
    ROS_WARN(">>>>> OrderCallback");
    received_orders_.push_back(*order_msg);
    if ((*order_msg).order_id.size()>13){
    	ROS_WARN("ORDER UPDATE RECEIVED.....");
    	order_update_flag_=1;
    }
}


void AriacOrderManager::break_beam_callback(const osrf_gear::Proximity::ConstPtr& msg) {
    

    double temp=ros::Time::now().toSec();
    if (flag_==1)
      {//ROS_INFO_STREAM("Time Delay observed"<<temp-camera_4_detection_time_);
}
    if (flag_==1     &&    temp-camera_4_detection_time_>4.50)
    {
    if(msg->object_detected) {
        //ROS_INFO("Part Required Detected by Break beam triggered..");
        flag_break_beam_ = 1;
        break_beam_time = ros::Time::now();
    }
  }
}
void AriacOrderManager::LogicalCamera4Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    //For conveyer pickup
    

    if (init_==0)
      return;

    for(int i=0; i<image_msg->models.size();i++){
       if (image_msg->models[i].type==camera_4_product_){
        //ROS_INFO_STREAM("Part Required Detected");
        flag_=1;
        camera_4_detection_time_=ros::Time::now().toSec();
        //ROS_INFO_STREAM("Camera Detected at "<<camera_4_detection_time_);
        geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
        StampedPose_in.header.frame_id = "/logical_camera_4_frame";
        StampedPose_in.pose = image_msg->models[i].pose;
        //ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        camera4_op_pose_=StampedPose_out.pose;
        //ROS_INFO_STREAM("part is at   "<<camera4_op_pose_);
       }

     }
     if (image_msg->models.size()==0)
      {flag_=0;}
    init_=1;}



void AriacOrderManager::LogicalCamera1Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
//For Flipping


if (init_cam1_==1){
	//ROS_INFO_STREAM("Pulley detected for flipping");
	//init_cam1_=0;
	pulley_on_bin_=false;
	 geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
	for(int i=0;i<image_msg->models.size();i++){
		if (image_msg->models[i].type=="pulley_part"   )// &&   abs(image_msg->models[i].pose.position.z-0.15)>0.1){  //to see if pulley is still standing
			//geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
        	{StampedPose_in.header.frame_id = "/logical_camera_1_frame";
       		StampedPose_in.pose = image_msg->models[i].pose;
        	//ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        	part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        	flipping_pulley_pose_=StampedPose_out.pose;
        	pulley_on_bin_=true;

			//flipping_pulley_pose_
        	}
		}
	}
}


void AriacOrderManager::LogicalCameraRepickCallback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
//For repick

//ROS_INFO_STREAM("HELLO  "<<init_repick_);
if (init_repick_ != 0){
	
	//ROS_INFO_STREAM("Camera loop");
	int flag=0;
	//ROS_INFO_STREAM("Pulley detected for flipping");
	//init_cam1_=0;
	//pulley_on_bin_=false;
	geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
	for(int i=0;i<image_msg->models.size();i++){
		if (image_msg->models[i].type==repick_part_   )// &&   abs(image_msg->models[i].pose.position.z-0.15)>0.1){  //to see if pulley is still standing
			//geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
        	{StampedPose_in.header.frame_id = "/logical_camera_5_frame";
       		StampedPose_in.pose = image_msg->models[i].pose;
        	//ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        	part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        	repick_pose_=StampedPose_out.pose;
        	//pulley_on_bin_=true;
        	flag=1;
        	//ROS_INFO_STREAM("PArt to Repicking found");
			//flipping_pulley_pose_
        	}
		}
	if (flag!=1)
		init_repick_=-1;
}
}



void AriacOrderManager::LogicalCamera5Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
	//for kit tray1
	if (init_cam5_==1)
		{
	cam5_list_.clear();
	
	for(int i=0;i<image_msg->models.size();i++){
		std::pair<std::string,geometry_msgs::Pose> product_type_pose_cam5;
		product_type_pose_cam5.first=image_msg->models[i].type;
		product_type_pose_cam5.second=image_msg->models[i].pose;
		//ROS_INFO_STREAM("  "<<product_type_pose_cam5.first);
		cam5_list_.push_back(product_type_pose_cam5);

		}
		ROS_INFO_STREAM("Making List of parts on kit tray 1 with these many components   "<<cam5_list_.size()  );
	}
	init_cam5_=0;
	repose_flag_=0;
	geometry_msgs::PoseStamped StampedPose_in,StampedPose_out; 
	if(init_cam5_2_==1){
		for(int i=0;i<image_msg->models.size();i++){
			StampedPose_in.header.frame_id = "/logical_camera_8_frame";
       		StampedPose_in.pose = image_msg->models[i].pose;
        	//ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        	part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        	repick_pose_=StampedPose_out.pose;
        	//ROS_INFO_STREAM("///////////////");
        	if (euler_distance(repick_pose_,reposition_part_pose_)<0.005   &&   reposition_part_type_==image_msg->models[i].type){
        		//ROS_INFO_STREAM("Part at right position detected sending back pose");
        		reposition_part_pose_back_=StampedPose_out.pose;
        		repose_flag_=1;

        	}
        }

	//init_cam5_2_=0;
	}
	

	}



void AriacOrderManager::LogicalCamera5_2Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
	//for kit tray1
	if (init_cam5_dash_==1)
		{
	cam5_list_dash_.clear();
	
	for(int i=0;i<image_msg->models.size();i++){
		std::pair<std::string,geometry_msgs::Pose> product_type_pose_cam5;
		product_type_pose_cam5.first=image_msg->models[i].type;
		product_type_pose_cam5.second=image_msg->models[i].pose;
		//ROS_INFO_STREAM("  "<<product_type_pose_cam5.first);
		cam5_list_dash_.push_back(product_type_pose_cam5);

		}
		ROS_INFO_STREAM("Making List of parts on kit tray 2 with these many components   "<<cam5_list_.size()  );
	}
	init_cam5_dash_=0;
	repose_flag_dash_=0;
	geometry_msgs::PoseStamped StampedPose_in,StampedPose_out; 
	if(init_cam5_2_dash_==1){
		for(int i=0;i<image_msg->models.size();i++){
			StampedPose_in.header.frame_id = "/logical_camera_9_frame";
       		StampedPose_in.pose = image_msg->models[i].pose;
        	//ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        	part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        	repick_pose_dash_=StampedPose_out.pose;
        	//ROS_INFO_STREAM("///////////////");
        	if (euler_distance(repick_pose_dash_,reposition_part_pose_dash_)<0.005   &&   reposition_part_type_dash_==image_msg->models[i].type){
        		//ROS_INFO_STREAM("Part at right position detected sending back pose");
        		reposition_part_pose_back_dash_=StampedPose_out.pose;
        		repose_flag_dash_=1;

        	}
        }

	//init_cam5_2_dash_=0;
	}
	

	}

void AriacOrderManager::JointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_states){
    //ROS_WARN(">>>>> OrderCallback");
    auto positions=joint_states->position;
    std::vector <int>indices={1,3,2,0,4,5,6};
    joint_states_.clear();
    for(int i=0;i<indices.size();i++){
    	joint_states_.push_back(positions[indices[i]]);
    }
}


void AriacOrderManager::JointStatesCallback_2(const sensor_msgs::JointState::ConstPtr& joint_states){
    //ROS_WARN(">>>>> OrderCallback");
    auto positions=joint_states->position;
    std::vector <int>indices={1,3,2,0,4,5,6};
    joint_states_2_.clear();
    for(int i=0;i<indices.size();i++){
    	joint_states_2_.push_back(positions[indices[i]]);
    }
}

/**
 * @brief Get the product frame for a product type
 * @param product_type
 * @return
 */
std::string AriacOrderManager::GetProductFrame(std::string product_type) {
    //--Grab the last one from the list then remove it
    if (!product_frame_list_.empty()) {
        std::string frame = product_frame_list_[product_type].back();
        ROS_INFO_STREAM("Frame >>>> " << frame);
        product_frame_list_[product_type].pop_back();
        return frame;
    } else {
        ROS_ERROR_STREAM("No product frame found for " << product_type);
        ros::shutdown();
    }
}



int AriacOrderManager::GetFromArm2(std::string product_type,geometry_msgs::Pose part_pose,std::string product_frame,int agv_id){
	int arm_id;
	std::string arm1,arm_dash1;
	
	if (agv_id==1){
		arm1={"arm1"};
		arm_dash1={"arm2"};
		arm_id=2;

	}

	if (agv_id==2){
		arm1={"arm2"};
		arm_dash1={"arm1"};
		arm_id=2;
	}

	RobotController arm(arm1);
	RobotController arm_dash(arm_dash1);
	
	bool failed_pick = arm_dash.PickPart(part_pose);
    	ROS_WARN_STREAM("Picking up state arm2" << failed_pick);
    	ros::Duration(0.5).sleep();

    	while(!failed_pick){
        	auto part_pose = camera_.GetPartPose("/world",product_frame);
        	failed_pick = arm_dash.PickPart(part_pose);
        	//n=n+1;
        	ROS_INFO_STREAM("Attempt no to pick up part");
        	

    	}
	//Part has been picked up by arm2
    	if(arm_dash.GetGripperStatus()==false){
    		ROS_INFO_STREAM("Arm2 has dropped part");
    		return -1;
    	}

    	arm.SendRobotHome();
	    auto place_pose=part_pose;
	    place_pose.position.x=0.31292;
	    place_pose.position.y=-0.5954;
	    place_pose.position.z=0.9499;
	    place_pose.position.z+=0.1;
	    if(product_type == "pulley_part")
        	place_pose.position.z += 0.08;
	   	
	   	if(arm_id==2){ 
	    	home_joint_pose_[1]=1.55;
	    	home_joint_pose_[0]-=0.4;
		}
		if(arm_id==1){ 
	    	home_joint_pose_[1]=3.1+1.55;
	    	home_joint_pose_[0]-=0.4;
		}
	    while(1){         
		arm_dash.SendRobotJointPosition(home_joint_pose_);ros::spinOnce();
		ROS_INFO_STREAM("Diff is "<<arm_dash.GetJointStates()[0]-home_joint_pose_[0]);
		if (arm_dash.GetJointStates()[0]-home_joint_pose_[0]<0.1 )
			break;
		}
		 
		home_joint_pose_[1]=3.1;
    	home_joint_pose_[0]+=0.4;
		

		arm_dash.GoToTarget(place_pose);
		while(1){
			ROS_INFO_STREAM("Send arm2 to drop part ");
			arm_dash.GoToTarget(place_pose);
			ros::spinOnce();
			if (abs(arm_dash.GetEEPose().position.x-0.31292)<0.05){
				ROS_INFO_STREAM("State achieved");
				break;
			}
		}
		if(arm_dash.GetGripperStatus()==false){
			ROS_INFO_STREAM("Part droppped somewhere go again");
			return -1;
		}

		arm_dash.GripperToggle(false);
		
		while(1){
			ROS_INFO_STREAM("Send arm2 home");
			arm_dash.SendRobotHome();
			ros::spinOnce();
			if (arm_dash.GetJointStates()[0]-home_joint_pose_[0]<0.05)
				break;
		}
		//home_joint_pose_[1]=3.1+1.5;
		if(arm_id==2){ 
	    	home_joint_pose_[1]=3.1+1.55;
	    	
		}
		if(arm_id==1){ 
	    	home_joint_pose_[1]=1.55;
	    	
		}
		while(1){
			ROS_INFO_STREAM("Send arm1 home twisted");
			arm.SendRobotJointPosition(home_joint_pose_);
			ros::spinOnce();
			if (abs(arm.GetJointStates()[1]-(home_joint_pose_[1]))<0.1)
				break;
		}
		home_joint_pose_[1]=3.1;


		place_pose.position.z-=0.1;
		place_pose.position.z+=0.02;
		auto up_pose=place_pose;
		up_pose.position.z+=0.2;
		
		while(1){//Pick up part by arm1
			arm.GripperToggle(true);
			arm.GoToTarget({up_pose,place_pose});
			if (arm.GetGripperStatus()==true){
				ROS_INFO_STREAM("Product attached");
				break;
			}
			place_pose.position.z-=0.005;
		}
		//Product picked up by arm1
		


		while(1){//Send arm1 home
			ROS_INFO_STREAM("Send arm1 home ");
			arm.SendRobotHome();
			ros::spinOnce();
			if (arm.GetJointStates()[0]-home_joint_pose_[0]<0.05)
				break;
		}
		return 0;
}

bool AriacOrderManager::PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id) {
    //agv_id=2;
    while(1){
	int arm_id;
	std::string arm1,arm_dash1;
	
	if (agv_id==1){
		arm1={"arm1"};
		arm_dash1={"arm2"};
		arm_id=2;

	}

	if (agv_id==2){
		arm1={"arm2"};
		arm_dash1={"arm1"};
		arm_id=2;
		arm1_.SendRobotJointPosition(end_position_);
	}


	RobotController arm(arm1);
	RobotController arm_dash(arm_dash1);
    std::string product_type = product_type_pose.first;
    ROS_WARN_STREAM("Product type >>>> " << product_type);
    std::string product_frame = this->GetProductFrame(product_type);
    ROS_WARN_STREAM("Product frame >>>> " << product_frame);
    auto part_pose = camera_.GetPartPose("/world",product_frame);

 //    if (agv_id==1){
	// 	while(1){
	// 		arm_dash.SendRobotJointPosition(end_position_);
	// 		ros::spinOnce();
	// 		if(abs(arm_dash.GetJointStates()[0]-end_position_[0])<0.1){
	// 			ROS_INFO_STREAM("Send arm_dash to endjoint");
	// 			break;
	// 		}
	// 	}
 //    }

	// if (agv_id==2){
	// 	while(1){
	// 		arm_dash.SendRobotJointPosition(end_position_);
	// 		ros::spinOnce();
	// 		if(abs(arm_dash.GetJointStates()[0]-end_position_[0])<0.1){
	// 			ROS_INFO_STREAM("Send arm_dash to endjoint");
	// 			break;
	// 		}
	// 	}
 //    }


    if(product_type == "pulley_part")
        part_pose.position.z += 0.08;
    //--task the robot to pick up this part
    int repick_flag=0;
    
    
    if (agv_id==1   &&    part_pose.position.y<-1.7){  //arm1 cannot reach this position
    	repick_flag=1;
    	
    	int result=GetFromArm2(product_type,part_pose,product_frame,1);
    	if (result==-1)
    		continue;


    }
    else if(agv_id==2   &&    part_pose.position.y>1.65){
    	repick_flag=1;
    	
    	int result=GetFromArm2(product_type,part_pose,product_frame,2);
    	if (result==-1)
    		continue;

    }
    
    
    else{//Sadha Pick up
    	
	    ROS_INFO_STREAM("Sadha Pick up");
	    int n=0;
	    bool failed_pick = arm.PickPart(part_pose);
	    ROS_WARN_STREAM("Picking up state " << failed_pick);
	    ros::Duration(0.5).sleep();

	    while(!failed_pick){
	        auto part_pose = camera_.GetPartPose("/world",product_frame);
	        failed_pick = arm.PickPart(part_pose);
	        n=n+1;
	        ROS_INFO_STREAM("Attempt no to pick up part"  <<n);
	    }
	    ROS_INFO_STREAM("Successfully picked up");
	}
    

    
    if (product_type=="pulley_part"){
                      
    	//do something for second arm
		pulleyFlipper();

	}	



    //--get the pose of the object in the tray from the order
    geometry_msgs::Pose drop_pose = product_type_pose.second;

    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;

    


    if(agv_id==1){
        StampedPose_in.header.frame_id = "/kit_tray_1";
        StampedPose_in.pose = drop_pose;
        ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        StampedPose_out.pose.position.y -= 0.2;
        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");

    }
    else{
        StampedPose_in.header.frame_id = "/kit_tray_2";
        StampedPose_in.pose = drop_pose;
        //ROS_INFO_STREAM("StampedPose_in " << StampedPose_in.pose.position.x);
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        StampedPose_out.pose.position.y += 0.2;
        StampedPose_out.pose.position.y-=0.06;

        //ROS_INFO_STREAM("StampedPose_out " << StampedPose_out.pose.position.x);
    }

    ros::spinOnce();
    
    if (arm.GetGripperStatus()==false){
    	ROS_WARN("Part has been dropped somewhere");
    	
    	arm.SendRobotHome();
    	continue;
    }


    ros::spinOnce();
	if (repick_flag==1    &&   agv_id==1){             //Because while repickig its perfectly picked up 
		StampedPose_out.pose.position.y+=0.06;
	}
	if (repick_flag==1    &&   agv_id==2){             //Because while repickig its perfectly picked up 
		StampedPose_out.pose.position.y-=0.06;
	}
	//end_position_[1]=1.55;
	
	while(1){         
		arm.SendRobotJointPosition(end_position_);
		ROS_INFO_STREAM("Diff is "<<arm.GetJointStates()[0]-end_position_[0]);
		ros::spinOnce();
		if (arm.GetJointStates()[0]-end_position_[0]<0.1 )
			break;
	}
	//end_position_[1]=3.1;
	ros::Duration(0.5).sleep();
    auto result = arm.DropPart(StampedPose_out.pose);

    bool result1;
    if (result==-1){
      ROS_INFO_STREAM("Retrying due to defected part or dropped part");
      // std::pair<std::string,geometry_msgs::Pose> retry_pair;
      // retry_pair.first=product_type;
      // retry_pair.second=drop_pose;
      // //Make it go to home first
      arm.SendRobotHome();
      ros::Duration(0.5).sleep();
      arm.GripperToggle(false);   //if the part is defected it will drop it
      continue;
      //result1=PickAndPlace(retry_pair, int(1)); }
    }


    if (result==0)
      result1=false;
    else
      result1=true;
    return result1;
	
	


	
}
}


void printVector(std::vector<auto> vect)
{for (int i=0;i<vect.size();i++)
	{ROS_INFO_STREAM(vect[i]);}
	}




void AriacOrderManager::PickFromConveyer(std::string product_type)
{  camera_4_product_=product_type;   //Let camera detect product_type parts
   


   ROS_INFO_STREAM("Picking up from conveyer");
   
   //std::vector<double> conveyer_joint_pose_ = {-1.1800,0.0,-1.1, 1.9, 3.9, 4.7, 0};
   arm1_.SendRobotConveyer();
 
   auto temp_pose=camera4_op_pose_;
   temp_pose.position.z=temp_pose.position.z-0.1;

   geometry_msgs::Pose temporaryPose;
   temporaryPose.position.x = 1.193;
   temporaryPose.position.y = 0.407;
   temporaryPose.position.z = 1.175+0.1 ;
   
   auto temp_pose_1 = temporaryPose;
   temp_pose_1.position.z -= (0.120);
   if (product_type=="pulley_part")
    {
     temp_pose_1.position.z += (0.120+0.08-0.04-0.1); 
    }                                       //Take robot to conveyer position
   arm1_.GoToTarget({ temporaryPose,temp_pose_1,});       
   int n=0;
   
   while(1){
   
    init_=1;
    ros::spinOnce();
    if (arm1_.GetGripperStatus()==true){
            ROS_INFO_STREAM("Product is attached");
            break;}
    //auto time1=ros::Time::now();
    auto up_pose=camera4_op_pose_;
    up_pose.position.z=up_pose.position.z+0.15;
    up_pose.position.y=up_pose.position.y+0.10;
    if (flag_==1){
	    ROS_INFO_STREAM("Moving down...");
	    camera4_op_pose_.position.y=camera4_op_pose_.position.y-0.25+0.008;
	    arm1_.GripperToggle(true);
	    camera4_op_pose_.position.z=camera4_op_pose_.position.z+0.02;
	    if (product_type=="pulley_part"){
    		if ( camera4_op_pose_.position.y<0.425){
    			ROS_INFO_STREAM("1st if");
    			camera4_op_pose_.position.z=camera4_op_pose_.position.z+0.08-0.0110;
    			camera4_op_pose_.position.y=camera4_op_pose_.position.y+0.00005-0.03;    			
    		}
	    	if ( camera4_op_pose_.position.y>0.425){
	    		ROS_INFO_STREAM("2nd if");
    			camera4_op_pose_.position.z=camera4_op_pose_.position.z+0.08-0.0140;
    			camera4_op_pose_.position.y=camera4_op_pose_.position.y+0.00005-0.027;    			
    		}


	    	}
	    ROS_INFO_STREAM("Moving to grip  "<<product_type);
	    arm1_.GoToTarget(camera4_op_pose_);
	    ros::Duration(0.3).sleep();
	    ros::spinOnce();
        if (arm1_.GetGripperStatus()==true){
            ROS_INFO_STREAM("Product is attached");
            break;}
        else{
        	ros::spinOnce();
        	arm1_.GoToTarget(temp_pose_1);}
      }
    
    
    //ros::Duration(0.25).sleep();
	n++;
	}
   
init_=0;
//arm1_.SendRobotConveyer();
arm1_.GoToTarget(temporaryPose);
ROS_INFO_STREAM("Going up");

if (product_type=="pulley_part"){//waste
	
	ros::Duration(1).sleep();

}


	
arm1_.SendRobotHome();



if (product_type=="pulley_part"){
                      

		pulleyFlipper();

}

ros::spinOnce();
if (arm1_.GetGripperStatus()==false){
	PickFromConveyer(product_type);
}




}


//will send to bin5
//flip and pick up 
//ends with arm near bin5
void AriacOrderManager::pulleyFlipper(){
		init_cam1_=1;
		while(1){  //To ensure flipping position
    		arm1_.SendRobotFlipping();
    		ros::spinOnce();
    		if (abs(joint_states_[0]-0.507)<0.08)
    			break;
		}
		ros::spinOnce();
		if (arm1_.GetGripperStatus()==false    &&   pulley_on_bin_==false){
			ROS_INFO_STREAM("Not attached to gripper foing back");
			return;
		}
		geometry_msgs::Pose flipping_pose1, flipping_pose2;
    	flipping_pose1.position.x=-0.2726;
    	flipping_pose1.position.y=1.1701;
    	flipping_pose1.position.z=0.7883+0.08+0.22-0.20;
    	flipping_pose2=flipping_pose1;
    	flipping_pose2.position.z=flipping_pose2.position.z+0.1;

    	arm1_.GoToTarget({flipping_pose2,flipping_pose1});   //To drop pulley in center of bin

    	ros::spinOnce();
    	if (arm1_.GetGripperStatus()==false  &&   pulley_on_bin_==false    )         //pulley fell down while flipping
    		{ROS_INFO_STREAM("Not attached to gripper foing back");
    			return;}

    	arm1_.GripperToggle(false);
    	ros::Duration(0.5).sleep();
    	ros::spinOnce();
    	
    	while(1){  //To ensure pulley sat down properly at 90deg
	    	arm1_.GripperToggle(true);
	    	ros::spinOnce();

	    	flipping_pulley_pose_.position.z=flipping_pulley_pose_.position.z+0.04;
	    	arm1_.GoToTarget({flipping_pose1,flipping_pulley_pose_});
	    	ros::Duration(0.2).sleep();
	    	arm1_.GoToTarget(flipping_pose1);
	    	ros::spinOnce();
	    	
	    	int n1=0;
	    	while(arm1_.GetGripperStatus()==false){
	    		ROS_INFO_STREAM("Going down to pickup pulley again");
	    		flipping_pulley_pose_.position.z=flipping_pulley_pose_.position.z+0.055;  //pulley thickmess
	    		if (n1>3)
	    			n1=0;
	    		for(int i=n1;i>0;i--)
	    			flipping_pulley_pose_.position.z=flipping_pulley_pose_.position.z-0.01;   //decrementing with each iteration

	    		auto flipping_pulley_pose_up=flipping_pulley_pose_;
	    		flipping_pulley_pose_up.position.z+=0.2;
	    		arm1_.GoToTarget(flipping_pulley_pose_up);
	    		arm1_.GoToTarget(flipping_pulley_pose_);
				ros::Duration(0.3).sleep();
	    		ros::spinOnce();
	    		arm1_.GoToTarget(flipping_pose1);
	    		
	    		

	    		n1++;}

	    	
	    	//ros::Duration(1).sleep();
	   		ros::spinOnce();
	   		//printVector(joint_states_);
	   		joint_states_[4]=joint_states_[4]-1.3;     //turning the wrist
	   		auto temp1=joint_states_;
	   		auto temp=joint_states_[4];
	   		ros::spinOnce();
	   		while(1){         //so that we know the wrist has turned
	   			arm1_.SendRobotJointPosition(temp1);ros::spinOnce();
	   			ROS_INFO_STREAM(joint_states_[4]-temp);
	   			if (joint_states_[4]-temp<0.1)
	   				break;
	   		}



	   		robot_tf_listener_.waitForTransform("arm1_linear_arm_actuator", "arm1_ee_link",
	                                            ros::Time(0), ros::Duration(10));
	    	robot_tf_listener_.lookupTransform("/arm1_linear_arm_actuator", "/arm1_ee_link",
	                                           ros::Time(0), robot_tf_transform_);


	    	fixed_orientation_.x = robot_tf_transform_.getRotation().x();
	    	fixed_orientation_.y = robot_tf_transform_.getRotation().y();
	    	fixed_orientation_.z = robot_tf_transform_.getRotation().z();
	    	fixed_orientation_.w = robot_tf_transform_.getRotation().w();


	    	ROS_INFO_STREAM("90 deg fixed_orientation_ is "<<fixed_orientation_);
	    	ros::Duration(0.15).sleep();
	    	//ROS_INFO_STREAM("Sleeping");
	    	
	   		auto flipping_pose1_with_orientation=flipping_pose1;
	   		flipping_pose1_with_orientation.position.z+=0.04;
	   		//flipping_pose1_with_orientation.position.x+=0.2;
	   		flipping_pose1_with_orientation.orientation=fixed_orientation_;
	   		ROS_INFO_STREAM("Taking ee to place pulley");
	    	arm1_.GoToTarget_with_orientation(flipping_pose1_with_orientation);
	    	

	    	arm1_.GripperToggle(false);
	    	ros::Duration(0.5).sleep();
	    	ros::spinOnce();
	    	ROS_INFO_STREAM("Pulley z is at" <<flipping_pulley_pose_.position.z);
	    	if (flipping_pulley_pose_.position.z<0.81){      //means pulley has fallen down and pick up again
	    		continue;
				ROS_INFO_STREAM("Pulley is not upright try again");
				}

	    	break;
    	}

    	auto flipping_pose_dash=flipping_pose1;
    	flipping_pose_dash.position.x=flipping_pose_dash.position.x+0.08;
    	flipping_pose_dash.position.y=flipping_pose_dash.position.y+0.12;
    	flipping_pose_dash.position.z+=0.04;
    	flipping_pose_dash.orientation=fixed_orientation_;
    	auto flipping_pose1_with_orientation=flipping_pose1;
    	flipping_pose1_with_orientation.orientation=fixed_orientation_;
    	arm1_.GoToTarget_with_orientation({flipping_pose_dash});                                     //taking back and macking straight
    	ros::spinOnce();
    	//joint_states_[4]=joint_states_[4]+1.6;   //making wrsit straight again
    	//arm1_.SendRobotJointPosition(joint_states_);
    	

    	flipping_pose1=flipping_pose_dash;
    	//flipping_pose1.position.x=flipping_pose1.position.x-0.25;
    	flipping_pose1.position.z=flipping_pose1.position.z+0.28;
    	flipping_pose1.position.y=flipping_pose1.position.y-0.25;
    	ROS_INFO_STREAM("Moving up");
    	arm1_.GoToTarget(flipping_pose1);                 //moving up
    	//ros::Duration(1).sleep();


    	auto flipping_pose3=flipping_pose1;
    	flipping_pose3.position.x=-0.45;
    	ROS_INFO_STREAM("moving ee ahead in air");
    	arm1_.GoToTarget({flipping_pose1,flipping_pose3});  //moving ee ahead in the air
    	ros::Duration(0.2).sleep();
    	//ros::Duration(1).sleep();

    	auto flipping_pose4=flipping_pose3;
    	ROS_INFO_STREAM("moving ee towards pulley");
    	ros::spinOnce();
    	//flipping_pose4.position.z=flipping_pose4.position.z-0.20;
    	flipping_pose4.position.y=flipping_pulley_pose_.position.y;
    	arm1_.GoToTarget({flipping_pose3,flipping_pose4});     //moving down ahead
    	ros::Duration(0.2).sleep();
    	//ros::Duration(1).sleep();

    	
    	ROS_INFO_STREAM("moving ee down");
    	flipping_pose4.position.z=flipping_pose4.position.z-0.28;
    	ros::spinOnce();
    	flipping_pose4.position.y=flipping_pulley_pose_.position.y;
    	arm1_.GoToTarget(flipping_pose4);
    	//ros::Duration(1).sleep();

    	ros::spinOnce();
    	flipping_pose4.position.y=flipping_pulley_pose_.position.y;
    	ROS_INFO_STREAM("moving ee behind to drop pulley");
    	auto flipping_pose5=flipping_pose4;
    	flipping_pose5.position.x=-0.25;    	
    	arm1_.GoToTarget({flipping_pose4,flipping_pose5});   //moving behind to drop the pulley
    	ros::Duration(0.2).sleep();
    	//ros::Duration(2).sleep();
    	auto flipping_pose6=flipping_pose1;
    	flipping_pose6.position.z-=0.35;
    	ros::spinOnce();
    	int n1=0;
    	arm1_.GripperToggle(true);
    	while(arm1_.GetGripperStatus()==false ){//pick up routine
    		ROS_INFO_STREAM("Going down to pickup pulley again");
    		flipping_pulley_pose_.position.z=flipping_pulley_pose_.position.z+0.055-0.035;  //pulley thickmess
    		//if (n1>3)
    		//	n1=0;
    		if (n1>3)
    			n1=0;
    		for(int i=n1;i>0;i--)
    			flipping_pulley_pose_.position.z=flipping_pulley_pose_.position.z-0.01;   //decrementing with each iteration

    		auto flipping_pulley_pose_up=flipping_pulley_pose_;
    		flipping_pulley_pose_up.position.z+=0.2;
    		arm1_.GoToTarget(flipping_pulley_pose_up);
    		arm1_.GoToTarget(flipping_pulley_pose_);
			ros::Duration(0.3).sleep();
    		if (pulley_on_bin_==false)            //if pulley falls down
    			{ROS_INFO_STREAM("Pick up was attempted but pulley not on bin hence returning");
    				return;}

    		ros::spinOnce();
    		arm1_.GoToTarget(flipping_pose6);
    		n1++;
    		}//pick up routine
		

		ros::spinOnce();
		init_cam1_=0;
    }


void AriacOrderManager::CompletethisOrder(osrf_gear::Order order){
	ROS_INFO_STREAM("Complete this order");
	arm1_.SendRobotHome();
	arm2_.SendRobotHome();
	auto shipments = order.shipments;
	for (const auto &shipment: shipments){
	 auto shipment_type = shipment.shipment_type;
	  for (const auto &product: shipment.products){
	    ROS_INFO_STREAM("Product to place is  "    <<product.type);
	    //arm2_.SendRobotJointPosition(end_position_);
	    std::pair<std::string,geometry_msgs::Pose> product_type_pose;
	    product_type_pose.first=product.type;
	    product_type_pose.second=product.pose;
	    if (product_frame_list_.find(product.type)!=product_frame_list_.end()){  //Found in list
	      if (shipment.agv_id=="any"   ||shipment.agv_id=="agv1"){
	      PickAndPlace(product_type_pose, 1);
	  	}
	  	else{
  		  PickAndPlace(product_type_pose,2);
	  	}
	  	}
	    else{//conveyer part
	      ROS_INFO_STREAM("Please pick up from conveyer"    <<product.type);
	      PickFromConveyer(product_type_pose.first);
	      bool result,result1;
	      geometry_msgs::Pose drop_pose = product_type_pose.second;

	      geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;

	      
	    if(shipment.agv_id=="any"     ||    shipment.agv_id=="agv1"){
	        StampedPose_in.header.frame_id = "/kit_tray_1";
	        StampedPose_in.pose = drop_pose;
	        ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
	        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
	        StampedPose_out.pose.position.z += 0.1;
	        StampedPose_out.pose.position.y -= 0.2;
	        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");

	    }
	    else{
	        StampedPose_in.header.frame_id = "/kit_tray_2";
	        StampedPose_in.pose = drop_pose;
	        //ROS_INFO_STREAM("StampedPose_in " << StampedPose_in.pose.position.x);
	        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
	        StampedPose_out.pose.position.z += 0.1;
	        StampedPose_out.pose.position.y += 0.2;
	        //ROS_INFO_STREAM("StampedPose_out " << StampedPose_out.pose.position.x);
	    }
	      

	      ros::Duration(0.5).sleep();
	      while(1) {//Conveyer part

	          ros::spinOnce();
	          if (arm1_.GetGripperStatus()==false){
	          	//arm1_.SendRobotConveyer();
	          	PickFromConveyer(product_type_pose.first);
	          	continue;
	          	}

	          result = arm1_.DropPart(StampedPose_out.pose);
	          ROS_INFO_STREAM("result from DropPart: "<< result);

		    // bool result1;
		    if (result==-1){
		      ROS_INFO_STREAM("Retrying due to defected part");
		      arm1_.SendRobotConveyer();
		      arm1_.GripperToggle(false);
		      continue;
		      }
		    if (result==0) {
		  	  ROS_INFO_STREAM("Breaking out......");
		  	  break;
		  		}

			}
		}//conveyer part
	if (order_update_flag_==1){
		ROS_WARN("Order being updated.....");
		return;
		}
	}//product
  }//shipment
}//func

void AriacOrderManager::CompleteOrderUpdate(osrf_gear::Order order){
	ROS_WARN("Order being Updated");
	//look at components on tray
		//take diff of order components and on tray components
		//make vector of components on tray that are not needed
		//make vector of components on tray that need repositioning
		//make vector of components on tray that are new
	//remove waste components
	//reposition elements present
	//pick up and place new elements needed
	int agv_id=0;
	auto agv_id_string=order.shipments[0].agv_id;
	
	std::vector <std::pair<std::string,geometry_msgs::Pose> > cam5_list_right;
	if (agv_id_string=="any"    ||    agv_id_string=="agv1"){
		init_cam5_=1;
		ros::Duration(0.5).sleep();
		ros::spinOnce();     //to update part list of kit_tray i.e. cam5_list_	
		init_cam5_=0;
		cam5_list_right=cam5_list_;
		agv_id=1;
	}

	else{
		init_cam5_dash_=1;
		ros::Duration(0.5).sleep();
		ros::spinOnce();     //to update part list of kit_tray i.e. cam5_list_	
		init_cam5_dash_=0;
		cam5_list_right=cam5_list_dash_;
		agv_id=2;
	}

	//agv_id=1;
	std::vector <std::pair<std::string,geometry_msgs::Pose> > order_list;
	auto shipments = order.shipments;
	for (const auto &shipment: shipments){
	 auto shipment_type = shipment.shipment_type;
	  for (const auto &product: shipment.products){
	  	std::pair<std::string,geometry_msgs::Pose> temp;
	  	temp.first=product.type;
	  	temp.second=product.pose;
	  	order_list.push_back(temp);
	  }
	std::vector <std::pair<std::string,std::vector<geometry_msgs::Pose> > > repos_list;
	std::vector <std::pair<std::string,geometry_msgs::Pose> > order_repos_list(order_list);
	std::vector <std::pair<std::string,geometry_msgs::Pose> > cam5_repos_list(cam5_list_right);
	std::vector <std::pair<std::string,geometry_msgs::Pose> > diff_list;
	std::vector <std::pair<std::string,geometry_msgs::Pose> > get_list;
	ROS_INFO_STREAM("Camera 5 can see these many components  "<<cam5_list_right.size());
	for(int i=0;i<cam5_list_right.size();i++)
		{ROS_INFO_STREAM("  " <<cam5_list_right[i].first);}

	for(int i=0;i<order_repos_list.size();i++){
		for (int j=0;j<cam5_repos_list.size();j++){
			if (cam5_repos_list[j].first==order_repos_list[i].first){
				std::vector<geometry_msgs::Pose> poses;
				poses.push_back(cam5_repos_list[j].second);
				poses.push_back(order_repos_list[i].second);
				std::pair<std::string,std::vector<geometry_msgs::Pose>> temp;
				temp.first=cam5_repos_list[j].first;
				temp.second=poses;
				repos_list.push_back(temp);
				cam5_repos_list.erase(cam5_repos_list.begin()+j);
				order_repos_list.erase(order_repos_list.begin()+i);
				i--;
				continue;
			}
		}
	}

	diff_list=cam5_repos_list;
	get_list=order_repos_list;
	
	for(int i=0;i<diff_list.size();i++){
		ROS_INFO_STREAM("Throwing    "<<diff_list[i].first);
		ThrowComponents(diff_list[i],agv_id);
	}

	for(int j=0;j<repos_list.size();j++){
		ROS_INFO_STREAM("Reposition    "<<repos_list[j].first);
		RepositionComponents(repos_list[j],agv_id);
	}

	for(int k=0;k<get_list.size();k++){
		if (product_frame_list_.find(get_list[k].first)!=product_frame_list_.end())  //Found in list
	      {ROS_INFO_STREAM("Get    "<<get_list[k].first);
	      PickAndPlace(get_list[k], agv_id);     //add conveyer type code
	  	}
	}
}

}













void AriacOrderManager::ThrowComponents(std::pair<std::string,geometry_msgs::Pose>diff_list,int agv_id){
	

	int arm_id;
	std::string arm1,arm_dash1;
	
	if (agv_id==1){
		arm1={"arm1"};
		arm_dash1={"arm2"};
		arm_id=2;

	}

	if (agv_id==2){
		arm1={"arm2"};
		arm_dash1={"arm1"};
		arm_id=2;
	}

	RobotController arm(arm1);
	RobotController arm_dash(arm_dash1);
	geometry_msgs::Pose part_pose;
	
	ros::spinOnce();
	ROS_INFO_STREAM("ThrowComponents");
	ros::spinOnce();
	while(1){         //so that we know the arm is at agv1
		arm.SendRobotJointPosition(end_position_);
		ros::spinOnce();
		ROS_INFO_STREAM(arm.GetJointStates()[0]-end_position_[0]);
		if (arm.GetJointStates()[0]-end_position_[0]<0.1)
			break;
	}
	geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;     
	if (agv_id==1){
	part_pose=diff_list.second;
	     
    StampedPose_in.header.frame_id = "/logical_camera_8_frame";
    StampedPose_in.pose = part_pose;
    part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
    part_pose=StampedPose_out.pose;
	}

	if (agv_id==2){
	part_pose=diff_list.second;
	//geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
    StampedPose_in.header.frame_id = "/logical_camera_9_frame";
    StampedPose_in.pose = part_pose;
    part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
    part_pose=StampedPose_out.pose;
	}
	

	if (diff_list.first=="pulley_part")
		part_pose.position.z+=0.08;
	

	part_pose.position.z+=0.02;
	auto up_pose=part_pose;
	up_pose.position.z+=0.2;
	int n=0;
	arm.GripperToggle(true);
	ROS_INFO_STREAM("GOing to pick   "<<diff_list.first);
	

	while(1){
		if (agv_id==1)
		{init_cam5_2_=1;
		reposition_part_type_=diff_list.first;
		reposition_part_pose_=part_pose;
		ros::Duration(0.25).sleep();
		ros::spinOnce();
		reposition_part_pose_back_.position.z+=0.02;
		}

		if (agv_id==2)
		{init_cam5_2_dash_=1;
		reposition_part_type_dash_=diff_list.first;
		reposition_part_pose_dash_=part_pose;
		ros::Duration(0.25).sleep();
		ros::spinOnce();
		reposition_part_pose_back_dash_.position.z+=0.02;
	}
		
		if(agv_id==1){
		for (int i=0;i<n;i++)
			reposition_part_pose_back_.position.z-=0.005;}


		if(agv_id==2){
		for (int i=0;i<n;i++)
			reposition_part_pose_back_dash_.position.z-=0.005;}



		if(agv_id==1){
		if (repose_flag_==1){
			arm.GoToTarget({up_pose,reposition_part_pose_back_});
			part_pose=reposition_part_pose_back_;
		}
		else{
			ROS_INFO_STREAM("Reposition flag unsuccessfull");
			arm.GoToTarget({up_pose,part_pose});
			part_pose.position.z-=0.005;

		}
		}


		if(agv_id==2){
		if (repose_flag_dash_==1){
			arm.GoToTarget({up_pose,reposition_part_pose_back_dash_});
			part_pose=reposition_part_pose_back_dash_;
		}
		else{
			ROS_INFO_STREAM("Reposition flag unsuccessfull");
			arm.GoToTarget({up_pose,part_pose});
			part_pose.position.z-=0.005;

		}
		}



		ros::Duration(0.1).sleep();
		arm.GoToTarget(up_pose);
		//other_part_pose.position.z-=0.005;
		
		if (arm.GetGripperStatus()==true){
			ROS_INFO_STREAM("Product attached");
			break;
		}

		n++;
	}
	init_cam5_2_=0;
	init_cam5_2_dash_=0;

	ros::spinOnce();
	while(1){         //so that we know the arm is at agv1 throwing pos
		arm.SendRobotJointPosition(end_position_);
		ros::spinOnce();
		ROS_INFO_STREAM(joint_states_[0]-2.2);
		if (arm.GetJointStates()[0]-2.2<0.1)
			break;
	}
	arm.GripperToggle(false);
	ROS_INFO_STREAM("Part dropped");

}

float AriacOrderManager::euler_distance(geometry_msgs::Pose p1,geometry_msgs::Pose p2){
	float x1=p1.position.x;
	float y1=p1.position.y;
	float z1=p1.position.z;
	float x2=p2.position.x;
	float y2=p2.position.y;
	float z2=p2.position.z;

	float dist=((x2-x1)*(x2-x1))+((y2-y1)*(y2-y1));
	return dist;



}




std::pair<std::string,geometry_msgs::Pose> AriacOrderManager:: PartThere(geometry_msgs::Pose drop_pose,int agv_id){
	
	if (agv_id==1){
	init_cam5_=1;
	ros::Duration(0.5).sleep();
	ros::spinOnce();
	init_cam5_=0;
	//cam5_list_ updated
	std::pair<std::string,geometry_msgs::Pose> temp;
	temp.first="None";
	for(int i=0;i<cam5_list_.size();i++){
		//ROS_INFO_STREAM("Checking for  "<<cam5_list_[i].first);
		auto part_pose=cam5_list_[i].second;
		geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
	    StampedPose_in.header.frame_id = "/logical_camera_8_frame";
	    StampedPose_in.pose = part_pose;
	    part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
	    part_pose=StampedPose_out.pose;  //in world frame now

	    if( euler_distance(part_pose,drop_pose)<0.005){
	    	ROS_INFO_STREAM("Checking for  "<<cam5_list_[i].first);
	    	ROS_INFO_STREAM("There is this part where we have to keep part");
	    	temp.first=cam5_list_[i].first;
	    	temp.second=part_pose;
	    	return temp;
	    }

	}
return temp;}

	else{
		init_cam5_dash_=1;
		ros::Duration(0.5).sleep();
		ros::spinOnce();
		init_cam5_dash_=0;
		//cam5_list_ updated
		std::pair<std::string,geometry_msgs::Pose> temp;
		temp.first="None";
		for(int i=0;i<cam5_list_dash_.size();i++){
			//ROS_INFO_STREAM("Checking for  "<<cam5_list_[i].first);
			auto part_pose=cam5_list_dash_[i].second;
			geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
		    StampedPose_in.header.frame_id = "/logical_camera_9_frame";
		    StampedPose_in.pose = part_pose;
		    part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
		    part_pose=StampedPose_out.pose;  //in world frame now

		    if( euler_distance(part_pose,drop_pose)<0.005){
		    	ROS_INFO_STREAM("Checking for  "<<cam5_list_dash_[i].first);
		    	ROS_INFO_STREAM("There is this part where we have to keep part");
		    	temp.first=cam5_list_dash_[i].first;
		    	temp.second=part_pose;
		    	return temp;
		    	}

			}
		return temp;
	}

	}


void AriacOrderManager::RepositionComponents(std::pair<std::string,std::vector<geometry_msgs::Pose> >diff_list,int agv_id){
	ROS_INFO_STREAM("Reposition Components");
	
	int arm_id;
	std::string arm1,arm_dash1;
	
	if (agv_id==1){
		arm1={"arm1"};
		arm_dash1={"arm2"};
		arm_id=2;

	}

	if (agv_id==2){
		arm1={"arm2"};
		arm_dash1={"arm1"};
		arm_id=2;
	}

	RobotController arm(arm1);
	RobotController arm_dash(arm_dash1);
	
	ros::spinOnce();
	
	if (agv_id==1)
		end_position_[1]=1.55;
	else
		end_position_[1]=3.1+1.5;
	
	while(1){         //so that we know the arm is at agv1
		arm.SendRobotJointPosition(end_position_);
		ros::spinOnce();
		//ROS_INFO_STREAM(joint_states_[0]-2.2);
		if (arm.GetJointStates()[0]-2.2<0.1)
			break;
	}
	end_position_[1]=3.1;
	

	//Transform Poses

	auto part_pose=diff_list.second.at(0);
	

	geometry_msgs::PoseStamped StampedPose_in,StampedPose_out; 
	if (agv_id==1){
		         
	    StampedPose_in.header.frame_id = "/logical_camera_8_frame";
	    StampedPose_in.pose = part_pose;
	    part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
	    part_pose=StampedPose_out.pose;
	}
    //ROS_INFO_STREAM(part_pose); 
	if (agv_id==2){
		//geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;          
	    StampedPose_in.header.frame_id = "/logical_camera_9_frame";
	    StampedPose_in.pose = part_pose;
	    part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
	    part_pose=StampedPose_out.pose;
	}


    ros::spinOnce();
	auto drop_pose=diff_list.second.at(1);
    //int agv_id=1;           //change this to have as input
	if(agv_id==1){
        StampedPose_in.header.frame_id = "/kit_tray_1";
        StampedPose_in.pose = drop_pose;
        //ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        StampedPose_out.pose.position.y -= 0.2;
        StampedPose_out.pose.position.y+=0.06;
        //ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");

    }
    else{
        StampedPose_in.header.frame_id = "/kit_tray_2";
        StampedPose_in.pose = drop_pose;
        //ROS_INFO_STREAM("StampedPose_in " << StampedPose_in.pose.position.x);
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        StampedPose_out.pose.position.y += 0.2;
        StampedPose_out.pose.position.y-=0.06;
        //ROS_INFO_STREAM("StampedPose_out " << StampedPose_out.pose.position.x);
    }
	
	

	//to check wheter part already there
	ROS_INFO_STREAM("Distance between 2 parts is   "<<euler_distance(part_pose,StampedPose_out.pose));
    if (euler_distance(part_pose,StampedPose_out.pose)<0.07){
    	ROS_INFO_STREAM("Reposition not required as part already at right position");
    	ros::Duration(1.0).sleep();
    	return;
    }

    

    for(int i=0;i<MultipleRepositionVector_.size();i++){
    	if (diff_list.first==MultipleRepositionVector_[i].first    &&    euler_distance(MultipleRepositionVector_[i].second.at(0), part_pose)<0.005){
    		part_pose=MultipleRepositionVector_[i].second.at(1);
    	}
    }
    geometry_msgs::Pose up_pose;
    //There is some part there
    auto other_part_type_pose=PartThere(StampedPose_out.pose,agv_id);
    auto other_part_pose=other_part_type_pose.second;
    if(other_part_type_pose.first!="None"    &&   other_part_type_pose.first!=diff_list.first   ){
    	//there is a part there or there is not a part with same type
    	ROS_INFO_STREAM("Multiple repositioning Activated....");
    	std::vector<geometry_msgs::Pose> tempposes;
    	tempposes.push_back(other_part_pose);
    	tempposes.push_back(part_pose);
    	std::pair< std::string, std::vector<geometry_msgs::Pose > > temppair;
    	temppair.first=other_part_type_pose.first;
    	temppair.second=tempposes;
    	MultipleRepositionVector_.push_back(temppair);
    	ros::Duration(1.0).sleep();
    	geometry_msgs::Pose temp_pose;
    	temp_pose.position.x=0.488342;
    	if (agv_id==1)
    		temp_pose.position.y=3.353200;
    	else
    		temp_pose.position.y=-3.353200;
    	temp_pose.position.z=0.755067;
    	temp_pose.position.z+=0.1;
    	temp_pose.orientation=other_part_type_pose.second.orientation;
    	ROS_INFO_STREAM("Poses to check are    "<<StampedPose_out.pose<<other_part_type_pose.second);
    	up_pose=other_part_pose;

    	while(1){
    	auto other_part_pose=other_part_type_pose.second;  //this is the same as stamped_pose_out
    	//other_part_pose.position.z+=0.02;
		up_pose=other_part_pose;
		other_part_pose.position.z+=0.02;
		up_pose.position.z+=0.2;
		int n=0;
		arm.GripperToggle(true);
		ROS_INFO_STREAM("GOing to pick other part   "<<other_part_type_pose.first);

		while(1){
			
			if(agv_id==1){
			init_cam5_2_=1;
			reposition_part_type_=other_part_type_pose.first;
			reposition_part_pose_=other_part_pose;
			ros::Duration(0.25).sleep();
			ros::spinOnce();
			reposition_part_pose_back_.position.z+=0.02;
			}

			if(agv_id==2){
			init_cam5_2_dash_=1;
			reposition_part_type_dash_=other_part_type_pose.first;
			reposition_part_pose_dash_=other_part_pose;
			ros::Duration(0.25).sleep();
			ros::spinOnce();
			reposition_part_pose_back_dash_.position.z+=0.02;
			}



			
			if(agv_id==1){		
				for (int i=0;i<n;i++)
					reposition_part_pose_back_.position.z-=0.005;
				}

			if(agv_id==2){		
				for (int i=0;i<n;i++)
					reposition_part_pose_back_dash_.position.z-=0.005;
				}

			
			
			if(agv_id==1){
			if (repose_flag_==1){
				arm.GoToTarget({up_pose,reposition_part_pose_back_});
				other_part_pose=reposition_part_pose_back_;
			}
			else{
				ROS_INFO_STREAM("Reposition flag unsuccessfull");
				arm.GoToTarget({up_pose,other_part_pose});
				other_part_pose.position.z-=0.005;

			}
			}


			if(agv_id==2){
			if (repose_flag_dash_==1){
				arm.GoToTarget({up_pose,reposition_part_pose_back_dash_});
				other_part_pose=reposition_part_pose_back_dash_;
			}
			else{
				ROS_INFO_STREAM("Reposition flag unsuccessfull");
				arm.GoToTarget({up_pose,other_part_pose});
				other_part_pose.position.z-=0.005;

			}
			}

			ros::Duration(0.1).sleep();
			arm.GoToTarget(up_pose);
			//other_part_pose.position.z-=0.005;
			
			if (arm.GetGripperStatus()==true){
				ROS_INFO_STREAM("Product attached");
				break;
			}

			n++;
		}
		init_cam5_2_=0;
		init_cam5_2_dash_=0;
		//Other part picked up

		

		while(1){
		    arm.GoToTarget(temp_pose);
		    ros::spinOnce();
		    if (abs(arm.GetEEPose().position.x-temp_pose.position.x)<0.1     &&   abs(arm.GetEEPose().position.y-temp_pose.position.y)<0.1 ){
		    	ROS_INFO_STREAM("State achieved");
		    	break;
		    }
		}


		ros::Duration(0.5).sleep();
		ros::spinOnce();
		if(arm.GetGripperStatus()==false){
			ROS_INFO_STREAM("Part was dropped before keeping");
			continue;
		}
		ROS_INFO_STREAM("Other Part being dropped at temp position");
		arm.GripperToggle(false);
		//Other part droppped at temp pose


		


		//arm.GoToTarget(up_pose);
		//ros::Duration(0.25).sleep();
		ros::spinOnce();
		if(agv_id==1){
		end_position_[1]=1.55;
		while(1){         //so that we know the arm is at agv1
			arm.SendRobotJointPosition(end_position_);
			ros::spinOnce();
			//ROS_INFO_STREAM(joint_states_[0]-2.2);
			if (arm.GetJointStates()[0]-end_position_[0]<0.1)
				break;
		}
		end_position_[1]=3.1;
		}

		if(agv_id==2){
		end_position_[1]=3.1+1.55;
		while(1){         //so that we know the arm is at agv1
			arm.SendRobotJointPosition(end_position_);
			ros::spinOnce();
			//ROS_INFO_STREAM(joint_states_[0]-2.2);
			if (arm.GetJointStates()[0]-end_position_[0]<0.1)
				break;
		}
		end_position_[1]=3.1;
		}

		break;

	}

		
		while(1){
		if (diff_list.first=="pulley_part")
			part_pose.position.z+=0.08;
		
		part_pose.position.z+=0.02;
		up_pose=part_pose;
		up_pose.position.z+=0.2;
		int n=0;
		arm.GripperToggle(true);
		ROS_INFO_STREAM("GOing to pick   "<<diff_list.first);
		while(1){
			if(agv_id==1){
			init_cam5_2_=1;
			reposition_part_type_=diff_list.first;
			reposition_part_pose_=part_pose;
			ros::Duration(0.25).sleep();
			ros::spinOnce();
			reposition_part_pose_back_.position.z+=0.02;
			}

			if(agv_id==2){
			init_cam5_2_dash_=1;
			reposition_part_type_dash_=diff_list.first;
			reposition_part_pose_dash_=part_pose;
			ros::Duration(0.25).sleep();
			ros::spinOnce();
			reposition_part_pose_back_dash_.position.z+=0.02;
			}
			

			if(agv_id==1){
			for (int i=0;i<n;i++)
				reposition_part_pose_back_.position.z-=0.005;
			}


			if(agv_id==2){
			for (int i=0;i<n;i++)
				reposition_part_pose_back_dash_.position.z-=0.005;
			}

			
			if(agv_id==1){
			if (repose_flag_==1){
				arm.GoToTarget({up_pose,reposition_part_pose_back_});
				part_pose=reposition_part_pose_back_;
			}
			else{
				ROS_INFO_STREAM("Reposition flag unsuccessfull");
				arm.GoToTarget({up_pose,part_pose});
				part_pose.position.z-=0.005;

			}
			}

			if(agv_id==2){
			if (repose_flag_dash_==1){
				arm.GoToTarget({up_pose,reposition_part_pose_back_dash_});
				part_pose=reposition_part_pose_back_dash_;
			}
			else{
				ROS_INFO_STREAM("Reposition flag unsuccessfull");
				arm.GoToTarget({up_pose,part_pose});
				part_pose.position.z-=0.005;

			}			
			}




			
			ros::Duration(0.1).sleep();
			arm.GoToTarget(up_pose);
			
			if (arm.GetGripperStatus()==true){
				ROS_INFO_STREAM("Product attached");
				break;
			}

			n++;
		}
		init_cam5_2_=0;
		init_cam5_2_dash_=0;
		//part to be repositioned picked up
	    
	    
	    


		while(1){
		    arm.GoToTarget(StampedPose_out.pose);
		    ros::spinOnce();
		    if (abs(arm.GetEEPose().position.x-StampedPose_out.pose.position.x)<0.1   &&    abs(arm.GetEEPose().position.y-StampedPose_out.pose.position.y)<0.1 ){
		    	ROS_INFO_STREAM("State achieved");
		    	break;
		    }
		}



	    ros::Duration(1.0).sleep();
	    ROS_INFO_STREAM("Gone to keep part ");
	    ros::spinOnce();
	    if(arm.GetGripperStatus()==false){
			ROS_INFO_STREAM("Part was dropped before keeping");
			continue;
		}

	    // if (arm.GetGripperStatus()==false){
	    // 	ROS_INFO_STREAM("Part was dropped somewhere while reposition before dropping");
	    // 	std::pair<std::string,geometry_msgs::Pose> temp;
	    // 	temp.first=diff_list.first;
	    // 	temp.second=diff_list.second.at(1);//in kit tray frame
	    // 	PickAndPlace(temp,int(1));
	    // }
	    
	    arm.GripperToggle(false);
	    //Dropping part at specified posn
	    break;}
	    



	    while(1){
	    arm.GoToTarget(temp_pose);
	    temp_pose.position.z-=0.1;
	    temp_pose.position.z+=0.025;
	    //auto other_part_pose=other_part_type_pose.second;
    	//other_part_pose.position.z+=0.025;
		up_pose=temp_pose;
		up_pose.position.z+=0.2;
		int n=0;
		arm.GripperToggle(true);
		ROS_INFO_STREAM("GOing to pick other part      "<<other_part_type_pose.first);
		while(1){
			if(agv_id==1){
			init_cam5_2_=1;
			reposition_part_type_=other_part_type_pose.first;
			reposition_part_pose_=temp_pose;
			ros::Duration(0.25).sleep();
			ros::spinOnce();
			reposition_part_pose_back_.position.z+=0.02;
			}


			if(agv_id==2){
			init_cam5_2_dash_=1;
			reposition_part_type_dash_=other_part_type_pose.first;
			reposition_part_pose_dash_=temp_pose;
			ros::Duration(0.25).sleep();
			ros::spinOnce();
			reposition_part_pose_back_dash_.position.z+=0.02;
			}



			if(agv_id==1){
			for (int i=0;i<n;i++)
				reposition_part_pose_back_.position.z-=0.005;
			}

			if(agv_id==2){
			for (int i=0;i<n;i++)
				reposition_part_pose_back_.position.z-=0.005;
			}

			if(agv_id==1){
			if (repose_flag_==1){
				arm.GoToTarget({up_pose,reposition_part_pose_back_});
				temp_pose=reposition_part_pose_back_;
			}
			else{
				ROS_INFO_STREAM("Reposition flag unsuccessfull");
				arm.GoToTarget({up_pose,temp_pose});
				temp_pose.position.z-=0.005;

			}
			}

			if(agv_id==2){
			if (repose_flag_dash_==1){
				arm.GoToTarget({up_pose,reposition_part_pose_back_dash_});
				temp_pose=reposition_part_pose_back_dash_;
			}
			else{
				ROS_INFO_STREAM("Reposition flag unsuccessfull");
				arm.GoToTarget({up_pose,temp_pose});
				temp_pose.position.z-=0.005;

			}
			}


			ros::Duration(0.1).sleep();
			arm.GoToTarget(up_pose);
			//other_part_pose.position.z-=0.005;
			//other_part_pose=reposition_part_pose_back_;
			if (arm.GetGripperStatus()==true){
				ROS_INFO_STREAM("Product attached");
				break;
			}

			n++;
		}
		init_cam5_2_=0;
		init_cam5_2_dash_=0;
		//picking up other part from temp position
		




		
		ros::Duration(1.0).sleep();
		part_pose.position.z+=0.15;
		while(1){
		    arm.GoToTarget(part_pose);
		    ros::spinOnce();
		    if (abs(arm.GetEEPose().position.x-part_pose.position.x)<0.1    and   abs(arm.GetEEPose().position.y-part_pose.position.y)<0.1     ){
		    	ROS_INFO_STREAM("State achieved");
		    	break;
		    }
		}

		ros::Duration(0.5).sleep();
		ros::spinOnce();
		if(arm.GetGripperStatus()==false){
			ROS_INFO_STREAM("Part was dropped before keeping");
			continue;
		}
		arm.GripperToggle(false);
		ROS_INFO_STREAM("Dropping other part to app location");
		//Dropping part at proper position

		ros::spinOnce();
		break;}
	}
    

    




    
    else{
    	ros::spinOnce();				
		if (diff_list.first=="pulley_part")
			part_pose.position.z+=0.08;
		
		ROS_INFO_STREAM("Normal reposition activated....");
		part_pose.position.z+=0.02;
		auto up_pose=part_pose;
		up_pose.position.z+=0.2;
		int n=0;
		arm.GripperToggle(true);
		ROS_INFO_STREAM("GOing to pick   "<<diff_list.first);
		while(1){
			arm.GoToTarget(part_pose);
			ros::spinOnce();
			arm.GoToTarget(up_pose);
			part_pose.position.z-=0.005;
			if (arm.GetGripperStatus()==true){
				ROS_INFO_STREAM("Product attached");
				break;
			}

		}

		

		


	    //Dropping part at specified posn
	    
	    arm.GoToTarget(StampedPose_out.pose);
	    ros::Duration(1.0).sleep();

	    if (arm.GetGripperStatus()==false){
	    	ROS_INFO_STREAM("Part was dropped somewhere while reposition before dropping");
	    	std::pair<std::string,geometry_msgs::Pose> temp;
	    	temp.first=diff_list.first;
	    	temp.second=diff_list.second.at(1);//in kit tray frame
	    	PickAndPlace(temp,agv_id);
	    }

	    
	    arm.GripperToggle(false);
		}
    

    ros::spinOnce();
	end_position_[1]=1.55;
	while(1){         //so that we know the arm is at agv1
		arm.SendRobotJointPosition(end_position_);
		ros::spinOnce();
		ROS_INFO_STREAM(arm.GetJointStates()[1]-end_position_[1]);
		if (arm.GetJointStates()[1]-end_position_[1]<0.1)
			break;
	}
	end_position_[1]=3.1;
    //arm1_.SendRobotJointPosition(end_position_);

    //arm1_.DropPart(StampedPose_out.pose);
    //arm1_.GripperToggle(false);



}


void AriacOrderManager::removeElement(osrf_gear::Order ord ){
	for(int i=0;i<received_orders_.size();i++)
		{if (ord.order_id==received_orders_[i].order_id){
			received_orders_.erase(received_orders_.begin()+i);
			return;}
		}
}

void AriacOrderManager::ExecuteOrder() {
    ROS_WARN(">>>>>> Executing order...");
    //scanned_objects_ = camera_.GetParts();

    //-- used to check if pick and place was successful
    while (1)//Wait to receive 1st order
        {ros::spinOnce();
            if (received_orders_.size()!=0)
                break;
        }

    

    arm1_.SendRobotHome();
    //ros::Duration(1.5).sleep();
    ROS_INFO_STREAM("ARM2");
    arm2_.SendRobotHome();
    ros::spinOnce();
    //ros::Duration(1.5).sleep();
    //ROS_INFO_STREAM("Will send arm22 behind");
    //joint_states_2_[0]-=0.6;     //taking arm2 away
	//auto temp1=joint_states_2_;
	//auto temp=joint_states_2_[0];
	//ros::spinOnce();
	//ros::Duration(1.5).sleep();
	//while(1){         //so that we know the wrist has turned
	//   			ROS_INFO_STREAM("SENDING ARM2 BEHIND");
	//   			arm2_.SendRobotJointPosition(temp1);ros::spinOnce();
	//   			ROS_INFO_STREAM(joint_states_2_[0]-temp);
	//   			if (abs(joint_states_2_[0]-temp)<0.1)
	//   				break;
	//   		}
	

	//printVector(joint_states_);

    //arm1_.SendRobotConveyer();

    current_order_=received_orders_.at(0);
    removeElement(current_order_);
    
    bool pick_n_place_success{false};

    std::list<std::pair<std::string,geometry_msgs::Pose>> failed_parts;

    ros::spinOnce();
    // ros::Duration(1.0).sleep();
    product_frame_list_ = camera_.get_product_frame_list();
    
    while (1){


	    CompletethisOrder(current_order_);
	    osrf_gear::Order order_update;
	    order_update.order_id="None";
	    ros::Duration(3.0).sleep();    //Waiting to see if there is a order update
	    ros::spinOnce();
	    if (order_update_flag_==1){
	    	
	    	for(const auto &ord:received_orders_){
	    		if(current_order_.order_id==ord.order_id.substr(0,current_order_.order_id.size())) {
	    			order_update=ord;
	    			removeElement(ord);

    				
	    			//removing order update from received orders
	    		}
	    	}
	    	if (order_update.order_id!="None"){
	    		CompleteOrderUpdate(order_update);
	    		order_update_flag_=0;
	    	}
	    }

	    ros::spinOnce();
	    int no=0;
	    if(received_orders_.size()==0){   //Waiting for new order to arrive if none has
	    	while(received_orders_.size()==0){
	    		ros::Duration(1.0).sleep();
	    		ros::spinOnce();
	    		ROS_INFO_STREAM("Waiting for new Order");
	    		no++;
	    	}
    		
	    }
	    if (received_orders_.size()==0){
	    	ROS_INFO_STREAM("No new order received Finished the code.....");
	    	break;
	    }
	    
	    current_order_=received_orders_.at(0);
		received_orders_.erase(received_orders_.begin());
		ROS_INFO_STREAM("Order Finished going for next order....  "<<current_order_.order_id);
   
		}		
   }
   

   // for (const auto &order:received_orders_){
   //       //auto order_id = order.order_id;
   //       auto shipments = order.shipments;
   //       for (const auto &shipment: shipments){
   //           auto shipment_type = shipment.shipment_type;
   //            for (const auto &product: shipment.products){
   //              ROS_INFO_STREAM("Product to place is  "    <<product.type);
   //              std::pair<std::string,geometry_msgs::Pose> product_type_pose;
   //              product_type_pose.first=product.type;
   //              product_type_pose.second=product.pose;
   //              if (product_frame_list_.find(product.type)!=product_frame_list_.end())  //Found in list
   //                PickAndPlace(product_type_pose, int(1));
   //              else{
   //                ROS_INFO_STREAM("Please pick up from conveyer"    <<product.type);
   //                PickFromConveyer(product_type_pose.first);
   //                bool result,result1;
   //                geometry_msgs::Pose drop_pose = product_type_pose.second;

			//       geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;

			//       // if(agv_id==1){
			//       StampedPose_in.header.frame_id = "/kit_tray_1";
			//       StampedPose_in.pose = drop_pose;
			//       //ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
			//       part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
			//       StampedPose_out.pose.position.z += 0.1;
			//       StampedPose_out.pose.position.y -= 0.2;
			//       //ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");

			//     // }
			//       ros::Duration(0.5).sleep();
   //                while(1) {

	  //                 ros::spinOnce();
	  //                 if (arm1_.GetGripperStatus()==false){
	  //                 	//arm1_.SendRobotConveyer();
	  //                 	PickFromConveyer(product_type_pose.first);
	  //                 	continue;
	  //                 }

	  //                 result = arm1_.DropPart(StampedPose_out.pose);
	  //                 ROS_INFO_STREAM("result from DropPart: "<< result);

			// 	    // bool result1;
			// 	    if (result==-1){
			// 	      ROS_INFO_STREAM("Retrying due to defected part");
			// 	      arm1_.SendRobotConveyer();
			// 	      arm1_.GripperToggle(false);
			// 	      continue;
				      

			// 	  }
			// 	  if (result==0) {
			// 	  	ROS_INFO_STREAM("Breaking out......");
			// 	  	break;
			// 	  }
			// }
			    

   //                  }
                
   //              }
   //              //Write function for pick and place from conveyer
   //              ROS_INFO_STREAM("Shipment completed Please send back AGV");
   //              SubmitAGV(1);
   //              ros::spinOnce();
   //              //SubmitAGV(1);
   //              }
   //            ROS_INFO_STREAM("Congragulations Order Completed");
   //            ros::spinOnce();
   //            }
             
//}


   


   





void AriacOrderManager::SubmitAGV(int num) {
    std::string s = std::to_string(num);
    ros::ServiceClient start_client =
            order_manager_nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv"+s);
    if (!start_client.exists()) {
        ROS_INFO("Waiting for the client to be ready...");
        start_client.waitForExistence();
        ROS_INFO("Service started.");
    }

    osrf_gear::AGVControl srv;
    // srv.request.kit_type = "order_0_kit_0";
    start_client.call(srv);

    if (!srv.response.success) {
        ROS_ERROR_STREAM("Service failed!");
    } else
        ROS_INFO("Service succeeded.");
}