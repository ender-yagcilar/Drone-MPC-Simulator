#include "drone.h"

double Drone::pid_calculate(double setpoint, double measuredValue, double dt) {
    double error = setpoint - measuredValue;


    // Proportional term
    double pTerm = Kp * error;

    // Integral term
    integral += error * dt;
    double iTerm = Ki * integral;

    // Derivative term
    double dTerm = Kd * (error - prevError) / dt;
    prevError = error;

    // Calculate the output
    double output = pTerm + iTerm + dTerm;

    return output;
}

Drone::Drone(int argc,char** argv,int drone_ID){
    this->argc = argc;
    this->argv = argv;
    
    this->ID = drone_ID;
    
}

Drone::~Drone(){
    goal_thr.join();
    attitude_thr.join();
};

void Drone::state_cb(const mavros_msgs::State::ConstPtr &msg){
    
    state = *msg;
}

void Drone::odom_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    odom = *msg;
}


void Drone::MPC_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
    mpc_cb_msg = *msg;
    is_mpc_publishing = true;
    pos_pub_bool = false;
}

void Drone::wait_connection(){

    ros::Rate loop_rate(LOOP_RATE);

    while(ros::ok() && state.connected ==false){

        ROS_INFO("Not connected,waiting...%d",state.connected);
        ros::spinOnce();
        ros::Duration(2.0).sleep();
    }
   
    
}
void Drone::set_sim_params(){
    param.request.value.integer = 0;
    param.request.param_id = "NAV_DLL_ACT";
   
    if(param_set_client.call(param)==true && param.response.success == 1){
        ROS_INFO("NAV_DLL_ACT is set to 0.\n");
    }

    param.request.param_id = "NAV_RCL_ACT";
    if(param_set_client.call(param)==true && param.response.success == 1){
        ROS_INFO("NAV_RCL_ACT is set to 0.\n");
    }
}

void Drone::mode_offboard(){

    ros::Rate loop_rate(LOOP_RATE);
    for(int i = 0;i<100 && ros::ok();i++){
        pos_msg.pose.position.x = 0;
        pos_msg.pose.position.y = 0;
        pos_msg.pose.position.z = 0;
        pos_pub.publish(pos_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    fly_mode.request.custom_mode = "OFFBOARD";
    if(mode_client.call(fly_mode) == true){
        if(fly_mode.response.mode_sent == true){
            ROS_INFO("\nOFFBOARD mode: Active!\n");
        }
    }
}

void Drone::mode_land(){

    fly_mode.request.custom_mode = "AUTO.LAND";
    if(mode_client.call(fly_mode) == true){
        if(fly_mode.response.mode_sent == true){
            ROS_INFO("\nLAND mode: Active!\n");
        }
    }
}


void Drone::arm(){
    arm_cmd.request.value = 1;
    if(arming_client.call(arm_cmd) == true && arm_cmd.response.result == 0){
        ROS_INFO("Drone is armed!\n");
    }else{
        ROS_INFO("Drone couldn't armed!\n");
    }
}

void Drone::gp_origin_set(){

    ros::Rate looprate(20);
    gp_origin_msg.position.latitude=  47.3977694;
    gp_origin_msg.position.longitude= 8.5456340;
    gp_origin_msg.position.altitude=  535.125;

    while(ros::ok()){
        
        
        gp_origin_pub.publish(gp_origin_msg);
        ros::spinOnce();
        looprate.sleep();
        break;

    }
}

int Drone::reached_status(){
    float delta_x = abs(pos_msg.pose.position.x - odom.pose.position.x);
    float delta_y = abs(pos_msg.pose.position.y - odom.pose.position.y);
    float delta_z = abs(pos_msg.pose.position.z - odom.pose.position.z);

    float dist  = sqrt(  pow(delta_x,2) +   pow(delta_y,2) + pow(delta_z,2));

    if(dist < 0.15 ){
        return 1;
    }
    else{
        return 0;
    }
}

void Drone::goal_thr_func(double tolerance){
    ros::Rate looprate(20);

    while(ros::ok() && pos_pub_bool){
        
        pos_pub.publish(pos_msg);
            
        ros::spinOnce();
        looprate.sleep();


    }
}

void Drone::go_position(float x,float y,float z){
    ros::Rate looprate(20);

    while(ros::ok()){
        pos_msg.pose.position.x = x;
        pos_msg.pose.position.y = y;
        pos_msg.pose.position.z = z;
        
        pos_pub.publish(pos_msg);
        
        if(reached_status()){
            break;
        }
        ros::spinOnce();
        looprate.sleep();


    }
}



void Drone::altitude_thr_func(int x){
    ros::Rate looprate(20);

    while(ros::ok()){
          // Set the desired altitude (Z-axis)
        altitude_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        
        
        altitude_msg.type_mask = //mavros_msgs::PositionTarget::IGNORE_PX |
                                //mavros_msgs::PositionTarget::IGNORE_PY |
                                //mavros_msgs::PositionTarget::IGNORE_PZ |
                                mavros_msgs::PositionTarget::IGNORE_VX |
                                mavros_msgs::PositionTarget::IGNORE_VY |
                                mavros_msgs::PositionTarget::IGNORE_VZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::IGNORE_YAW ;
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE ;
        altitude_msg.position.z = 5.0; // Set your desired positions
        altitude_msg.position.x = -5.0;
        altitude_msg.position.y = 0.0;

        // Publish the position setpoint message
        altitude_pub.publish(altitude_msg);

        ros::spinOnce();
        looprate.sleep();


    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void Drone::work(){

    //Servisler subscriberlar vs definitionı
    param_set_client = nh.serviceClient<mavros_msgs::ParamSet> ("/mavros/param/set");
    arming_client    = nh.serviceClient<mavros_msgs::CommandBool> ("/mavros/cmd/arming");
    mode_client      = nh.serviceClient<mavros_msgs::SetMode> ("/mavros/set_mode");
    land_client      = nh.serviceClient<mavros_msgs::CommandTOL> ("/mavros/cmd/land");

    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state",10,&Drone::state_cb,this);
    odom_sub  = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,&Drone::odom_cb,this);
    MPC_sub  = nh.subscribe<geometry_msgs::TwistStamped>("/MPC_outputs",10,&Drone::MPC_cb,this);

    pos_pub         = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
    gp_origin_pub   = nh.advertise<geographic_msgs::GeoPointStamped>("/mavros/global_position/set_gp_origin",10);
    att_pub         = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",10);
    altitude_pub    = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);
    

    pos_msg.pose.position.x = 0;
    pos_msg.pose.position.y = 0;
    pos_msg.pose.position.z = 0;
    

    ros::Rate loop_rate(LOOP_RATE);

    while(ros::ok()){

        
        wait_connection();
        gp_origin_set();
        set_sim_params();
        mode_offboard();
        arm();

        goal_thr = std::thread(&Drone::goal_thr_func,this,0.15);

        
        for(int i = 0;i<pos_traj.size() && ros::ok();){
            pos_msg.pose.position.x = pos_traj.at(i).x;
            pos_msg.pose.position.y = pos_traj.at(i).y;
            pos_msg.pose.position.z = pos_traj.at(i).z;
            if(reached_status() || is_mpc_publishing){
                i++;
            }
            
        }
        ROS_INFO("Position bitti");
 
        while(!is_mpc_publishing && ros::ok()){}//Mpc publishlemiyorsa bekle
        
        //pos_pub_bool=false;//Mpc publishlemeye başlayınca position basmayı bıraksın

        attitude_thr = std::thread(&Drone::attitude_thr_func,this,0.15);
        while(ros::ok()){}

        /*Land*/
        mode_land();



        ros::spinOnce();
        loop_rate.sleep();
        break;
    }
}

