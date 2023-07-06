#include "drone.h"


void Drone::attitude_msg_angle_set(double roll,double pitch,double yaw){
        //Bu metotla eğer drone'a açı verilecekse ilgili mesaja o açı değerleri konulur.
                
        //                       Setting  Thrust  Value
        pid_dt = (ros::Time::now() - att_msg.header.stamp).toSec();
        pid_thrust = pid_calculate(pos_traj[0].z, odom.pose.position.z, pid_dt);
        if(abs(pid_thrust)>0.5 ){pid_thrust = 0;}
        
        //                          Setting Angles

        tf::Quaternion quaternion;
        quaternion.setRPY(roll, pitch, yaw);
        
        att_msg.header.stamp = ros::Time::now();
        att_msg.orientation.x = quaternion.x();
        att_msg.orientation.y = quaternion.y();
        att_msg.orientation.z = quaternion.z();
        att_msg.orientation.w = quaternion.w();

        att_msg.thrust = 0.70705 + pid_thrust;


        att_msg.type_mask =
            mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE|
            mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE|
            mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

}

void Drone::attitude_msg_rate_set(){
        
    //                   Trapezoidal Rule For Integrating Acceleration
    time_diff = (curr_mpc_msg.header.stamp - prev_mpc_msg.header.stamp).toSec();

    delta_bodyrate[0] = time_diff * (prev_mpc_msg.twist.angular.x + curr_mpc_msg.twist.angular.x)/2 ;//Pitchteki angular velocitynin hesabı(angular acceleration trapezoidal rule yapılıyor)
    delta_bodyrate[1] = time_diff * (prev_mpc_msg.twist.angular.y + curr_mpc_msg.twist.angular.y)/2 ; 
    delta_bodyrate[2] = time_diff * (prev_mpc_msg.twist.angular.z + curr_mpc_msg.twist.angular.z)/2 ; 

    //                  Setting attitude angle rates
    //Bu metot ileride MPC'den gelen veriyi okuyup mesajı ayarlayacak
    att_msg.body_rate.x = att_msg.body_rate.x + delta_bodyrate[0];
    att_msg.body_rate.y = att_msg.body_rate.y + delta_bodyrate[1];
    att_msg.body_rate.z = att_msg.body_rate.z + delta_bodyrate[2];

    //                      Setting Thrust Value
    kg = mpc_cb_msg.twist.linear.x/9.81 ;
    att_msg.thrust =  0.8186062*kg - 0.3261397*kg*kg + 0.06053072*kg*kg*kg;

    //Aşağıdaki fonksiyon simülasyonda elde edilmiş data noktalarından elde edilmiş fittir.Bu data noktaları:
    //Thrust = 0.565   → 1.035 kg→              1→1,8318	kg
    //Thrust = 0.707035→ 1.535 kg→		           1→2,1710	kg
    //Thrust = 1	  → 2.635 kg→  N		  1→2.6350	kg
    //Bu data noktaları kullanılarak thrusta basılacak değer hesaplanıyor. Newtondan kgye 9.81'e bölerek geçilmelidir.
    //att_msg.thrust = 3,330669*10^-16 + 0,8186062*kg - 0,3261397*kg^2 + 0,06053072*kg^3;(10^-16'lı terim çok ufak olduğundan yok sayıldı)


    att_msg.type_mask = 
            mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
}


void Drone::attitude_thr_func(int x){
    ros::Rate looprate(20);

    att_msg.header.stamp = ros::Time::now();


    //att_msg.body_rate.x = 0;
    //att_msg.body_rate.y = 0;
    //att_msg.body_rate.z = 0;

    //prev_mpc_msg = mpc_cb_msg;
    //curr_mpc_msg = mpc_cb_msg;

    
    while(ros::ok()){
        
        attitude_msg_angle_set(mpc_cb_msg.twist.angular.x,mpc_cb_msg.twist.angular.y,mpc_cb_msg.twist.angular.z);//Açı verilecekse bu kullanılır
        //attitude_msg_rate_set();//Body rate verilecekse bu kullanılır.
        

        att_pub.publish(att_msg);

        //prev_mpc_msg = curr_mpc_msg;
        //curr_mpc_msg = mpc_cb_msg;

        ros::spinOnce();
        looprate.sleep();
    }
}

