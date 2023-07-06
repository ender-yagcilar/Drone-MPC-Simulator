#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>//Droneu armla
#include <mavros_msgs/ParamSet.h>//Her şey başlarken simülasyondaki DLL ve RCL'yi 0a eşitle
#include <mavros_msgs/State.h>//Drone statei al
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <thread>
#include <math.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>


#define PI 3.14159265359
#define LOOP_RATE 20



struct Point3D{
    float x;
    float y;
    float z;
};

class Drone{
    public:
        Drone(int argc,char** argv,int);
        ~Drone();
        void work();

    private:
        int argc;
        char** argv;
        int ID;
        

        ros::NodeHandle nh;

        /*Drone Position Trajectory*/
        std::vector<Point3D> pos_traj {
            {0,0,2},
        };

        //EFE MPC
        /*Drone Acceleration Initial values,first is old values and the second is current values*/
        float body_acc[2][3] = {{0,0,0},{0,0,0}};
        float delta_bodyrate[3] = {0,0,0};
        geometry_msgs::TwistStamped prev_mpc_msg;
        geometry_msgs::TwistStamped curr_mpc_msg;
        double time_diff;
        float kg;//Thrust in kg units

        
        
        
        /*Messages are listed below*/
        mavros_msgs::CommandBool arm_cmd;
        mavros_msgs::ParamSet param;
        mavros_msgs::State state;
        geometry_msgs::PoseStamped odom;//Konum datası -> local_positiondan çekiliyor
        geometry_msgs::PoseStamped pos_msg;//hedef datası -> setpoint_position/local'e publanıyor
        mavros_msgs::SetMode fly_mode;//Offboard moda almak için kullancağım değişken
        mavros_msgs::CommandTOL land_msg;//Land servisine verilecek mesaj bu
        geographic_msgs::GeoPointStamped gp_origin_msg;
        mavros_msgs::AttitudeTarget att_msg;
        mavros_msgs::PositionTarget altitude_msg;
        geometry_msgs::TwistStamped mpc_cb_msg;
        
        
        //Ros Publishers,Subscribers and Clients
        ros::ServiceClient param_set_client;
        ros::ServiceClient arming_client;
        ros::ServiceClient mode_client;
        ros::ServiceClient land_client;
        ros::ServiceClient glob_pos_client; 
    
        ros::Subscriber state_sub;
        ros::Subscriber odom_sub;
        ros::Subscriber MPC_sub;

        ros::Publisher pos_pub;
        ros::Publisher gp_origin_pub;
        ros::Publisher att_pub;
        ros::Publisher altitude_pub;
        
  

        /*Callbacks*/
        void state_cb(const mavros_msgs::State::ConstPtr &msg);
        void odom_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void MPC_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);

        /*Methods*/
        void wait_connection();//Waits till connected
        void set_sim_params();
        void mode_offboard();
        void arm();
        void gp_origin_set();
        int reached_status();
        void go_position(float x,float y,float z);
        void mode_land();
        void attitude_msg_angle_set(double roll,double pitch,double yaw);//Bunu kullanmıyoruz çünkü rate kullanıyoruz
        void attitude_msg_rate_set();//Bu metotla angular velocityleri veriyoruz. 
        //Açısal ivme modu olmadığından trapezoidal numerik toplamla açısal hız veriyoruz.
        void attitude_msg_thrust_set();

        
        
        //Thread functions
        void goal_thr_func(double tolerance);
        void attitude_thr_func(int x);
        void altitude_thr_func(int x);

        /*Threads*/
        std::thread goal_thr;    bool pos_pub_bool = true;
        std::thread attitude_thr;bool is_mpc_publishing = false;
        std::thread altitude_thr;bool altitude_pub_bool=1;

        //Thrust PID Variables -> Nurullah MPC
        double Kp = 0.1;   // Proportional gain
        double Ki = 0.2;   // Integral gain
        double Kd = 0.1;   // Derivative gain

        double prevError = 0;
        double integral = 0;
        double pid_dt=0;

        double pid_thrust;

        double pid_calculate(double setpoint, double measuredValue, double dt);
};

