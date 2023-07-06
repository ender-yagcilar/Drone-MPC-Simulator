#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>


int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "mpc_node");

  // Create a node handle
  ros::NodeHandle nh;

  // Publish the four float numbers
  ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped>("MPC_outputs", 20);

  // Create a FourFloats message
  geometry_msgs::TwistStamped msg;


  ros::Rate rate(5);

  while (ros::ok())
  {
    

    msg.header.stamp = ros::Time::now();
    msg.twist.linear.x = 1.535*9.81;
    msg.twist.angular.x = 0.00;
    msg.twist.angular.y = 0.00;
    msg.twist.angular.z = 0.06;
    // Publish the message
    pub.publish(msg);

    // Spin once and sleep for the rest of the cycle
    ros::spinOnce();
    rate.sleep();
  }

  // Spin
  ros::spin();

  return 0;
}
