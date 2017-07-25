#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "sample_source");

  ros::NodeHandle nh;

  std::string topic_name;
  if (!ros::param::get("/source_topic_name", topic_name))
  {
    ROS_ERROR("Please ensure source_topic_name parameter is set for this test");
    return 1;
  }

  auto chatter_pub = nh.advertise<std_msgs::String>(topic_name, 10);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std_msgs::String msg;

    msg.data = std::string("hello world");

    ROS_DEBUG("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}