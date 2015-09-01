#include "ros/ros.h"
#include "robotino_local_move/FullPath.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robotino_manager_client");
  if (argc != 2)
  {
    ROS_INFO("Usage: manager_client goal (eg. manager_client 12");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<robotino_local_move::FullPath>("FullPath");
  robotino_local_move::FullPath srv;
  srv.request.goal = atoll(argv[1]);
  if (client.call(srv))
  {
    ROS_INFO("Path: %s", srv.response.full_path.c_str());
  }
  else
  {
    ROS_ERROR("Failed to generate path!");
    return 1;
  }

  return 0;
}
