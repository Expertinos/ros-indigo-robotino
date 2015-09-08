#include "invert_laser.h"

int main (int argc, char** argv)
{
	ros::init (argc, argv, "invert_laser_node");

	ros::NodeHandle n;

	Invert_Laser ril(n);

	ros::spin();

	return 0;

}
