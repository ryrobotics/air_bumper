#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Eigen>

using namespace Eigen;
using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Global Var <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int cnt = 0;

Vector3d accel;

int imu_flg = false;
int imu_thr = 20;		// threshold for collision detection
uint32_t imu_sw = 10;	// sliding window for reducing false detection

ros::Publisher crashPub;
ros::Publisher imu_pub;
geometry_msgs::PointStamped crashROS;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Callback Functions <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
	current_state = *msg;
	if (current_state.armed)
		ROS_INFO_STREAM_ONCE("Vehicle armed");
}

// geometry_msgs::TwistStamped vel_state;
// void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
// {
// 	vel_state = *msg;
// }

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> IMU-based Collision Detection <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

uint32_t detect_start;
double accel_max = 0;
sensor_msgs::Imu imu_state;

void check_collision_accel(double accel_axis, double &accel_max, const string &axisName) {
    double adjustment = (axisName == "Z") ? 9.8 : 0.0; // Adjust for gravity on Z-axis
    double corrected_accel = accel_axis - adjustment;

    if (abs(corrected_accel) > imu_thr) {
        if ((imu_state.header.seq - detect_start) > imu_sw) {
            cnt++;
            detect_start = imu_state.header.seq;
            accel_max = 0;
            accel.setZero();
            cout << "[" << cnt << "]: " << axisName << " axis" << endl;
        }

        if (abs(corrected_accel) > accel_max &&
            (imu_state.header.seq - detect_start) < imu_sw) {
            imu_flg = true;
            accel_max = abs(corrected_accel);

            accel[0] = imu_state.linear_acceleration.x;
            accel[1] = imu_state.linear_acceleration.y;
            accel[2] = imu_state.linear_acceleration.z - 9.8;
        }
    }
}

void imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
	imu_state = *msg;
	imu_pub.publish(imu_state);

	// ROS_INFO("IMU seq: [%d]", imu_state.header.seq);
	// ROS_INFO("IMU accel x: [%f], y: [%f], z: [%f], w: [%f]",
	//          imu_state.linear_acceleration.x, imu_state.linear_acceleration.y, imu_state.linear_acceleration.z);

    // Check for collision on each axis
    check_collision_accel(imu_state.linear_acceleration.x, accel_max, "X");
    check_collision_accel(imu_state.linear_acceleration.y, accel_max, "Y");
    check_collision_accel(imu_state.linear_acceleration.z, accel_max, "Z");

	if (imu_flg == true &&
		((imu_state.header.seq - detect_start) > imu_sw)) // Each detect horizon only record the max value
	{
		// accel direction is opposite to the collision direction
		Vector3d accc = - accel; 

		double phi = atan2(accc(1), accc(0)) * 180 / M_PI;
		double theta = acos(accc(2) / accc.norm()) * 180 / M_PI;

		cout << "IMU accel x: [" << accel[0] << "], y: [" << accel[1] << "], z: [" << accel[2] << "]" << endl;
		cout << "Collision Detected at [" << phi << "] deg (X-Y)" << " [" << theta << "] deg (Z)" << endl;
		cout << ">>>>>>>>>>>> Collision Detected <<<<<<<<<<<<" << endl;

		// Publish collision information
		crashROS.header.frame_id = string("world");
		crashROS.header.stamp = msg->header.stamp;
		crashROS.point.x = accc[0];
		crashROS.point.y = accc[1];
		crashROS.point.z = accc[2];
		crashPub.publish(crashROS);

		imu_flg = false;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_detection");
	ros::NodeHandle nh;

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, imu_cb);

	// ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/livox/imu", 10, imu_cb);
	// ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 1, vel_cb);

	crashPub = nh.advertise<geometry_msgs::PointStamped>("imu_detection/crash_point", 10);
	imu_pub = nh.advertise<sensor_msgs::Imu>("/usr_imu", 10);

	// the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(20.0);

	// wait for FCU connection
	while (ros::ok() && !current_state.connected)
	{
		ROS_INFO_STREAM_ONCE("connecting to FCU...");
		ros::spinOnce();
		rate.sleep();
	}

	ROS_INFO_STREAM_ONCE("FCU Connected...");
	for (int i = 100; ros::ok() && i > 0; --i)
	{
		ros::spinOnce();
		rate.sleep();
	}

	ros::spin();

	return 0;
}
