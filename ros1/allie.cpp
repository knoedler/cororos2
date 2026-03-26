#include <boost/shared_ptr.hpp>
#include <robot_control_fsm/robot_control_fsm.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <vesc_msgs/VescStateStamped.h>

using namespace std;

// Settings for Behaviours
const float front_x_offset         = -0.49;
      float front_window_y_lidar[] = {-0.305, 0.305};
const float controller_angular_K  = 1.0;
const float controller_linear_K   = 0.6;
const float drive_limit_mult      = 0.25;
const float drive_limit_speed     = 2.1;
const float drive_width           = 0.635;

// Globals
boost::shared_ptr<RobotControlFSM::RobotControlFSM> controller;
static int fd;

// Publishers
ros::Publisher debug_drive_control_publisher;

// Subscribers
ros::Subscriber estop_status;
ros::Subscriber battery_status;

//Callback handlers
void DriveCommandHandler(const geometry_msgs::Twist data);
void EStopHandler(const std_msgs::Int32 data);
void BatteryHandler(sensor_msgs::BatteryState msg);

//Helpler functions
int maestroSetTarget(unsigned char channel, unsigned short target);

int main(int argc, char **argv) {
  ros::init(argc, argv, "behaviours", ros::init_options::NoSigintHandler);
  ros::NodeHandle mNH;
  
  // Open the Maestro's virtual COM port.
  //const char * device = "/dev/ttyACM0";  // Linux
  const char * device = "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Micro_Maestro_6-Servo_Controller_00242199-if00";  // Linux
  fd = open(device, O_RDWR | O_NOCTTY);
  if (fd == -1) {
      device = "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Micro_Maestro_6-Servo_Controller_00357982-if00";  // Linux
      fd = open(device, O_RDWR | O_NOCTTY);
      if (fd == -1) {
	  perror(device);
	  return 1;
      }
  }
 
  struct termios options;
  tcgetattr(fd, &options);
  options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
  options.c_oflag &= ~(ONLCR | OCRNL);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tcsetattr(fd, TCSANOW, &options);
  
  //publishers
  debug_drive_control_publisher = mNH.advertise<geometry_msgs::Twist>("/cmd_vel_debug", 10, true);
  
  //subscribers
  estop_status = mNH.subscribe<std_msgs::Int32>("/estop_arduino", 0, EStopHandler);
  battery_status = mNH.subscribe("/mavros/battery", 1, BatteryHandler);
  
  controller.reset(new RobotControlFSM::RobotControlFSM(mNH, &DriveCommandHandler, controller_angular_K, controller_linear_K, front_window_y_lidar, front_x_offset));

  ros::spin();
  
  controller.reset();
  
  close(fd);

  return EXIT_SUCCESS;
}

void DriveCommandHandler(const geometry_msgs::Twist data) {
  debug_drive_control_publisher.publish(data);
  
  int maxval = 6700;    // enough power for skid steering
	int minval = 5300;
	unsigned short target1 = 6000 + 2000 * (data.linear.x - data.angular.z);
	//unsigned short target1 = 6000 - 2000 * (data.linear.x + data.angular.z); // ames
	if (target1 < minval) { target1 = minval; }
	if (target1 > maxval) { target1 = maxval; }
	unsigned short target2 = 6000 - 2000 * (data.linear.x + data.angular.z);
	//unsigned short target2 = 6000 + 2000 * (data.linear.x - data.angular.z); // ames
	if (target2 < minval) { target2 = minval; }
	if (target2 > maxval) { target2 = maxval; }
  
  maestroSetTarget(0, target1);
	maestroSetTarget(1, target2);
	maestroSetTarget(2, target2);
	maestroSetTarget(3, target1);
}

void EStopHandler(const std_msgs::Int32 msg) {
  controller->setExtEStopStatus(msg.data);
}

void BatteryHandler(sensor_msgs::BatteryState msg) {
  controller->setBatteryVoltage(msg.voltage);
}
 
// Sets the target of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
// The units of 'target' are quarter-microseconds.
int maestroSetTarget(unsigned char channel, unsigned short target)
{
  unsigned char command[] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F};
  if (write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
  return 0;
}
