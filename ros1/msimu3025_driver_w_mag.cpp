// Simple driver for MSIMU3025
// Does not send any commands to the IMU, only reads data.
// Only looks for one message type from the IMU, then timestamps it and publishes the contents of that message
#include <ros/ros.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <termio.h>
#include <stdio.h>
#include <time.h>
#include <sys/ioctl.h> 
#include <linux/serial.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <shared_messages/Command.h>

#define PACKET_BYTES 54
static double G_last_move_time = 0.0;

// Returns one if the fletcher (checksum at end of packet) is correct
static int fletcher_valid(unsigned char* msg, int msg_size)
{
    int f1 = 0;
    int f2 = 0;
    for (int i = 0; i < (msg_size-2); i++) {
	f1 += msg[i];
	f2 += f1;
    }
    f1 = f1 % 256;
    f2 = f2 % 256;
    if ((f1 == msg[msg_size-2]) && (f2 == msg[msg_size-1])) {
	return 1;
    }
    ROS_INFO_THROTTLE(0.1, "fletcher_valid failed %x %x %x %x\n", f1, msg[msg_size-2], f2, msg[msg_size-1]);
    return 0;
}

// Would probably be better in its own node
static void commandCallback(shared_messages::Command msg)
{
    if (msg.chan8 > 1600) { // move allowed
	G_last_move_time = ros::Time::now().toSec();
    }
}


int main(int argc,char** argv)
{   
  ros::init(argc, argv, "msimu3025_driver");
  ros::NodeHandle *n = new ros::NodeHandle("~");

  // Wait for ros to start up
  ros::Time last_ros_time_;
  bool wait = true;
  while (wait) {
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0)
      wait = false;
  }

  // Get the path to the serial port, but have a reasonable default set
  std::string s = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0";
  n->getParam("serial_path_msimu3025", s);

  // Subscriber for commands to know when robot is still
  static ros::Subscriber command_sub_ = n->subscribe("/drone_commands", 5, commandCallback);
  
  // Create a publisher to send the ros messages to
  ros::Publisher pub_msimu3025 = n->advertise<sensor_msgs::Imu>("/msimu3025_raw", 1);
  ros::Publisher pub_mag = n->advertise<sensor_msgs::MagneticField>("/msimu3025_magnetic", 1);
  ros::Publisher pub_msimu3025_temperature = n->advertise<sensor_msgs::Temperature>("/msimu3025_temperature", 1);

  // Open and set up the serial port
  struct termios tio;
  int tty_fd;
  unsigned char c;

  if((tty_fd = open(s.c_str() , O_RDWR )) == -1){
    ROS_ERROR("Error opening serial port\n");
    return -1;
  }

  // This section is needed otherwise the serial reads will be bunched into groups
  struct serial_struct serial;
  ioctl(tty_fd, TIOCGSERIAL, &serial); 
  serial.flags |= ASYNC_LOW_LATENCY; // (0x2000)
  ioctl(tty_fd, TIOCSSERIAL, &serial);

  memset(&tio,0,sizeof(tio));
  tio.c_iflag=0;
  tio.c_oflag=0;
  tio.c_cflag=CS8|CREAD|CLOCAL;
  tio.c_lflag=0;
  tio.c_cc[VMIN]=1;
  tio.c_cc[VTIME]=5;
  cfsetospeed(&tio,B460800);    
  cfsetispeed(&tio,B460800);
  tcsetattr(tty_fd,TCSANOW,&tio);

  unsigned char bytes[540];
  int bytes_avail = 0;
  unsigned int calcval2 = 0;
  sensor_msgs::Imu imu_msg;
  imu_msg.orientation_covariance[0] = -1.0; // Since no orientation is published
  imu_msg.header.frame_id = "msimu3025_frame";
  imu_msg.angular_velocity_covariance[0] = 0.62 * 2.0 * M_PI / 360.0 / 3600.0;
  imu_msg.angular_velocity_covariance[4] = 0.56 * 2.0 * M_PI / 360.0 / 3600.0;
  imu_msg.angular_velocity_covariance[8] = 0.80 * 2.0 * M_PI / 360.0 / 3600.0;
  imu_msg.linear_acceleration_covariance[0] = 2.6 * (1e-6) * 9.80665;
  imu_msg.linear_acceleration_covariance[4] = 2.6 * (1e-6) * 9.80665;
  imu_msg.linear_acceleration_covariance[8] = 6.7 * (1e-6) * 9.80665;
  sensor_msgs::MagneticField mag_msg;
  mag_msg.magnetic_field_covariance[0] = 0.0016 / 10000.0;
  mag_msg.magnetic_field_covariance[4] = 0.0016 / 10000.0;
  mag_msg.magnetic_field_covariance[8] = 0.0016 / 10000.0;
  sensor_msgs::Temperature imu_temperature;
  imu_temperature.variance = 1.5;
  
  while (ros::ok()) {
    ros::spinOnce();
    if (read(tty_fd,&c,1) > 0) {
      bytes[bytes_avail] = c;
      bytes_avail++;
      
      if (bytes_avail >= PACKET_BYTES) {
        // Test for valid block
        if ((bytes[0] == 0xa5) && (bytes[1] == 0xa5) && (bytes[2] == 0xa2) && (bytes[3] == PACKET_BYTES - 6)) {
          if (fletcher_valid(bytes, PACKET_BYTES)) {
            /*printf("packet: ");
            for(int i = 0; i < bytes_avail; i++) {
              if(bytes[i] <= 0xf) printf("0");
              printf("%X", bytes[i]);
            }
            printf("\n");*/
            for(int i = 4; i < PACKET_BYTES && ros::ok(); i += bytes[i + 1] + 2) {
              switch(bytes[i]) {
                case 0x81:
                  if(bytes[i + 1] == 0x0c) {
                    // acceleration - convert from g to m/s2
                    calcval2 = (bytes[i + 2] << 24) |  (bytes[i + 3] << 16) |  (bytes[i + 4] << 8) | bytes[i + 5];
                    imu_msg.linear_acceleration.x = 9.80665 * (*(float*) &calcval2);
                    
                    calcval2 = (bytes[i + 6] << 24) |  (bytes[i + 7] << 16) |  (bytes[i + 8] << 8) | bytes[i + 9];
                    imu_msg.linear_acceleration.y = 9.80665 * (*(float*) &calcval2);
                    
                    calcval2 = (bytes[i + 10] << 24) |  (bytes[i + 11] << 16) |  (bytes[i + 13] << 8) | bytes[i + 14];
                    imu_msg.linear_acceleration.z = 9.80665 * (*(float*) &calcval2);
                  }
                  break;
                case 0x82:
                  if(bytes[i + 1] == 0x0c) {
                    // gyro - convert from degrees per second to radians per second
                    calcval2 = (bytes[i + 2] << 24) |  (bytes[i + 3] << 16) |  (bytes[i + 4] << 8) | bytes[i + 5];
                    imu_msg.angular_velocity.x = 2.0 * M_PI * (*(float*) &calcval2) / 360.0;
                    
                    calcval2 = (bytes[i + 6] << 24) |  (bytes[i + 7] << 16) |  (bytes[i + 8] << 8) | bytes[i + 9];
                    imu_msg.angular_velocity.y = 2.0 * M_PI * (*(float*) &calcval2) / 360.0;
                    
                    calcval2 = (bytes[i + 10] << 24) |  (bytes[i + 11] << 16) |  (bytes[i + 13] << 8) | bytes[i + 14];
                    imu_msg.angular_velocity.z = 2.0 * M_PI * (*(float*) &calcval2) / 360.0;
                  }
                  break;
                case 0x83:
                  if(bytes[i + 1] == 0x0c) {
                    // magnetic - convert from gauss to tesla
                    calcval2 = (bytes[i + 2] << 24) |  (bytes[i + 3] << 16) |  (bytes[i + 4] << 8) | bytes[i + 5];
                    mag_msg.magnetic_field.x = (*(float*) &calcval2) / 10000.0;
                    
                    calcval2 = (bytes[i + 6] << 24) |  (bytes[i + 7] << 16) |  (bytes[i + 8] << 8) | bytes[i + 9];
                    mag_msg.magnetic_field.y = (*(float*) &calcval2) / 10000.0;
                    
                    calcval2 = (bytes[i + 10] << 24) |  (bytes[i + 11] << 16) |  (bytes[i + 13] << 8) | bytes[i + 14];
                    mag_msg.magnetic_field.z = (*(float*) &calcval2) / 10000.0;
                  }
                  break;
                case 0x87:
                  if(bytes[i + 1] == 0x04) {
                    // temperature
                    calcval2 = (bytes[i + 2] << 24) |  (bytes[i + 3] << 16) |  (bytes[i + 4] << 8) | bytes[i + 5];
                    imu_temperature.temperature = *(float*) &calcval2;
                  }
                  break;
              }
            }
            
	    // IMU was sometimes kicking out numbers like 3.5e21  Those huge numbers would cause
	    // the filter to fail.  Filter those out here.  In particular it would be all
	    // three angular velocities, but the accelerations would be fine.
	    // Not sure how it made it through the fletcher valid check above or what is going wrong.
	    // Good feature would be to log the error and all of the bytes to see if there is a pattern.
	    // On some robots it never happens and others it is once every few  minutes or so.
	    if ((fabs(imu_msg.angular_velocity.x) < 50.0) && (fabs(imu_msg.angular_velocity.y) < 50.0) &&
		(fabs(imu_msg.angular_velocity.z) < 50.0) && (fabs(imu_msg.linear_acceleration.x) < 500.0) &&
		(fabs(imu_msg.linear_acceleration.y) < 500.0) && (fabs(imu_msg.linear_acceleration.z) < 500.0)) {

		// Set the offset
		static int zcnt = 0;
		static double zs[2000];
		static double zoffset = 0.0;
		static double last_cal = 0.0;
		zcnt++;
		zcnt = zcnt % 2000;
		zs[zcnt] = imu_msg.angular_velocity.z;
		double timenow = ros::Time::now().toSec();
		if (((timenow - last_cal) > 16.0) && ((timenow - G_last_move_time) > 13.0)) {
		    last_cal = timenow;
		    double zsum = 0.0;
		    double zmax = 0.0;
		    for (int i = 0; i < 2000; i++) {
			zsum = zsum + zs[i];
			if (fabs(zs[i]) > zmax) {
			    zmax = fabs(zs[i]);
			}
		    }
		    if ((zmax < 0.01) && (zsum < 2.0)) { // zmax could be as low as 0.003
			zoffset = zsum / 2000.0;
			//printf("Cal worked %lf %lf %lf\n", zmax, zoffset, ros::Time::now().toSec() - timenow);
		    }
		    else {
			//printf("Cal failed %lf %lf\n", zmax, zoffset);
		    }
		}
		imu_msg.angular_velocity.z = imu_msg.angular_velocity.z - zoffset;

		imu_msg.header.stamp = ros::Time::now();
		imu_msg.header.seq++;
		mag_msg.header = imu_msg.header;
		imu_temperature.header = imu_msg.header;

		pub_msimu3025.publish(imu_msg);
		pub_mag.publish(mag_msg);
		pub_msimu3025_temperature.publish(imu_temperature);
	    }
          }
          bytes_avail = 0;
        } else { // syncing
          if (bytes_avail == PACKET_BYTES) {
            // If we have enough bytes, but didn't match, shift the bytes by 1
            bytes_avail--;
            for (int i = 0; i < bytes_avail; i++) {
                bytes[i] = bytes[i + 1];
            }
          }
          //ROS_INFO_THROTTLE(0.1, "syncing %x %d\n", c, bytes_avail);
          //fprintf(stderr, "syncing %x %d\n", c, bytes_avail);
        }
      }
    }
  }
  close(tty_fd);
  return EXIT_SUCCESS;
}
