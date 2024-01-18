#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
#include <iostream>
#include <fstream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

bool zero_orientation_set = false;

bool set_zero_orientation(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Zero Orientation Set.");
  zero_orientation_set = false;
  return true;
}

int main(int argc, char** argv)
{
  serial::Serial ser;
  std::string port;
  std::string tf_parent_frame_id;
  std::string tf_frame_id;
  std::string frame_id;
  double time_offset_in_seconds;
  bool broadcast_tf;
  double linear_acceleration_stddev;
  double angular_velocity_stddev;
  double orientation_stddev;
  uint8_t last_received_message_number;
  bool received_message = false;
  int data_packet_start;

  tf::Quaternion orientation;
  tf::Quaternion zero_orientation;

  ros::init(argc, argv, "INS550D");

  ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>("port", port, "/dev/ttyUSB0");
  private_node_handle.param<std::string>("tf_parent_frame_id", tf_parent_frame_id, "imu_base");
  private_node_handle.param<std::string>("tf_frame_id", tf_frame_id, "imu_link");
  private_node_handle.param<std::string>("frame_id", frame_id, "imu_link");
  private_node_handle.param<double>("time_offset_in_seconds", time_offset_in_seconds, 0.0);
  private_node_handle.param<bool>("broadcast_tf", broadcast_tf, true);
  private_node_handle.param<double>("linear_acceleration_stddev", linear_acceleration_stddev, 0.0);
  private_node_handle.param<double>("angular_velocity_stddev", angular_velocity_stddev, 0.0);
  private_node_handle.param<double>("orientation_stddev", orientation_stddev, 0.0);

  ros::NodeHandle nh("imu");
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("data", 200);
  ros::Publisher imu_temperature_pub = nh.advertise<sensor_msgs::Temperature>("temperature", 200);
  ros::Publisher imu_gps_pub = nh.advertise<sensor_msgs::NavSatFix>("gps", 200);
  ros::Publisher trigger_time_pub = nh.advertise<sensor_msgs::TimeReference>("trigger_time", 200);  ////

  ros::ServiceServer service = nh.advertiseService("set_zero_orientation", set_zero_orientation);

  ros::Rate r(200);  // 200 hz

  sensor_msgs::Imu imu;

  imu.linear_acceleration_covariance[0] = linear_acceleration_stddev;
  imu.linear_acceleration_covariance[4] = linear_acceleration_stddev;
  imu.linear_acceleration_covariance[8] = linear_acceleration_stddev;

  imu.angular_velocity_covariance[0] = angular_velocity_stddev;
  imu.angular_velocity_covariance[4] = angular_velocity_stddev;
  imu.angular_velocity_covariance[8] = angular_velocity_stddev;

  imu.orientation_covariance[0] = orientation_stddev;
  imu.orientation_covariance[4] = orientation_stddev;
  imu.orientation_covariance[8] = orientation_stddev;

  sensor_msgs::TimeReference trigger_time_msg;  ////
  sensor_msgs::Temperature temperature_msg;
  sensor_msgs::NavSatFix gps_msg;
  temperature_msg.variance = 0;
  gps_msg.position_covariance_type = 0;  // COVARIANCE_TYPE_UNKNOWN

  static tf::TransformBroadcaster tf_br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0, 0, 0));

  std::string input;
  std::string read;
  zero_orientation_set = false;
  double long i = 0;
  char xorcheck = 0;
  int Length = 58;  // data length 58
  // std::ofstream outfile;
  // outfile.open("~/dev/catkin_ws/src/ins550d/src/log.txt",std::ofstream::trunc);
  while (ros::ok())
  {
    try
    {
      if (ser.isOpen())
      {
        // read string from serial device
        if (ser.available())
        {
          read = ser.read(ser.available());
          ROS_DEBUG("read %i new characters from serial port, adding to %i characters of old input.", (int)read.size(),
                    (int)input.size());
          input += read;
          while (input.length() >= Length)  // while there might be a complete package in input
          {
            // parse for data packets
            data_packet_start = input.find(0xBD);

            if (data_packet_start != std::string::npos)
            {
              if (input.find(0x0B) - data_packet_start == 2)
              {
                // ROS_DEBUG("found possible start of data packet at position %d", data_packet_start);
                xorcheck = 0;
                // std::string pack;
                for (int i = 0; i < Length - 1; i++)
                {
                  xorcheck = xorcheck ^ input[data_packet_start + i];
                  // pack+=input[data_packet_start+i];
                }

                if (input[data_packet_start + Length - 1] == xorcheck)  // input[data_packet_start +Length-1]==xorcheck
                {
                  // ROS_DEBUG("seems to be a real data package: long enough and found end characters");
                  // printf("initialDATA:%x \n",input[data_packet_start+2]);

                  // get quaternion values
                  //  int16_t w = (((0xff &(char)input[data_packet_start + 2]) << 8) | 0xff
                  //  &(char)input[data_packet_start + 3]); int16_t x = (((0xff &(char)input[data_packet_start + 4]) <<
                  //  8) | 0xff &(char)input[data_packet_start + 5]); int16_t y = (((0xff &(char)input[data_packet_start
                  //  + 6]) << 8) | 0xff &(char)input[data_packet_start + 7]); int16_t z = (((0xff
                  //  &(char)input[data_packet_start + 8]) << 8) | 0xff &(char)input[data_packet_start + 9]);

                  // double wf = w/16384.0;
                  // double xf = x/16384.0;
                  // double yf = y/16384.0;
                  // double zf = z/16384.0;

                  // get RPY
                  short int roll =
                      ((0xff & (char)input[data_packet_start + 4]) << 8) | (0xff & (char)input[data_packet_start + 3]);
                  short int pitch =
                      ((0xff & (char)input[data_packet_start + 6]) << 8) | (0xff & (char)input[data_packet_start + 5]);
                  short int yaw =
                      ((0xff & (char)input[data_packet_start + 8]) << 8) | (0xff & (char)input[data_packet_start + 7]);

                  // calculate RPY in rad
                  short int* temp = (short int*)&roll;
                  float rollf = (*temp) * (360.0 / 32768) * (M_PI / 180.0);
                  temp = (short int*)&pitch;
                  float pitchf = (*temp) * (360.0 / 32768) * (M_PI / 180.0);
                  temp = (short int*)&yaw;
                  float yawf = (*temp) * (360.0 / 32768) * (M_PI / 180.0);

                  // tf::Quaternion orientation(xf, yf, zf, wf);
                  tf::Quaternion orientation;
                  orientation = tf::createQuaternionFromRPY(rollf, -pitchf, -yawf);

                  if (!zero_orientation_set)
                  {
                    zero_orientation = orientation;
                    zero_orientation_set = true;
                  }

                  // http://answers.ros.org/question/10124/relative-rotation-between-two-quaternions/
                  tf::Quaternion differential_rotation;
                  differential_rotation = zero_orientation.inverse() * orientation;

                  // get gyro values

                  short int gx =
                      ((0xff & (char)input[data_packet_start + 10]) << 8) | (0xff & (char)input[data_packet_start + 9]);
                  short int gy = ((0xff & (char)input[data_packet_start + 12]) << 8) |
                                 (0xff & (char)input[data_packet_start + 11]);
                  short int gz = ((0xff & (char)input[data_packet_start + 14]) << 8) |
                                 (0xff & (char)input[data_packet_start + 13]);

                  temp = (short int*)&gx;
                  float gxf = (*temp) * 300.0 / 32768;
                  temp = (short int*)&gy;
                  float gyf = (*temp) * 300.0 / 32768;
                  temp = (short int*)&gz;
                  float gzf = (*temp) * 300.0 / 32768;

                  // calculate rotational velocities in rad/s
                  // without the last factor the velocities were too small
                  // http://www.i2cdevlib.com/forums/topic/106-get-angular-velocity-from-mpu-6050/
                  // FIFO frequency 100 Hz -> factor 10 ?
                  // seems 25 is the right factor
                  // TODO: check / test if rotational velocities are correct
                  // double gxf = gx * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
                  // double gyf = gy * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
                  // double gzf = gz * (4000.0/65536.0) * (M_PI/180.0) * 25.0;

                  // get acelerometer values
                  // int ax = (((0xff &(char)input[data_packet_start + 18]) << 24) |((0xff
                  // &(char)input[data_packet_start + 17]) << 16) |((0xff &(char)input[data_packet_start + 16]) << 8) |
                  // 0xff &(char)input[data_packet_start + 15]); int ay = (((0xff &(char)input[data_packet_start + 22])
                  // << 24) |((0xff &(char)input[data_packet_start + 21]) << 16) |((0xff &(char)input[data_packet_start +
                  // 20]) << 8) | 0xff &(char)input[data_packet_start + 19]); int az = (((0xff
                  // &(char)input[data_packet_start + 26]) << 24) |((0xff &(char)input[data_packet_start + 25]) << 16)
                  // |((0xff &(char)input[data_packet_start + 24]) << 8) | 0xff &(char)input[data_packet_start + 23]);
                  short int ax = ((0xff & (char)input[data_packet_start + 16]) << 8) |
                                 (0xff & (char)input[data_packet_start + 15]);
                  short int ay = ((0xff & (char)input[data_packet_start + 18]) << 8) |
                                 (0xff & (char)input[data_packet_start + 17]);
                  short int az = ((0xff & (char)input[data_packet_start + 20]) << 8) |
                                 (0xff & (char)input[data_packet_start + 19]);

                  // calculate accelerations in m/s²
                  // double axf = ax * (8.0 / 65536.0) * 9.81;
                  // double ayf = ay * (8.0 / 65536.0) * 9.81;
                  // double azf = az * (8.0 / 65536.0) * 9.81;

                  temp = (short int*)&ax;
                  float axf = *temp * 12.0 / 32768;
                  temp = (short int*)&ay;
                  float ayf = *temp * 12.0 / 32768;
                  temp = (short int*)&az;
                  float azf = *temp * 12.0 / 32768;

                  // get gps values

                  int latitude = (((0xff & (char)input[data_packet_start + 24]) << 24) |
                                  ((0xff & (char)input[data_packet_start + 23]) << 16) |
                                  ((0xff & (char)input[data_packet_start + 22]) << 8) |
                                  0xff & (char)input[data_packet_start + 21]);
                  int longitude = (((0xff & (char)input[data_packet_start + 28]) << 24) |
                                   ((0xff & (char)input[data_packet_start + 27]) << 16) |
                                   ((0xff & (char)input[data_packet_start + 26]) << 8) |
                                   0xff & (char)input[data_packet_start + 25]);
                  int altitude = (((0xff & (char)input[data_packet_start + 32]) << 24) |
                                  ((0xff & (char)input[data_packet_start + 31]) << 16) |
                                  ((0xff & (char)input[data_packet_start + 30]) << 8) |
                                  0xff & (char)input[data_packet_start + 29]);

                  int* tempA = (int*)&latitude;
                  double latitudef = *tempA * 1e-7L;
                  tempA = (int*)&longitude;
                  double longitudef = *tempA * 1e-7L;
                  tempA = (int*)&altitude;
                  double altitudef = *tempA * 1e-3L;

                  // get temperature
                  // int16_t temperature = (((0xff &(char)input[data_packet_start + 22]) << 8) | 0xff
                  // &(char)input[data_packet_start + 23]); double temperature_in_C = (temperature / 340.0 ) + 36.53;
                  // ROS_DEBUG_STREAM("Temperature [in C] " << temperature_in_C);

                  // uint8_t received_message_number = input[data_packet_start + 25];
                  // ROS_DEBUG("received message number: %i", received_message_number);

                  // if (received_message) // can only check for continuous numbers if already received at least one
                  // packet
                  //{
                  // uint8_t message_distance = received_message_number - last_received_message_number;
                  // if ( message_distance > 1 )
                  //{
                  //  ROS_WARN_STREAM("Missed " << message_distance - 1 << " MPU6050 data packets from arduino.");
                  //}
                  //}
                  // else
                  //{
                  received_message = true;
                  //}
                  // last_received_message_number = received_message_number;

                  // calculate measurement time
                  ros::Time measurement_time = ros::Time::now() + ros::Duration(time_offset_in_seconds);

                  // publish imu message
                  imu.header.stamp = measurement_time;
                  imu.header.frame_id = frame_id;

                  quaternionTFToMsg(differential_rotation, imu.orientation);

                  imu.angular_velocity.x = gxf;
                  imu.angular_velocity.y = gyf;
                  imu.angular_velocity.z = gzf;

                  imu.linear_acceleration.x = axf;
                  imu.linear_acceleration.y = ayf;
                  imu.linear_acceleration.z = azf;

                  imu_pub.publish(imu);

                  // publish temperature message
                  temperature_msg.header.stamp = measurement_time;
                  temperature_msg.header.frame_id = frame_id;
                  // temperature_msg.temperature = temperature_in_C;

                  imu_temperature_pub.publish(temperature_msg);

                  // publish gps message
                  gps_msg.header.stamp = measurement_time;
                  gps_msg.header.frame_id = frame_id;
                  if (abs(gps_msg.altitude - altitudef) > 100)
                  {
                    input.erase(0, data_packet_start + 1);
                    continue;
                  }
                  else
                  {
                    gps_msg.latitude = latitudef;
                    gps_msg.longitude = longitudef;
                    gps_msg.altitude = altitudef;
                    imu_gps_pub.publish(gps_msg);
                  }
                  // publish triggertime message
                  if (1)
                  {  // triggerCounter != lastTriggerCounter
                    // ros::Time time_ref(0, 0);
                    trigger_time_msg.header.frame_id = frame_id;
                    trigger_time_msg.header.stamp = measurement_time;
                    // trigger_time_msg.time_ref = time_ref;
                    trigger_time_pub.publish(trigger_time_msg);
                    // lastTriggerCounter = triggerCounter;
                  }

                  // publish tf transform
                  if (broadcast_tf)
                  {
                    transform.setRotation(differential_rotation);
                    tf_br.sendTransform(
                        tf::StampedTransform(transform, measurement_time, tf_parent_frame_id, tf_frame_id));
                  }
                  // outfile<<pack<<std::endl; // write package into output file
                }
                input.erase(0,
                            data_packet_start + Length);  // delete everything up to and including the processed packet
              }
              else
              {
                input.erase(0, data_packet_start + 1);  // delete up to false data_packet_start character so it is not
                                                        // found again
              }
            }
            else
            {
              // no start character found in input, so delete everything
              input.clear();
            }
          }
        }
      }

      else
      {
        // try and open the serial port
        try
        {
          ser.setPort(port);
          ser.setBaudrate(230400);  // 115200  230400
          serial::Timeout to = serial::Timeout::simpleTimeout(100);
          ser.setTimeout(to);
          ser.open();
        }
        catch (serial::IOException& e)
        {
          ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
          ros::Duration(5).sleep();
        }

        if (ser.isOpen())
        {
          ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
        }
      }
    }
    catch (serial::IOException& e)
    {
      ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
      ser.close();
    }
    ros::spinOnce();
    r.sleep();
  }
  // outfile.close();
}
