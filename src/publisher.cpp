#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <fcntl.h> 
#include <string.h>
#include <termios.h>

using namespace std;

int main(int argc, char** argv)
{
    int serial_port_num;
    
    ros::init(argc, argv, "radar_publisher"); 
    ros::NodeHandle nh("~");

    ROS_INFO("\033[1;32m---->\033[0m Radar Parser Started.");

    std::string serialPort;
    nh.param<std::string>("serialPort", serialPort, "/dev/ttyACM0");

    ros::Publisher pubRange = nh.advertise<std_msgs::Float32>("radar_range",10);
    ros::Publisher pubSpeed = nh.advertise<std_msgs::Float32>("radar_speed",10);

    serial_port_num = open(serialPort.c_str(), O_RDWR | O_NOCTTY | O_SYNC);   

    if(serial_port_num == -1)
    {
        ROS_ERROR("Error opening radar port!");
        return -1;
    }
    else
        ROS_DEBUG("Radar port opened successfully!");

    struct termios SerialPortSettings;
    cfsetospeed (&SerialPortSettings, B9600);
    cfsetispeed (&SerialPortSettings, B9600);

    ros::Rate r(100);
    while (ros::ok()){

        tcflush(serial_port_num, TCIFLUSH);

        char read_buffer[125];
        int bytes_read = read(serial_port_num,&read_buffer,125);

        if (bytes_read <= 4)
            continue;

        float range_value = 0;
        float speed_value = 0;

        if (read_buffer[1] == 'm' && read_buffer[2] == 'p' && read_buffer[3] == 's')
        {
            int i = 6;
            string str_value;
            while (i < bytes_read){
              str_value += read_buffer[i];
              ++i;
            }
            std_msgs::Float32 speed_message;
            speed_message.data = std::stof(str_value);
            pubSpeed.publish(speed_message);

        }else if (read_buffer[0] == '\"' && read_buffer[1] == 'm' && read_buffer[2] == '\"')
        {
            int i = 4;
            string str_value;
            while (i < bytes_read){
              str_value += read_buffer[i];
              ++i;
            }
            std_msgs::Float32 range_message;
            range_message.data = std::stof(str_value);
            pubRange.publish(range_message);
        }
          
        r.sleep();
    }

    close(serial_port_num);

    return 0;
}