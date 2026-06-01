#include <stdio.h> /* Standard input/output definitions */
#include <string.h> /* String function definitions */
#include <unistd.h> /* UNIX standard function definitions */
#include <fcntl.h> /* File control definitions */
#include <errno.h> /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <eigen3/Eigen/Geometry> 

#include "hi226_imu/packet.h"
#include "hi226_imu/imu_data_decode.h"

using namespace std;

boost::asio::serial_port* serial_port = 0;
static ros::Publisher pub, pub_mag, pub_gps;
static sensor_msgs::Imu msg;
static sensor_msgs::MagneticField msg_mag;
static sensor_msgs::NavSatFix msg_gps;
static std::string name, frame_id;
static int fd_ = -1;

int uart_set(int fd, int baude, int c_flow, int bits, char parity, int stop)
{
    struct termios options;
 
    if(tcgetattr(fd, &options) < 0)
    {
        perror("tcgetattr error");
        return -1;
    }

    cfsetispeed(&options,B115200);
    cfsetospeed(&options,B115200);

    options.c_cflag |= CLOCAL;
    options.c_cflag |= CREAD;

    switch(c_flow)
    {
        case 0:
            options.c_cflag &= ~CRTSCTS;
            break;
        case 1:
            options.c_cflag |= CRTSCTS;
            break;
        case 2:
            options.c_cflag |= IXON|IXOFF|IXANY;
            break;
        default:
            fprintf(stderr,"Unkown c_flow!\n");
            return -1;
    }

    switch(bits)
    {
        case 5:
            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS8;
            break;
        default:
            fprintf(stderr,"Unkown bits!\n");
            return -1;
    }

    switch(parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~INPCK;
            break;

        case 's':
        case 'S':
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;

        case 'o':
        case 'O':
            options.c_cflag |= PARENB;
            options.c_cflag |= PARODD;
            options.c_cflag |= INPCK;
            options.c_cflag |= ISTRIP;
            break;

        case 'e':
        case 'E':
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_cflag |= INPCK;
            options.c_cflag |= ISTRIP;
            break;
        default:
            fprintf(stderr,"Unkown parity!\n");
            return -1;
    }

    switch(stop)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            fprintf(stderr,"Unkown stop!\n");
            return -1;
    }

    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 1;

    tcflush(fd,TCIFLUSH);

    if(tcsetattr(fd,TCSANOW,&options) < 0)
    {
        perror("tcsetattr failed");
        return -1;
    }

    return 0;

}

int main(int argc, char** argv)
{
       
    uint8_t ID = 0;
    int16_t Acc[3] = {0};
    int16_t Gyo[3] = {0};
    int16_t Mag[3] = {0};
    float Eular[3] = {0};
    float Quat[4]  = {0};
    int32_t Pressure = 0;

    int i;
    uint8_t buf[41];

    ros::init(argc, argv, "hi226_imu");
    ros::NodeHandle n("~"); 
    name = ros::this_node::getName();

    std::string port;
    if (n.hasParam("port"))
        n.getParam("port", port);
    else
    {
        ROS_ERROR("%s: must provide a port", name.c_str());
        return -1;
    }

    std::string model;
    if (n.hasParam("model"))
        n.getParam("model", model);
    else
    {
        ROS_ERROR("%s: must provide a model name", name.c_str());
    return -1;
    }
    ROS_WARN("Model set to %s", model.c_str());

    int baud;
    if (n.hasParam("baud"))
        n.getParam("baud", baud);
    else
    {
        ROS_ERROR("%s: must provide a baudrate", name.c_str());
        return -1;
    }  
    ROS_WARN("Baudrate set to %d", baud);

    n.param("frame_id", frame_id, string("IMU_link"));
    double delay;
    n.param("delay", delay, 0.0);

    boost::asio::io_service io_service;
    serial_port = new boost::asio::serial_port(io_service);
    try
    {
        serial_port->open(port);
    }
    catch (boost::system::system_error &error)
    {
        ROS_ERROR("%s: Failed to open port %s with error %s",
            name.c_str(), port.c_str(), error.what());
        return -1;
    }

    if (!serial_port->is_open())
    {
          ROS_ERROR("%s: failed to open serial port %s",
                name.c_str(), port.c_str());
        return -1;
    }

    typedef boost::asio::serial_port_base sb;

    sb::baud_rate baud_option(baud);
    sb::flow_control flow_control(sb::flow_control::none);
    sb::parity parity(sb::parity::none);
    sb::stop_bits stop_bits(sb::stop_bits::one);

    serial_port->set_option(baud_option);
    serial_port->set_option(flow_control);
    serial_port->set_option(parity);
    serial_port->set_option(stop_bits);

    const char *path = port.c_str();
    fd_ = open(path, O_RDWR);
    if(fd_ < 0)
    {    
        ROS_ERROR("Port Error!: %s", path);
        return -1;
    }

    pub = n.advertise<sensor_msgs::Imu>("/imu_data", 1);
    //pub_mag = n.advertise<sensor_msgs::MagneticField>("mag", 1);
    //pub_gps = n.advertise<sensor_msgs::NavSatFix>("gps", 1);      
    
    imu_data_decode_init();
    

    ROS_WARN("Streaming Data...");
    while (n.ok())
    {
        //ssize_t n = read(fd_, buf, sizeof(buf));
        //for(i=0; i<n; i++)
        {
            //Packet_Decode(buf[i]);
        }
        ssize_t n  = read(fd_, buf, sizeof(buf));
        for(i=0; i<n; i++)
        {
            Packet_Decode(buf[i]);
        } 
        get_raw_acc(Acc);
        get_raw_gyo(Gyo);
        get_raw_mag(Mag);
        get_eular(Eular);
        get_id(&ID);

        Eigen::Vector3d ea0(Eular[1] * M_PI / 180.0,
                Eular[0] * M_PI / 180.0,
                Eular[2] * M_PI / 180.0);             
        Eigen::Matrix3d R;  
        R = Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitX())  
            * Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())  
            * Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitZ());  
        Eigen::Quaterniond q;
        q = R; 

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_id;
        msg.orientation.w = (double)q.w();
        msg.orientation.x = (double)q.x();
        msg.orientation.y = (double)q.y();
        msg.orientation.z = (double)q.z(); 

        msg.linear_acceleration.x = Acc[0] * 1e-3 * 9.81;
        msg.linear_acceleration.y = Acc[1] * 1e-3 * 9.81;
        msg.linear_acceleration.z = Acc[2] * 1e-3 * 9.81;
 
        pub.publish(msg);
        //pub_mag.publish(msg_mag);
        //pub_gps.publish(msg_gps);
 
        //printf("Acc:%d %d %d\r\n",Acc[0], Acc[1], Acc[2]);
        //printf("Gyo:%d %d %d\r\n",Gyo[0], Gyo[1], Gyo[2]);
        //printf("Mag:%d %d %d\r\n",Mag[0], Mag[1], Mag[2]);
        //printf("Eular(P R Y):%0.2f %0.2f %0.2f\r\n",Eular[0], Eular[1], Eular[2]);
        //printf("quat(W X Y Z):%0.3f %0.3f %0.3f %0.3f\r\n",Quat[0], Quat[1], Quat[2], Quat[3]);
            
        //usleep(20*1000);

        //ROS_INFO("linear_acceleration.x = %lf",msg.linear_acceleration.x);
        //ROS_INFO("linear_acceleration.y = %lf",msg.linear_acceleration.y);
        //ROS_INFO("linear_acceleration.z = %lf",msg.linear_acceleration.z);

        //ROS_INFO("orientation.w = %lf",msg.orientation.w);
        //ROS_INFO("orientation.x = %lf",msg.orientation.x);
        //ROS_INFO("orientation.y = %lf",msg.orientation.y);
        //ROS_INFO("orientation.z = %lf",msg.orientation.z);

    }
    ROS_WARN("Wait 0.1s"); 
    ros::Duration(0.1).sleep();   
    ::close(fd_);
    return 0;
}



