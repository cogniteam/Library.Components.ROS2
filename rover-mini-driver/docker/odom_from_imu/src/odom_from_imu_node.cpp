#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include  <serial/serial.h>

#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>

using namespace std::chrono_literals;


using namespace std;
 

std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_pub;

double current_time;
double last_time;

double x = 0.0;
double y = 0.0;  

double initialYawOffset_ = 0.0;
bool setInitialOffset = false;


bool calcedBias_ = false;

bool odomRecived_ = false;

//from IMU
double  currnetImuHeadingRad_ = 0.0;

// from old odom
double currentAngularVel_ = 0.0;
float  currentLinearVelX_ = 0.0;
double currentWheelsHeading_ = 0.0;


double vx_ = 0.0;
double vy_ = 0.0;
double vz_ = 0.0;

const int HZ_ = 15;

bool ImuInit_ = false;
int numOfImuMsg = 0;
int ImuThresh = 10;
float yawBias_ = 0.0;

bool exit_ = false;

unsigned long long int  iteration = 0;

serial::Serial serial_(std::string(), 115200, serial::Timeout::simpleTimeout(50), 
  serial::eightbits, serial::parity_none,
            serial::stopbits_one, serial::flowcontrol_none);


static void mySigintHandler(int sig, void *ptr)
{
  cerr << " user pressed CTRL+C " << endl;
  exit_ = true;
}

std::vector<double> splitSensorMeasurement(std::string const& input)
{
  std::istringstream istream(input);

  std::vector<double> output;
  double current_val;
  while (istream >> current_val)
  {
    output.push_back(current_val);
    char comma_eater;
    if (istream >> comma_eater && comma_eater != ',')
    {
      // we expected either end of string or a comma
      output.clear();
      break;
    }
  }

  return output;
}


float calcYawBias(){

  float yawBias = 0.0;
  cerr<<" calc bias "<<endl;
  
  rclcpp::Rate loop_rate(HZ_);

  auto last_time = rclcpp::Clock{}.now().seconds();


  double workingHeading;

  //calcualte imu yaw bias
  auto start = rclcpp::Clock{}.now().seconds();
	
  int countR = 0;
  
  float prevYawRad_ = 0.0;
  while(rclcpp::ok()) {

    if( exit_){
      return 0.0;
    }

    if( !ImuInit_){

      continue;
    }

    if(prevYawRad_ == 0.0 ){
      prevYawRad_ = currnetImuHeadingRad_;
      cerr<<"222222222222222 "<<endl;

      continue;
    }

    countR++;

    float diff = currnetImuHeadingRad_ - prevYawRad_;


    yawBias += (diff);

    prevYawRad_ = currnetImuHeadingRad_;

    auto end = rclcpp::Clock{}.now().seconds();

    auto duration = (end - start);
    if(duration > 10){
      break;
    }

    loop_rate.sleep();
  }

  yawBias = 0.00025;//yawBias / float(countR);


  cerr<<" yawBias "<<yawBias <<" countR "<<countR<<endl; 

  return yawBias;
}


string readline()
{ 
  size_t size = 65536;
  std::string eol = "\n";
  string line;
  auto res = serial_.readline(line, size, eol);
 
  return line;
}

void imu_callback()
{ 

  string line = readline();
  //cerr<<"line: "<<line<<endl;

  std::vector<double> measurement;
  measurement = splitSensorMeasurement(line);

  tf2::Quaternion q;
  q.setX(measurement[11]);
  q.setY(measurement[12]);
  q.setZ(measurement[13]);
  q.setW(measurement[10]);

  tf2::Matrix3x3 m( q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, currnetImuHeadingRad_);

  ImuInit_ = true;

  if( !setInitialOffset){

    initialYawOffset_ = currnetImuHeadingRad_;
    setInitialOffset = true;
  }
  
}

void tf_publisher_timer_callback()
{    
    if( !calcedBias_){
      yawBias_ = calcYawBias();
      calcedBias_ = true;
      return;  
    }
   
    if( !odomRecived_){

      return;
    }
    if( exit_){
      exit(1);
    }

    
    // cerr<<"iteration "<<iteration<<" odom "<<currentLinearVelX_<<" from imu "<<currnetImuHeadingRad_<<" yawBias_ "<<yawBias_<<endl;

    current_time = rclcpp::Clock{}.now().seconds();


    float imuHeadingRadWithBias = currnetImuHeadingRad_ - ( iteration * yawBias_);// -initialYawOffset_;

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time);
    double delta_x = (currentLinearVelX_ * cos(imuHeadingRadWithBias) ) * dt;
    double delta_y = (currentLinearVelX_ * sin(imuHeadingRadWithBias) ) * dt;

    x += delta_x;
    y += delta_y;

    tf2::Quaternion q;
    q.setRPY(0, 0, imuHeadingRadWithBias);
    geometry_msgs::msg::Quaternion qMsg =  tf2::toMsg(q);
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = rclcpp::Clock().now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    tf2::convert(qMsg, odom_trans.transform.rotation);
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;

    //   //send the transform
    odom_tf_pub->sendTransform(odom_trans); 

    last_time = current_time;

    iteration++;
}


void rawOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  currentLinearVelX_ = msg->twist.twist.linear.x;

  currentAngularVel_ = msg->twist.twist.angular.z;

  vx_ = msg->twist.twist.linear.x;
  vy_ = msg->twist.twist.linear.y;
  vz_ = msg->twist.twist.angular.z;

  cerr<<" currentLinearVelX_ "<<currentLinearVelX_<<endl;

  odomRecived_ = true;
  if( exit_){
   exit(1);
  } 

}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("odom_from_imu_node");


  signal(SIGINT, (void (*)(int))mySigintHandler);

  try
  {
    serial_.setPort("/dev/ttyACM0");
    serial_.open();
  }
  catch (std::exception const& e)
  { 
    serial_.close();

    cerr<<e.what()<<endl;
    return -1;
  }

  if (!serial_.isOpen()){
    
    cerr<<" failed to open /dev/ttyACM0 "<<endl;
    return -1;
  }

   auto imu_timer = rclcpp::create_timer(
    node,
    node->get_clock(),
    rclcpp::Duration(1/HZ_, 0),
    std::bind(imu_callback)); 


  
  odom_tf_pub = std::make_unique<tf2_ros::TransformBroadcaster>(*node);

  auto odomSub_ =
    node->create_subscription<nav_msgs::msg::Odometry>("/odom_raw", 10, rawOdomCallback);

  current_time = rclcpp::Clock{}.now().seconds();
  last_time = rclcpp::Clock{}.now().seconds();


  auto tf_publisher_timer = rclcpp::create_timer(
    node,
    node->get_clock(),
    rclcpp::Duration(1/HZ_, 0),
    std::bind(tf_publisher_timer_callback));

  

  if( exit_){
    return 0;
  }  

  // thread t1(imuTread);
  // t1.detach();   
  
   
 

  rclcpp::spin(node);
  rclcpp::shutdown();
 
  odomSub_ = nullptr;
  node = nullptr;
  serial_.close();

  return 0;
}

