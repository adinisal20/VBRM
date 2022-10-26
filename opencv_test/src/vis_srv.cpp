// header for ROS core functionalities
#include "rclcpp/rclcpp.hpp"

// including image message type to be able to receive and publish it
#include "sensor_msgs/msg/image.hpp"

// headers regarding the connection between opencv and ROS
#include <image_transport/image_transport.hpp>
#include "cv_bridge/cv_bridge.h"

// OpenCV core functions and various image processing algorithms
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <memory>
#include <future>
#include <fstream>
#include <typeinfo>
#include <bits/stdc++.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <chrono>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp" 
#include "sensor_msgs/msg/joint_state.hpp"
using namespace std::chrono_literals;
std::ofstream Plots;

using std::placeholders::_1;
    bool velocityControllerFlag;
    double q1;
    double q2;
    float L1 = 1;
    float L2 = 1;
// Defining a class that will be utilize in the "main" function
class ImageSubscriber : public rclcpp::Node
{

	// Declaring pointer to the publisher and subscriber the publish and receive images.
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub; // Position controller
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub; // Velocity controller
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;    // Joint_state topic subscriber

  	rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr client;
  public:
	// Constructor of the class. The class is derived from the parent class "Node" of rclcpp and
	// the node is called "image_processor", and each time it receive a data, it will call the callbackImage() function
    	ImageSubscriber() : Node("image_processor")
	{
		// Defining the subscriber: it will receive "Image" type data from the topic /camera1/image_raw 
      		subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/camera1/image_raw", 10, std::bind(&ImageSubscriber::callbackImage, this, _1));
			//Joint state publisher will receive the joint state parameters like velocity, position of joints etc. of the robot from the /joint_states topic
			joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,std::bind(&ImageSubscriber::jointCallback, this, _1));
			//Switch_controller client switches between position and velocity controller when called
			client = this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
		//Defining the publisher: it will publish "Image" type data to the "output_image" topic
            publisher_ = this->create_publisher<sensor_msgs::msg::Image>("output_image", 10);
			//Publishing image to this topic to send the position values
			position_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands",10);
			//Publishing to this topic to send the final joint velocity values calculated.
			velocity_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_velocity_controller/commands",10);
			setup_pos();
    }
		//Declaring a destructor
		~ImageSubscriber(){
            
        }
	
  public:
	//This function will get the position from /joint_states topic and print it in CLI
	void jointCallback(const sensor_msgs::msg::JointState data){
		q1 = data.position[0];
		q2 = data.position[1];
		std::cout<<"q1:"<<q1<<"\t"<<"q2:"<<q2<<std::endl;
	}

	void callbackImage(const sensor_msgs::msg::Image::SharedPtr msg) 
	{    
		//converting the ROS message type data to opencv type
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg,"bgr8" );
		
		cv::Mat threshold_img = cv_ptr->image;
		cv::Mat hsv_img;
		if (velocityControllerFlag == true)
		{
		cv::cvtColor(cv_ptr->image, hsv_img, cv::COLOR_BGR2HSV);
		cv::Mat green_;
		cv::inRange(hsv_img, cv::Scalar(30, 60, 80), cv::Scalar(80, 255, 255), green_);
		
		cv::Mat red_;
		cv::inRange(hsv_img, cv::Scalar(0, 60, 80), cv::Scalar(10, 255, 255), red_);
		
		cv::Mat magenta_;
		cv::inRange(hsv_img, cv::Scalar(125, 60, 80), cv::Scalar(160, 255, 255), magenta_);
		
		cv::Mat blue_;
		cv::inRange(hsv_img, cv::Scalar(90, 60, 80), cv::Scalar(130, 255, 255), blue_);

		cv::Point2d blue_center = cal_center(blue_);
		cv::Point2d red_center = cal_center(red_);
		cv::Point2d green_center = cal_center(green_);
		cv::Point2d magenta_center = cal_center(magenta_);

		RCLCPP_INFO(this->get_logger(), "blue center in pixel frame:(%f, %f)",blue_center.x,blue_center.y);
		RCLCPP_INFO(this->get_logger(), "red center in pixel frame:(%f, %f)",red_center.x,red_center.y);
		RCLCPP_INFO(this->get_logger(), "green center in pixel frame:(%f, %f)",green_center.x,green_center.y);
		RCLCPP_INFO(this->get_logger(), "magenta center in pixel frame:(%f, %f)",magenta_center.x,magenta_center.y);

		cv::circle(threshold_img, blue_center, 3, cv::Scalar(0,0,0), cv::FILLED);
		cv::circle(threshold_img, green_center, 3, cv::Scalar(0,0,0), cv::FILLED);
		cv::circle(threshold_img, red_center, 3, cv::Scalar(0,0,0), cv::FILLED);
		cv::circle(threshold_img, magenta_center, 3, cv::Scalar(0,0,0), cv::FILLED);
		//Uptill this line, we have thresholded the colour, averaged the circles ot get the centres(averaging done below) and then used the centres.
		double s_des[8][1] = {427,287,427,371,510,287,511,371};
        double s_[8][1] = {red_center.x,red_center.y,blue_center.x,blue_center.y,magenta_center.x,magenta_center.y,green_center.x,green_center.y};

		//Calculating error
		double error[8][1];
		//For all matrix operations, for loops have been used
		//Matrix subtraction
		for (int a = 0; a<8; a++){
			for (int b = 0; b<1; b++){
				error[a][b] = s_[a][b] - s_des[a][b];
			}
		}

		//Le jacobian to get the Velocity if camera
        double Le_plus[2][8] = {{-0.25,0,-0.25,0,-0.25,0,-0.25,0},{0,-0.25,0,-0.25,0,-0.25,0,-0.25}}; //Using MATLAB
        double Vc[2][1];
		double l = 0.0000005; //Lambda or gain
        double lambda[2][2]= {{-l,0},{0,-l}};
		//Matric multiplication
		for (int i = 0; i < 2; i++) {
        	for (int j = 0; j < 1; j++) {
            	Vc[i][j] = 0;
 
            	for (int k = 0; k < 8; k++) {
                	Vc[i][j] += Le_plus[i][k] * error[k][j];
            }
			}
		}
		double V_c[2][1];
		for (int i = 0; i < 2; i++) {
        	for (int j = 0; j < 1; j++) {
            	V_c[i][j] = 0;
 
            	for (int k = 0; k < 2; k++) {
                	V_c[i][j] += lambda[i][k] * Vc[k][j];
            }
			}	
		}
		std::cout<<"Velocity:"<<V_c[0][0]<<"\t"<<V_c[1][0]<<std::endl;// Printing out the velocity value to edcode the calculations.
		//Hardcoding the robot Jacobian to get the joint velocity and using the joint values from jiont_states topic.
		double robot_jacobian[6][2] = {{(-L2*sin(q1)*cos(q2))-(L1*cos(q1)*sin(q2)-L1*sin(q1)), (-L2*sin(q1)*cos(q2))-(L1*cos(q1)*sin(q2))},{(L2*cos(q1)*cos(q2))-(L2*sin(q1)*sin(q2)+L1*sin(q1)), (L2*cos(q1)*sin(q2))-(L2*sin(q1)*sin(q2))},{0, 0},{0,0},{0,0},{1,1}}; 

		//Inverting the robot jacobian using the cv funcitons
		cv::Mat A(6,2,CV_64F);
		std::memcpy(A.data, robot_jacobian, 6*2*sizeof(double));
		double inv_rob_jacob[2][6];
		cv::Mat B(2,6,CV_64F);
		std::memcpy(B.data, inv_rob_jacob, 2*6*sizeof(double));
		cv::invert(cv::_InputArray(A), cv::_OutputArray(B), cv::DECOMP_SVD);
		double inverted_final[2][2] = {{inv_rob_jacob[0][0], inv_rob_jacob[0][1]},{inv_rob_jacob[1][0],inv_rob_jacob[1][1]}};
		double q_dot[2][1];

		for (int i = 0; i < 2; i++) {
        	for (int j = 0; j < 1; j++) {
            	q_dot[i][j] = 0;
 
            	for (int k = 0; k < 2; k++) {
                	q_dot[i][j] += inverted_final[i][k] * V_c[k][j];
            }
			}
		}
		std::cout<<"q_dot:"<<q_dot[0][0]<<"\t"<<q_dot[1][0]<<std::endl; //Printing joint velicity in the CLI
		//Publishing the joint velocity for the robot to move.
		std_msgs::msg::Float64MultiArray q_vel;
		q_vel.data.push_back(q_dot[0][0]);
		q_vel.data.push_back(q_dot[1][0]);
		velocity_pub->publish(q_vel);
		//Creating a CSV file which can be acessed using matplotlib and hence a graph can be drawn.
		Plots.open ("plots.csv",std::ios::app);
		Plots <<s_[0][0]<< "," <<s_[1][0]<< "," <<s_[2][0]<< "," <<s_[3][0]<< "," <<s_[4][0]<< "," <<s_[5][0]<< "," <<s_[6][0]<< "," <<s_[7][0]<<"\n";
    		Plots.close() ;
		}

	
		sensor_msgs::msg::Image::SharedPtr msg_to_send = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8",threshold_img).toImageMsg();
		// converting opencv data type to ROS message type
   		

		// publishing the image.
   		publisher_->publish(*msg_to_send);
	}
	//This is the averaging and finding centres of circles part mentioned above
	cv::Point2d cal_center(cv::Mat mask)
    {
		double xAvg = 0, yAvg = 0, count = 0;
		for (int i = 0; i < mask.rows; i++)
		{
			for (int j = 0; j < mask.cols; j++)
			{
				if((int)mask.at<uchar>(i,j) == 255)
				{
					xAvg += j;
					yAvg += i;
					count += 1;
				}
			}
			
		}
		cv::Point2d center = cv::Point2d(double(xAvg/count), double(yAvg/count));
		return center;	
	
	}
	void publish(float J1, float J2)
	{
		std_msgs::msg::Float64MultiArray val;
		val.data = {J1, J2};
		position_pub->publish(val);
	}
	//Moving the robot to a certain angle and then switching to velocity controller
	void setup_pos()
	{
		sleep(2);
		publish(0.25,0.25);
		sleep(5);
		switchController();
	}
		
    
void switchController()
	{
	auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
	request->activate_controllers = {"forward_velocity_controller"};
	request->deactivate_controllers = {"forward_position_controller"};
	while (!client->wait_for_service(1s)) {
		if (!rclcpp::ok()) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
	}
	auto result = client->async_send_request(request);
	// Wait for the result.
	if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
	{ 
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "OK");
		velocityControllerFlag = true;  
	} else {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
	}
	}
};

int main(int argc, char * argv[])
{

	//initialize ROS
	rclcpp::init(argc, argv);
	//create the 
	rclcpp::spin(std::make_shared<ImageSubscriber>());
	rclcpp::shutdown();
  return 0;
}
