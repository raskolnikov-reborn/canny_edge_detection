#include "ros/ros.h"
#include "ros/package.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


// persistent variables: Should be class members in an Object oriented implementation
int lower_canny_threshold;
int upper_canny_threshold;
image_transport::Publisher pub;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	// cvMat from sensor_msgs::Image
	cv::Mat received_image = cv_bridge::toCvShare(msg, "bgr8")->image;

	// apply canny edge detection
	cv::Mat edge_image;
	cv::Canny(received_image, edge_image, lower_canny_threshold, upper_canny_threshold);

	// convert to sensor_msgs type output
	sensor_msgs::ImagePtr output = cv_bridge::CvImage(std_msgs::Header(), "mono8", edge_image).toImageMsg();
	output->header.stamp = ros::Time::now();
	output->header.frame_id = "camera";

	//publish the output
	pub.publish(output);

}


int main(int argc, char** argv)
{

	// Setup ros
	ros::init(argc,argv, "canny_edge_my_face_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	// local variables
	int target_frequency;
	std::string input_topic_param;
	std::string output_topic_param;

	// setup local and persistent variables through the rosparam server values should be private to this node
	private_nh.param<std::string>("input_topic", input_topic_param, "/cv_camera/image_raw");
	private_nh.param<std::string>("output_topic",output_topic_param, "image_processed");
	private_nh.param<int>("loop_frequency", target_frequency, 30);
	private_nh.param<int>("lower_canny_threshold", lower_canny_threshold, 20);
	private_nh.param<int>("upper_canny_threshold", upper_canny_threshold, 200);


	// setup up iteration frequency
	ros::Rate frequency(target_frequency);

	// setup an image transport on the private node handle
	image_transport::ImageTransport it(private_nh);
	
	// create a local subscriber
	image_transport::Subscriber sub;
	sub = it.subscribe(input_topic_param, 1, imageCallback);

	//setup a persistent publisher
	pub = it.advertise(output_topic_param, 1);

	// spin ros
	while (ros::ok())
	{
		ros::spinOnce();
		frequency.sleep();
	}
}
