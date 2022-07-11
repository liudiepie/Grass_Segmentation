#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
// #include <depthai_bridge/ImageConverter.hpp>
#include <cv_bridge/cv_bridge.h>


using namespace std;
using namespace cv;
using std::placeholders::_1;
// Inludes common necessary includes for development using depthai library
//rclcpp::Node::SharedPtr node = nullptr;
// dai::rosBridge::ImageConverter inputConverter(true);

class SegmentationSub : public rclcpp::Node{
    public:
	// Contructor
	// The name of the node is rgb_subscriber_node
	SegmentationSub()
	: Node("rgb_subscriber_node"){
	    subscription_1 = this->create_subscription<sensor_msgs::msg::Image>("color/video/image", 10, bind(&SegmentationSub::rgbCallback, this, _1));
	    subscription_2 = this->create_subscription<sensor_msgs::msg::Image>("stereo/depth", 10, bind(&SegmentationSub::depthCallback, this, _1));
	}
    private:

	void rgbCallback(const sensor_msgs::msg::Image::SharedPtr rgbImageMsg){ 
	    //cv::Mat rgbImage = inputConverter.rosMsgtoCvMat(*rgbImageMsg); 
	    ////cv::imshow("video", rgbImage); 
    	    try{ 
        	Mat img_proc, img_dilation; 
		Mat rgbImage = cv_bridge::toCvShare(rgbImageMsg, "bgr8")->image; 
		//convert BGR to Lab
		cvtColor(rgbImage, img_proc, COLOR_BGR2Lab);

		//extract the L channel
		vector<Mat> lab_planes(3);
		split(img_proc, lab_planes);

		//apply the CLAHE algorithm to the L channel
		Ptr<CLAHE> clahe = createCLAHE();
		clahe->setClipLimit(4);
		Mat dst;
		clahe->apply(lab_planes[0], dst);

		//merge the color planes back into an lab image
		dst.copyTo(lab_planes[0]);
		merge(lab_planes, img_proc);

		//convert Lab to BGR
		cvtColor(img_proc, img_proc, COLOR_Lab2BGR);
	
		//convert BGR to HSV
		cvtColor(img_proc, img_proc, COLOR_BGR2HSV);

		//detect the object based on HSV range
		//H: 34 - 70, S: 60 - 255, V: 60 - 255
		inRange(img_proc, Scalar(34, 60, 60), Scalar(70, 255, 255), img_proc);
	
		//dilation Size: (2n+1), Point: (n), n=4
		Mat element = getStructuringElement(MORPH_RECT, Size(9,9), Point(4,4));
		dilate(img_proc, img_dilation, element);

		//resize the images
		resize(rgbImage, rgbImage, Size(), 0.75, 0.75);
		resize(img_dilation, img_dilation, Size(), 0.75, 0.75);

		imshow("original image", rgbImage);
        	imshow("green filter", img_dilation);
		waitKey(30);
    	    }
    	    catch(cv_bridge::Exception& e){
        	std::cout<<"error"<<std::endl;
            }
	}

	void depthCallback(const sensor_msgs::msg::Image::SharedPtr depthImageMsg){
	    try{
		Mat depthImage = cv_bridge::toCvShare(depthImageMsg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
		double min;
		double max;
		minMaxLoc(depthImage, &min, &max);
		//int max_int = ceil(max);
		//Mat win_mat(Size(depthImage.cols, depthImage.rows), CV_8UC1, Scalar(0,0,0));
		depthImage.convertTo(depthImage, CV_8UC3, 255 / (max-min), -255 * min / (max-min));
		//depthImage.convertTo(depthImage, CV_8UC3);
		Mat falseColor; // size (640,480)
		applyColorMap(depthImage, falseColor, COLORMAP_JET);
		imshow("depth image", falseColor);
	    }
	    catch(cv_bridge::Exception& e){
		std::cout<<"error"<<std::endl;
	    }
	}

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_1;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_2;
};

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    //node = rclcpp::Node::make_shared("rgb_subscriber_node");
    std::cout<<"subscribe"<<std::endl; 
    //rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub = node->create_subscription<sensor_msgs::msg::Image>("color/video/image", 10, &rgbCallback);
    //auto sub = node->create_subscription<sensor_msgs::msg::Image>("rgb_image", 10, &rgbCallback);
    //rclcpp::spin(node);
    rclcpp::spin(make_shared<SegmentationSub>());
    rclcpp::shutdown();
    return 0;
}
