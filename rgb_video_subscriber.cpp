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
	    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("segmentation", 10);
	    timer_ = this->create_wall_timer(500ms, bind(&SegmentationSub::pub_callback, this));
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
		resize(img_dilation, green_binary, Size(640, 480));

		green_binary.convertTo(green_binary, CV_8UC1);

		imshow("original image", rgbImage);
        	imshow("green filter", green_binary);
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

		//convert depth image into 0-255 information
		//depthImage.convertTo(depthImage, CV_8UC1, 255 / (max-min), -255 * min / (max-min));
		convertScaleAbs(depthImage, depthImage, 255/max);
		Mat falseColor, cdimg, element; // size (640,480)
		applyColorMap(depthImage, cdimg, COLORMAP_JET);
		threshold(depthImage, falseColor, 100, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);

		//erosion and dilation
		element = getStructuringElement(MORPH_RECT, Size(7,7), Point(3,3));
		erode(falseColor, falseColor, element);
		dilate(falseColor, depth_binary, element);
		
		//show the depth image
		imshow("depth image", depth_binary);
	    }
	    catch(cv_bridge::Exception& e){
		std::cout<<"error"<<std::endl;
	    }
	}

	void pub_callback(){
	    cv_bridge::CvImagePtr cv_ptr;
	    
	    Mat comb, proc, element;
	    
	    //switch the content of depth_binary and green_binary either 0 or 255 
	    for(int i=0; i<green_binary.rows; ++i){
		    for(int j=0; j<green_binary.cols; ++j){
			    if(depth_binary.at<int>(i,j) != 0) depth_binary.at<int>(i,j) = 255;
			    if(green_binary.at<int>(i,j) != 0) green_binary.at<int>(i,j) = 255;
		    }
	    }
	   
	    //combine depth_binary and green_binary 
	    addWeighted(depth_binary, 0.5, green_binary, 0.5, 0.0, comb);
	    
	    proc = comb.clone();

	    //gaussian blur 
	    blur(proc, proc, Size(3,3));

	    //dilate the image to reduce the noise
	    element = getStructuringElement(MORPH_RECT, Size(5,5), Point(2,2));
	    dilate(proc, proc, element);
	    
	    //canny to do edge detection 
	    Canny(proc, proc, 100, 200, 3, false);

	    //find the contours of the image
	    vector<vector<Point>> contours;
	    vector<Vec4i> hierarchy;
	    findContours(proc, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
	    
	    //draw the contours on the combined image
	    cvtColor(comb, comb, COLOR_GRAY2BGR);
	    drawContours(comb, contours, -1, Scalar(0, 255, 0), 2);

	    /*
	    cvtColor(cdimg, cdimg, COLOR_BGR2GRAY);
	    blur(cdimg, cdimg, Size(3,3));
	    Canny(cdimg, cdimg, 100, 200, 3, false);

	    vector<vector<Point>> contours1;
	    vector<Vec4i> hierarchy1;
	    findContours(cdimg, contours1, hierarchy1, RETR_TREE, CHAIN_APPROX_NONE);

	    drawContours(comb, contours1, -1, Scalar(0, 0, 255), 2);
	    */
	    
	    //publish bgr result image
	    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", comb).toImageMsg();

	    publisher_->publish(*msg.get());
	}

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_1;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_2;
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

	size_t count_;

	Mat depth_binary, green_binary, cdimg;
};

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    //node = rclcpp::Node::make_shared("rgb_subscriber_node");
    //rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub = node->create_subscription<sensor_msgs::msg::Image>("color/video/image", 10, &rgbCallback);
    //auto sub = node->create_subscription<sensor_msgs::msg::Image>("rgb_image", 10, &rgbCallback);
    //rclcpp::spin(node);
    rclcpp::spin(make_shared<SegmentationSub>());
    rclcpp::shutdown();
    return 0;
}
