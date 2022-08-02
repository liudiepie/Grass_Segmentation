#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

using namespace std;
using namespace cv;

int main() {

    // Create pipeline
    dai::Pipeline pipeline;

    Mat frame, frame_proc, frame_dilation;

    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();

    xoutRgb->setStreamName("rgb");

    // Properties
    camRgb->setPreviewSize(640,360);
    //camRgb->setPreviewSize(1920,1080);
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);

    // Linking
    camRgb->preview.link(xoutRgb->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline, dai::UsbSpeed::SUPER);

    cout << "Connected cameras: ";
    for(const auto& cam : device.getConnectedCameras()) {
        cout << cam << " ";
    }
    cout << endl;

    // Print USB speed
    cout << "Usb speed: " << device.getUsbSpeed() << endl;

    // Output queue will be used to get the rgb frames from the output defined above
    auto qRgb = device.getOutputQueue("rgb", 20, false);

    while(true) {
        auto inRgb = qRgb->get<dai::ImgFrame>();

	//get frame 
	frame = inRgb->getCvFrame();
	
	//convert BGR to Lab
	cvtColor(frame, frame_proc, COLOR_BGR2Lab);

	//extract the L channel
	vector<Mat> lab_planes(3);
	split(frame_proc, lab_planes);

	//apply the CLAHE algorithm to the L channel
	Ptr<CLAHE> clahe = createCLAHE();
	clahe->setClipLimit(4);
	Mat dst;
	clahe->apply(lab_planes[0], dst);

	//merge the color planes back into an lab image
	dst.copyTo(lab_planes[0]);
	merge(lab_planes, frame_proc);

	//convert Lab to BGR
	cvtColor(frame_proc, frame_proc, COLOR_Lab2BGR);

	//convert BGR to HSV
	cvtColor(frame_proc, frame_proc, COLOR_BGR2HSV);

	//detect the object based on HSV range
	//H: 34 - 70, S: 60 - 255, V: 60 - 255
	inRange(frame_proc, Scalar(34, 60, 60), Scalar(70, 255, 255), frame_proc);

	//dilation Size: (2n+1), Point: (n), n = 4
	Mat element = getStructuringElement(MORPH_RECT, Size(9,9), Point(4,4));
	dilate(frame_proc, frame_dilation, element);

	//show the result
	if(!frame.empty()){
		flip(frame, frame, 0);
		flip(frame, frame, 1);
		imshow("rgb", frame);
		imshow("green filter", frame_dilation);
	}

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            break;
        }
    }
    return 0;
}
