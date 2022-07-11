# Green_filter
This project is to do a simple segmentation for the farmbot. When farmbot moves outdoor, it needs to do obstacle avoidance.   
However, grass is a kind of soft objects that farmbot can go through.  
If only apply for obstacle avoidance, farmbot will think grass as an obstacle it can't pass through.  
Therefore, this green filter is to ignore the green color from the camera to avoid this issue.  

## Pipeline
![](image/pipeline.JPG)

## Installation
This project is based on [depthai-ros](https://github.com/luxonis/depthai-ros) with ROS2-Foxy.    
First of all, follow the above link install dependencies in depthai-ros and build the repository.  
Second, go under workspace/ and replace the rgb_video_subscriber.cpp  
Third, go under workspace/ and replace the CMakeLists.txt  
Finally, build the workspace again, type  
```bash
colcon build --packages-select depthai_examples
```

## Execute
To run the files, create two terminals. Both terminals should under the workspace/  
choose one and type  
```bash
source install/setup.bash  
ros2 launch depthai_examples rgb_stereo_node  
```
switch to another terminal and type  
```bash
source install/setup.bash
ros2 run depthai_examples rgb_subscriber
```

## Result
<figure class="video_container">
  <iframe src="https://drive.google.com/file/d/1KawHPaypFYntZRwgIoU_QzIDGerMqJV5/view?usp=sharing" frameborder="0" allowfullscreen="true"> </iframe>
</figure>
