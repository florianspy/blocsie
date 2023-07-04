# BLOCSIE
BLOCSIE - Benchmark for LOCalization in a Simulated Industrial Environment
The Project was tested with the following software versions:
Unity version: 2022.x (Perception package minimum version is 2021.x, ROS version: ROS melodic.

## Unity package
The sources for the asset are listed in the file https://github.com/florianspy/blocsie/blob/main/BLOCSIE___UnityAssetSources.pdf

### installation
* Install the unity perception package following the guide https://docs.unity3d.com/Packages/com.unity.perception@1.0/manual/SetupSteps.html
* Install the unity ros_tcp_endpoint package following the guide https://github.com/Unity-Technologies/ROS-TCP-Connector
* Download unity project from
https://drive.google.com/file/d/1koVoI8hh0QAry02isOlENKvnfQ_PISZm/viewImport and install package in Unity 
### usage
1. Select which scenario you want to use by selecting the specific scenario in the top left menu and turning off the others.
2. Use the path creator object to specify the path through the environment as well as the speed.
To make the robot follow the created path, select the robot and scroll to the script called wheel_odom. Now select from the path object you created the height object and choose it for the Target Path variable of the wheel_odom script. 
3. Click on the Gameobject "robot" and adjust the sensors you want to use.
* The camera sensor(s) can be found below the Gameobject "d415". It allows one to set the camera resolution (via Script), the distortion parameters (via Script), frame_id (via Script), frame rate (via Script), the name of the topics (published topics set via cameratopic, caminfotopic via the Script), the field of view (via the Camera itself), and near and far clipping (via the Camera itself).
Further, the depth camera is accessible from here. It has the same options as the camera object. The depth camera, which publishes the material and angle of hit information, is available here (published topics set via cameratopic, caminfotopic).
* The LIDAR can be adjusted at the Gameobject "sick." Here options such as minimum range, maximum range, start angle, angular range, scans per second, and total amount of rays can be set up. The LIDAR script publishes distance data on the topic set with the variable "lidar_topic", while on the topic set with "matandangtopic", the messages contain information about distance, material, and angle of hit.
* Transformations can be found within the tf script (published topic set via tf_static), and the ground truth publisher is inside the Position Script (published topic: gt); both are directly attached to the Gameobject "robot".
*  Wheel odometry is published via the wheel_odom script, and data is provided via the topic set via "odometrytop". It publishes the /clock topic. During each execution, the robot is moved to its next position, and physics steps are triggered. This script keeps track of the simulation time and ensures deterministic simulation. The transformation from odom to base_footprint is also published here.
4. Before you click on the play button to start the simulation, ensure that the ROS side is already running and you already started the recording of the rosbag.
## Evaluation
The scripts for the evaluation can be downloaded from the following page https://github.com/florianspy/locchallbench/tree/main/gui. The github page contains the installation and the usage description.
## ROS 
### ROS messages
The source code for the ROS package containing the ROS message definition for transmitting angle, material, distance in one msg is in the ros_msg folder. It is a prerequisite for the noise generator. Compile it via the following command after copying the package into the catkin_ws src folder and going into the catkin_ws folder:

catkin_make
### ROS Unity connection
The connection between Unity and ROS requires building the following package ros_tcp_endpoint.
1. Download the package from  https://github.com/Unity-Technologies/ROS-TCP-Endpoint into your catkin_ws/src folder
2. catkin_make
### usage 
roslaunch ros_tcp_endpoint endpoint.launch

## Lookup table generator 
The source code to create txt files used in the noise generator as a lookup table for the data-driven model can be found in the modelgenerator folder.
Adopt now the mg.cpp file to generate the lookup table files you require. It already contains an example source code.
* For simple interpolation (the case when all measurement data is available), use the constructor with nine arguments, where the last argument is a bool.
* For the scaling prediction approach (applicable for the LDS), use the constructor with nine arguments, and ensure that the data in ref_material_1 in the first column is for 0 degrees.
* For the offset approach (applicable for the SICK TIM 561), use the constructor with fourteen arguments, and ensure that the data in ref_material_1, and ref_material_2 in the first column are both times for 0 degrees.
### compiling
Compile it via the following command:

g++ FileHandler.cpp MaterialInterpolator.cpp mg.cpp
### usage
The compiled file does not provide user interaction as all the commands have to be done within the source code. To generate the files a simple run command is necessary.
./compiledfile
## Noise generator 
The source code for the noise generator can be found in the noisegenerator folder. It requires the FileHandler from the modelgenerator folder. Before you use any rosbag from unity, ensure that you have run the python script rosbagwrite.py before, as it will write the messages into the rosbag according to the message time stamps and not the arrival time during recording. It is used with the following syntax (replace input.bag with the name of the bag you want the timestamps to be sorted, while output.bag will contain the sorted messages. The timestamps are only accesible with the python api, therefor, this python script is required.):

python rosbagwrite.py input.bag output.bag
### adjusting the source code
Inside ng.cpp file, the values of the following variables need to be adjusted to meet your settings:
* lidarmatandangtopic == name of the topic where Scanmat message containing distance, material, and angle information is published
* depthtopics == name of the topic where Cammat message containing depth, material, and angle information are published
* lidaroutputtopic == name of the lidar topic data with noise should be written into
* imgtopics == ros image topic of RGB images
* caminfo == the corresponding camera_info topic to the RGB image
* depthcaminfo == the corresponding camera_info topic to the depth image
* amount_of_materials_sensor == amount of materials = amount of lookup tables for this sensor
* path_to_res == the path to the lookup table files
* ReadFile == change all the commands to match the lookup table files you want to use
* conststd == constant standard deviation value (only applied for lidar)

### compiling

Compile it via the following command replace pathtoFileHandlercpp with the path you stored the modelgenerator:

1. g++  -I/opt/ros/melodic/include -I/home/catkin_ws/devel/include/lidarmatmsg -I/home/catkin_ws/src/interpolate/src -Ofast -g3 -c -Wall -fmessage-length=0 -std=c++14 -MMD -MP -MF  -c ng.cpp pathtoFileHandlercpp/FileHandler.cpp  -march=native
2. g++ -L/opt/ros/melodic/lib -L/opt/ros/melodic/lib/x86_64-linux-gnu -o "ng" ./ng.o ./FileHandler.o  -ltf2 -ltf -lxmlrpcpp -lcpp_common -lrosconsole_log4cxx -lroscpp_serialization -lopencv_calib3d -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lmessage_filters -lrostime -lroscpp -lboost_filesystem -lboost_system -lcv_bridge -lrosconsole_backend_interface -lrosconsole -ltf2_ros -lpthread -lboost_thread -lgsl -lgslcblas -lrosbag  -lm -lrt -ldl -lconsole_bridge  -lrosbag_storage -lcv_bridge -lopencv_core -lopencv_imgproc  -march=native
   
### usage
The compiled file can then be used in the following way (input.bag is the rosbag that contains your data, outputfolder is the folder where the results are written to, amountofdatasets are the number of datasets that should be created utilizing continuously the same random number generator):

./ng input.bag outputfolder amountofdatasets

You will get two rosbags for each dataset, one with a constant standard deviation (noise-conststd) and one using the data-driven model to create the noise (noise-datastd).

![logo-color](https://github.com/florianspy/blocsie/assets/39183098/504c376e-401c-45a2-a8a5-c388fd32ef49)
