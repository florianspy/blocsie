# BLOCSIE
BLOCSIE - Benchmark for LOCalization in a Simulated Industrial Environment
## Unity package
The required Unity version is 2022.x, the required ROS version is ROS melodic

Link to the unity project
https://drive.google.com/file/d/1koVoI8hh0QAry02isOlENKvnfQ_PISZm/view
* Install the unity perception package following the guide https://docs.unity3d.com/Packages/com.unity.perception@1.0/manual/SetupSteps.html
* Install the unity ros_tcp_endpoint package following the guide https://github.com/Unity-Technologies/ROS-TCP-Connector
* Import the downloaded package in Unity 

The sources for the asset are listed in the file https://github.com/florianspy/blocsie/blob/main/BLOCSIE___UnityAssetSources.pdf
## Evaluation
The scripts for the evaluation can be downloaded from the following page https://github.com/florianspy/locchallbench/tree/main
## ROS 
### ROS messages
The source code for the ROS package containing the ROS message definition for transmitting angle, material, distance in one msg is in the ros_msg folder. It is a prerequisite for the noise generator. Compile it via the following command after copying the package into the catkin_ws src folder and going into the catkin_ws folder:

catkin_make
### ROS Unity connection
The connection between Unity and ROS requires building the following package ros_tcp_endpoint.
1. Download the package from  https://github.com/Unity-Technologies/ROS-TCP-Endpoint into your catkin_ws/src folder
2. catkin_make

## Lookup table generator 
The source code to create txt files used in the noise generator as a lookup table for the data-driven model can be found in the modelgenerator folder.
Adopt now the mg.cpp file to generate the lookup table files you require. It already contains an example source code.
* For simple interpolation (the case when all measurement data is available), use the constructor with eight arguments.
* For the scaling prediction approach (applicable for the LDS), use the constructor with six arguments, and ensure that the data in ref_material_1 in the first column is for 0 degrees.
* For the offset approach (applicable for the SICK TIM 561), use the constructor with ten arguments, and ensure that the data in ref_material_1, and ref_material_2 in the first column are both times for 0 degrees.

Compile it via the following command:

g++ FileHandler.cpp MaterialInterpolator.cpp mg.cpp

## Noise generator 
The source code for the noise generator can be found in the noisegenerator folder. It requires the FileHandler from the modelgenerator folder. Before you use any rosbag from unity, ensure that you have run the python script rosbagwrite.py before, as it will write the messages into the rosbag according to the message time stamps and not the arrival time during recording. It is used with the following syntax (replace input.bag with the name of the bag you want the timestamps to be sorted):

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
