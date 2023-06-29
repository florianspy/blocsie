# blocsie
BLOCSIE - Benchmark for LOCalization in a Simulated Industrial Environment
## unity package
Unity version required is 2022.x, ROS version required is ROS melodic
Link to the unity project
https://drive.google.com/file/d/1koVoI8hh0QAry02isOlENKvnfQ_PISZm/view
Import the package in unity and also install unity ros_tcp_endpoint package following the guide https://github.com/Unity-Technologies/ROS-TCP-Connector.
Additionally you need to install the unity perception package following the guide https://docs.unity3d.com/Packages/com.unity.perception@1.0/manual/SetupSteps.html
The Assets from external sources are listed in 
## evaluation
Link to the evaluation part
https://github.com/florianspy/locchallbench/tree/main
## ros messages
The source code for the ros package containing the ros msg definition for transmitting angle, material, distance in one msg is in the ros_msg folder. It is required to be build before the noise generator.
Compile it via the following command after copying the packaged into the catkin_ws src folder and going into the catkin_ws folder:

catkin_make
## lookup table generator 
The source code to create txt files used in the noise generator as a lookup table for the datadriven model can be found in modelgn folder.
Adopt now the mg.cpp file to generate the lookuptable files you require. 
For simple interpolation, (the case when all measurement data is available), use the constructor with 8 arguments.
For the scaling prediction approach (applicable for the LDS), use the constructor with 6 arguments, and ensure that the data in ref_material_1 in the first column is for 0 degree.
For the offset approach (applicable for the SICK TIM 561), use the constructor with 10 arguments, and ensure that the data in ref_material_1, ref_material_2 in the first column are both times for 0 degree.


Compile it via the following command:
g++ FileHandler.cpp MaterialInterpolator.cpp mg.cpp

## noise generator 
The source code for the noise generator can be found in noisegn folder, keep in mind it requires the FileHandler from modelgn folder. 
Adopt now inside noisegn.cpp file, the values of the variables need to be adjusted:
* lidarmatandangtopic == name of topic where Scanmat message containing distance, material, and angle information are published
* depthtopics == name of topic where Cammat message containing depth, material and angle information are published
* lidaroutputtopic == name of the lidar topic data with noise should be written into
* imgtopics == ros image topic of RGB images
* caminfo == the corresponding camerinfo topic to the RGB image
* depthcaminfo == the corresponding camera_info topic to the depth image
* amount_of_materials == amount of materials for which lookup table files should be used
* path_to_res == the path to the lookup table files
* ReadFile == change all the commands to match the lookup table files you want to use
* conststd == constant standard deviation value (only applied for lidar)


Compile it via the following command replace pathtoFileHandlercpp with the path you stored the modelgn:

1. g++  -I/opt/ros/melodic/include -I/home/catkin_ws/devel/include/lidarmatmsg -I/home/catkin_ws/src/interpolate/src -Ofast -g3 -c -Wall -fmessage-length=0 -std=c++14 -MMD -MP -MF  -c ng.cpp pathtoFileHandlercpp/FileHandler.cpp  -march=native
2. g++ -L/opt/ros/melodic/lib -L/opt/ros/melodic/lib/x86_64-linux-gnu -o "ng" ./ng.o ./FileHandler.o  -ltf2 -ltf -lxmlrpcpp -lcpp_common -lrosconsole_log4cxx -lroscpp_serialization -lopencv_calib3d -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lmessage_filters -lrostime -lroscpp -lboost_filesystem -lboost_system -lcv_bridge -lrosconsole_backend_interface -lrosconsole -ltf2_ros -lpthread -lboost_thread -lgsl -lgslcblas -lrosbag  -lm -lrt -ldl -lconsole_bridge  -lrosbag_storage -lcv_bridge -lopencv_core -lopencv_imgproc  -march=native
