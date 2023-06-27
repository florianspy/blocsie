# blocsie
BLOCSIE - Benchmark for LOCalization in a Simulated Industrial Environment

Unity version required is 2022.x, ROS version required is ROS melodic
Link to the unity project
https://drive.google.com/file/d/1koVoI8hh0QAry02isOlENKvnfQ_PISZm/view
Import the package in unity and also install unity ros_tcp_endpoint package following the guide https://github.com/Unity-Technologies/ROS-TCP-Connector.
Additionally you need to install the unity perception package following the guide https://docs.unity3d.com/Packages/com.unity.perception@1.0/manual/SetupSteps.html
The Assets from external sources are listed in 

Link to the evaluation part
https://github.com/florianspy/locchallbench/tree/main

The source code for the ros package containing the ros msg definition for transmitting angle, material, distance in one msg is in the ros_msg folder. It is required to be build before the noise generator.
Compile it via the following command after copying the packaged into the catkin_ws src folder and going into the catkin_ws folder:
catkin_make

The source code to create txt files used in the noise generator as a lookup table for the datadriven model can be found in modelgn folder
Compile it via the following command:

The source code for the noisegenerator can be found in noisegn folder, keep in mind it requires the FileHandler from modelgn folder.
Compile it via the following command replace pathtoFileHandlercpp with the path you stored the modelgn:

g++  -I/opt/ros/melodic/include -I/home/catkin_ws/devel/include/lidarmatmsg -I/home/catkin_ws/src/interpolate/src -Ofast -g3 -c -Wall -fmessage-length=0 -std=c++14 -MMD -MP -MF  -c noisegn.cpp pathtoFileHandlercpp/FileHandler.cpp  -march=native

g++ -L/opt/ros/melodic/lib -L/opt/ros/melodic/lib/x86_64-linux-gnu -o "noisegn" ./noisegn.o ./FileHandler.o  -ltf2 -ltf -lxmlrpcpp -lcpp_common -lrosconsole_log4cxx -lroscpp_serialization -lopencv_calib3d -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lmessage_filters -lrostime -lroscpp -lboost_filesystem -lboost_system -lcv_bridge -lrosconsole_backend_interface -lrosconsole -ltf2_ros -lpthread -lboost_thread -lgsl -lgslcblas -lrosbag  -lm -lrt -ldl -lconsole_bridge  -lrosbag_storage -lcv_bridge -lopencv_core -lopencv_imgproc  -march=native
