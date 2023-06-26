#include "rangeng.h"
#include "camng.h"

double ReturnDiff(std::vector<double> vec)
{
    return (vec[1] - vec[0]);
}

int main(int argc, char* argv[]){
	if(argc < 4){
		std::cout<<"Not enough arguments\n";
		return -1;
	}
	std::string bagPath = argv[1];
	std::string outputPath = argv[2];
	//amount of bags you want to create
	int numSamples = atoi(argv[3]);
	int amount_of_materials =4;
	int deg_step_width=5;
	double dist_step_width=0.5;
	double max_range=10;
	double conststd=0.02;
	int max_deg=85;
	//get the data from data model and fill it in
 	std::string path_to_res = static_cast<std::string>("/home/catkin_ws/src/interpolate/src/");
	FileHandler reader(path_to_res+"SICK_white_interpolate.txt", ' ', ':');
	deg_step_width = ReturnDiff(reader.col_info_vec());
	dist_step_width = ReturnDiff(reader.row_info_vec());
	RangeSensor lidar=RangeSensor(std::string(""),dist_step_width,deg_step_width,amount_of_materials,max_range,max_deg,conststd);	
	std::string lidarmatandangtopic ="/scan_frontmanda";
	std::string lidaroutputtopic = "/scan_front";
	std::string setting1 = outputPath + "";
	mkdir(setting1.c_str(),0777);
	RGB subpub = RGB();
	DepthCam depthcam = DepthCam(dist_step_width,deg_step_width,amount_of_materials,max_range,max_deg);
    std::vector<std::string> caminfo;
    caminfo.push_back(std::string("/caminfo"));  
	std::vector<std::string> imgtopics;
    imgtopics.push_back(std::string("/camera/color/image_raw"));	
	std::vector<std::string> depthtopics;
    depthtopics.push_back(std::string("/depth2"));	
	std::vector<std::string> depthcaminfo;
    depthcaminfo.push_back(std::string("/caminfo"));  
	lidar.insert_datamodel_matdata(0,reader.file_data());
	depthcam.insert_datamodel_matdata(0,reader.file_data());
	reader.ReadFile(path_to_res+"SICK_grey_interpolate.txt");
	lidar.insert_datamodel_matdata(1,reader.file_data());
	depthcam.insert_datamodel_matdata(1,reader.file_data());
	reader.ReadFile(path_to_res+"SICK_black_interpolate.txt");
	lidar.insert_datamodel_matdata(2,reader.file_data());
	depthcam.insert_datamodel_matdata(2,reader.file_data());
	reader.ReadFile(path_to_res+"SICK_alu_interpolate.txt");
	lidar.insert_datamodel_matdata(3,reader.file_data());
	depthcam.insert_datamodel_matdata(3,reader.file_data());

	//get first caminfo topic
	rosbag::Bag bagIn;
	rosbag::Bag bagOut,bagOut2;
	bagIn.open(bagPath, rosbag::bagmode::Read);
	rosbag::View viewIn(bagIn);
    foreach(rosbag::MessageInstance const msg, viewIn){
         if(msg.getTopic()==caminfo[0]){
               const sensor_msgs::CameraInfoPtr& inf = msg.instantiate<sensor_msgs::CameraInfo>();
               subpub.caminfo(inf);
        }  
		if(msg.getTopic()==depthcaminfo[0]){
               const sensor_msgs::CameraInfoPtr& inf = msg.instantiate<sensor_msgs::CameraInfo>();
               depthcam.depthcaminfo(inf);
				break;
        }  
    }
	//https://answers.ros.org/question/28766/rosbag-info-c-api/
	std::cout<<viewIn.size()<<std::endl;
	int percentage=viewIn.size()/10;
	// less then 10 message in rosbag
	if(percentage == 0){
		percentage=1;
	}
	for(int i=0;i<numSamples;i++){
			std::string path = setting1 + "/noise-conststd"+std::to_string(i)+"-.bag";
			std::string path2 = setting1 + "/noise-datastd"+std::to_string(i)+"-.bag";
			bagOut.open(path, rosbag::bagmode::Write);
			bagOut2.open(path2, rosbag::bagmode::Write);
			int count =0;
			foreach(rosbag::MessageInstance const msg, viewIn){
				count++;
				if(count%percentage==0){
					std::cout<<(count*1.0)/viewIn.size()*100.0<<"\% done"<<std::endl;
				}
				if(msg.getTopic() == lidarmatandangtopic){
					lidarmatmsg::Scanmat::ConstPtr msgPtr = msg.instantiate<lidarmatmsg::Scanmat>();
					bagOut.write(lidaroutputtopic, msgPtr->header.stamp,lidar.GenerateConstStdMsg(msgPtr));
					bagOut2.write(lidaroutputtopic, msgPtr->header.stamp,lidar.GenerateDataDrivenStdMsg(msgPtr));
				}
				else if(msg.getTopic()==imgtopics[0]){
					sensor_msgs::ImageConstPtr imsg = msg.instantiate<sensor_msgs::Image>();
					bagOut.write(msg.getTopic(), imsg->header.stamp,subpub.camimg(imsg));
					bagOut2.write(msg.getTopic(), imsg->header.stamp,subpub.camimg(imsg));
				}
				else if(msg.getTopic()==depthtopics[0]){
					lidarmatmsg::Cammat::ConstPtr imsg = msg.instantiate<lidarmatmsg::Cammat>();
					bagOut.write(msg.getTopic(), imsg->header.stamp,depthcam.depth(imsg));
					bagOut2.write(msg.getTopic(), imsg->header.stamp,depthcam.depth(imsg));
				}
				else if(msg.getTopic() == lidaroutputtopic){
				}
				else{
					if(msg.getTopic() == "/clock"){
						bagOut.write(msg.getTopic(), msg.getTime(), msg);
						bagOut2.write(msg.getTopic(), msg.getTime(), msg);
					}
					else{
						//std::cout<<msg.getTime()<<" ";
						bagOut.write(msg.getTopic(),msg.getTime(), msg);
						bagOut2.write(msg.getTopic(), msg.getTime(), msg);						
					}
				}
			}
			bagOut.close();
			bagOut2.close();
	}
    return 0;
}
