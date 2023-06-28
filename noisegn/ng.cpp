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
	int amount_of_materials_lidar=4;
	int deg_step_width_lidar;// degrees between two values in the lookup table
	double dist_step_width_lidar;//distance in m between two values in the lookup table
	double max_range_lidar;// maximum value of the distance in lookup table
	int max_deg_lidar;// maximum value of the degree in lookup table
	double conststd=0.02;
	//get the data from data model and fill it in
 	std::string path_to_res = static_cast<std::string>("/home/catkin_ws/src/interpolate/src/");
	FileHandler reader(path_to_res+"SICK_white_interpolate.txt", ' ', ':');
	deg_step_width_lidar = ReturnDiff(reader.col_info_vec());
	dist_step_width_lidar = ReturnDiff(reader.row_info_vec());
	max_range_lidar=reader.row_info_vec().back();
	max_deg_lidar=reader.col_info_vec().back();	
	RangeSensor lidar=RangeSensor(std::string(""),dist_step_width_lidar,deg_step_width_lidar,amount_of_materials_lidar,max_range_lidar,max_deg_lidar,conststd);	
	std::string lidarmatandangtopic ="/scan_frontmanda";
	std::string lidaroutputtopic = "/scan_front";
	std::string setting1 = outputPath + "";
	mkdir(setting1.c_str(),0777);
	RGB subpub = RGB();
	int amount_of_materials_dcam=4;
	int deg_step_width_dcam;// degrees between two values in the lookup table
	double dist_step_width_dcam;//distance in m between two values in the lookup table
	double max_range_dcam;// maximum value of the distance in lookup table
	int max_deg_dcam;// maximum value of the degree in lookup table
	deg_step_width_dcam = ReturnDiff(reader.col_info_vec());
	dist_step_width_dcam = ReturnDiff(reader.row_info_vec());
	max_range_dcam=reader.row_info_vec().back();
	max_deg_dcam=reader.col_info_vec().back();
	DepthCam depthcam = DepthCam(dist_step_width_dcam,deg_step_width_dcam,amount_of_materials_dcam,max_range_dcam,max_deg_dcam);
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
	//get first caminfo topic we require it for the distortion information which will be applied on each image
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
					bagOut.write(msg.getTopic(), msg.getTime(), msg);
					bagOut2.write(msg.getTopic(), msg.getTime(), msg);
					
				}
			}
			bagOut.close();
			bagOut2.close();
	}
    	return 0;
}