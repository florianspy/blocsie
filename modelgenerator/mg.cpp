#include "MaterialInterpolator.h"
#include <iostream>
//be aware when filehandler crashes your program just quits and does not throw an error
//static constexpr std::string_view project_folder = "@PROJECT_SOURCE_DIR@";

int main()
{
	std::string path_to_res = static_cast<std::string>("/home/catkin_ws/src/interpolate") + "/src/";
	FileHandler reader_lds_white(path_to_res + "LDS_whitepaperascsv.txt", ' ', ':');
	FileHandler reader_sick_alu(path_to_res + "SICK_alu_ascsv.txt", ' ', ':');
	FileHandler reader_sick_white(path_to_res + "SICK_whitepaperascsv.txt", ' ', ':');
	FileHandler reader_sick_black(path_to_res + "SICK_blackpaper_ascsv.txt", ' ', ':'); 
	FileHandler reader_grey_intensity(path_to_res + "grey_intensity.txt",path_to_res + "grey_interpolate.txt",' ', ':');
	FileHandler reader_alu_intensity(path_to_res + "alu_intensity.txt",path_to_res + "alu_interpolate.txt",' ', ':');
	std::vector<double> intensity_;
	for(int i=0;i<reader_grey_intensity.file_data().at(0).size();i++){
		intensity_.push_back(reader_grey_intensity.file_data().at(0)[i]);
	}
	//example code to predict for lds with scaling approach
	//MaterialInterpolator lds(reader_lds_white.file_data(),reader_lds_white.row_info_vec(),0.469,0.531,intensity_, reader_grey_intensity.col_info_vec(),3.5,0.1,5);
	//lds.PrintpredictedMat();
	//example code to predict for sick with offset approach
	MaterialInterpolator sick(reader_sick_white.file_data(),reader_sick_white.row_info_vec(),reader_sick_black.file_data(),reader_sick_black.row_info_vec(),0.179,1.9992,-1.164,0.452,0.548,intensity_, reader_grey_intensity.col_info_vec(),10,0.5,5);
	sick.PrintpredictedMat();	
	//example code to interpolate data when measured at all distances / degrees for sick without offset approach. In this case the intensity values are not used, you can uses those of grey_intensity
	//MaterialInterpolator refmat(reader_sick_white.file_data(),reader_sick_white.row_info_vec(),reader_sick_white.col_info_vec(),intensity_,reader_alu_intensity.col_info_vec(),10,0.5,5,false);
	//refmat.PrintpredictedMat();
	//std::vector<std::vector<double>> full=refmat.sigma_with_deg_dis();
	//reader_sick_black.set_target_file_path(path_to_res+"SICK_alu_interpolate.txt");
	//reader_sick_black.WriteFile(full);
	return 1;
}
