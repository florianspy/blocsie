#include "DataInterpolator.h"
#include "config.hpp"


void d_main()
{
    std::string path_to_res = static_cast<std::string>(project_folder) + "/res/";

    FileHandler reader(path_to_res + "LDS_90.txt", ' ', ':');
    //reader.PrintMatrix();
    
    DataInterpolator interpolator(reader, 0.05, 1);
    interpolator.InterpolateMatrix(0.1, 1.5, 0, 10, true, true);
    interpolator.PrintMatrix(interpolator.interpol_matrix());
    reader.WriteFile(interpolator.interpol_matrix());     

	return;
}