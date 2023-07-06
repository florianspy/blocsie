#include "FileHandler.h"


/**
 * @brief Construct a new FileHandler:: File Handler object
 * @details This constructor reads the file located at \p source_file_path and automatically generates the file path
 * used to store the data as well as the file name (see set_target_file_path() for more information)
 * @note Using the same FileHandler object to handle multiple files requires updating the \p target_file_path_ using
 * set_target_file_path(std::string target_file_path) to avoid override. Therefor using one FileHandler for each file is recommended
 * 
 * @param source_file_path The system path to the file
 * @param delim The deliminator seperating the data values in the file provided by \p souce_file_path
 * @param sec_delim In case the row information of a file is seperated from the rest of the data using a different delimiter
 */
FileHandler::FileHandler(std::string source_file_path, char delim, char sec_delim) 
            : source_file_path_(source_file_path), delim_(delim), sec_delim_(sec_delim)
{
    ReadFile(source_file_path_);

    set_target_file_path();
}


/**
 * @brief Construct a new FileHandler:: File Handler object
 * @details Reads the file located at \p source_file_path and sets the file path used to store the data
 * @note Using the same FileHandler object to handle multiple files requires updating the \p target_file_path_ using
 * set_target_file_path(std::string target_file_path) to avoid override. Therefor using one FileHandler for each file is recommended
 * 
 * @param source_file_path The path to the storage location of a file
 * @param target_file_path The path to the target directory extended by the name of the file the data should be stored in 
 * (if the file does not exist it will be created automatically)
 * @param delim The deliminator seperating the data values in the file provided by \p souce_file_path
 * @param sec_delim In case the row information of a file is seperated from the rest of the data using a different delimiter
 */
FileHandler::FileHandler(std::string source_file_path, std::string target_file_path, char delim, char sec_delim)
            : source_file_path_(source_file_path), target_file_path_(target_file_path), delim_(delim), sec_delim_(sec_delim)
{
    ReadFile(source_file_path_);
}



/**
 * @brief Getter function of member variable \p source_file_path_
 * 
 * @return std::string of the source file path
 */
std::string FileHandler::source_file_path() const
{
    return source_file_path_;
}


/**
 * @brief Getter function of member variable \p target_file_path_
 * 
 * @return std::string of the target file path
 */
std::string FileHandler::target_file_path() const
{
    return target_file_path_;
}


/**
 * @brief Getter function of member variable \p row_info_vec_
 * 
 * @return std::vector<double> containing the row information of the source file
 */
std::vector<double> FileHandler::row_info_vec() const
{
    return row_info_vec_;
}


/**
 * @brief Getter function of member variable \p col_info_vec_
 * 
 * @return std::vector<double> containing the column information of the source file
 */
std::vector<double> FileHandler::col_info_vec() const
{
    return col_info_vec_;
}


/**
 * @brief Getter function of member variable \p file_data_
 * 
 * @return std::vector<std::vector<double>> containing the data read from the file
 */
std::vector<std::vector<double>> FileHandler::file_data() const
{
    return file_data_;
}



/**
 * @brief Set \p target_file_path_ to be in the same directory as the source file and set the target name of the file to be the same
 * as the source file name extended with '_interpolated'
 */
void FileHandler::set_target_file_path()
{
    size_t last = source_file_path_.find_last_of('.');
    size_t insert = source_file_path_.size() - last;
    std::string file_extension = source_file_path_.substr(last, source_file_path_.size()-1);

    target_file_path_ = source_file_path_.substr(0, source_file_path_.size()-insert) +  "_interpolated" + file_extension;

    return;
}


/**
 * @brief Set \p target_file_path_ to given path
 * 
 * @param target_file_path The path the file will be stored in (path/to/dir/file_name.file_extension)
 */
void FileHandler::set_target_file_path(std::string target_file_path)
{
    target_file_path_ = target_file_path;
    
    return;
}




/**
 * @brief Read the file located at \p source_file_path and store the data in variable \p file_data_
 * @details This function reads a file and stores data in variable \p fiel_data_ . Files read with this function have to contain
 * values of type integer or float
 * @note The file data read can only be used in DataInterpolator or MaterialInterpolator if the following conditions are met:
 * -# The first row has to contain information about the collumn values (i.e. angle, distance)
 * -# The first column has to contain information about the row values (i.e. angle, distance)
 * -# The row information has to use a different delimiter so it can be identified
 * 
 * @param source_file_path The system path the file is stored in
 */
void FileHandler::ReadFile(std::string source_file_path)
{
    ClearMatrices();

    std::fstream interpol_file_handler;
    interpol_file_handler.open(source_file_path);

    if (interpol_file_handler.is_open())
    {
        std::string line;
        int line_count = 0;

        while (getline(interpol_file_handler, line)) 
        {
            file_data_.push_back({});
            std::stringstream str_stream(line);
            std::string value;

            while (getline(str_stream, value, delim_)) 
            {

                if (line_count == 0)
                {
		
                    col_info_vec_.push_back(std::stod(value));
                    
                    continue;
                }

                if (value.back() != sec_delim_)
                {
                    file_data_.at(line_count-1).push_back(std::stod(value));
                }
                else
                {
                    row_info_vec_.push_back(std::stod(value.substr(0, value.size()-1)));
                }
            }
            
            line_count++;
        }
        
        file_data_.pop_back();  // remove empty line in matrix
    }
    else 
    {
        std::cerr << "File " << source_file_path_ << " does not exist!" << std::endl;
        exit(0);
    }

    interpol_file_handler.close();

    std::cout << "File reading successfull!" << std::endl;

    return;
}


/**
 * @brief Write data of \p mat to a file
 * @details Writes all data stored in \p mat to a file reusing the source file delimiter to seperate values
 * 
 * @param mat The data that will be written to a file
 */
void FileHandler::WriteFile(const std::vector<std::vector<double>>& mat)
{
    std::fstream interpol_file_handler;
    interpol_file_handler.open(target_file_path_, std::ios::out);

    for (unsigned int i = 0; i < mat.size(); i++)
    {
        for (unsigned int j = 0; j < mat.at(i).size(); j++)
        {
            if (!std::isnan(mat.at(i).at(j)))
            {
		if(j==0 && i != 0){
			interpol_file_handler << mat.at(i).at(j)<<sec_delim_<< delim_;
		}
		else{
			interpol_file_handler << mat.at(i).at(j) << delim_;
		}
            }
            else
            {
                interpol_file_handler << INFINITY << delim_;
            }
        }

        // avoid empty line at the end of the file
        if (i != mat.size()-1)
        {
            interpol_file_handler << "\n";
        }
    }

    interpol_file_handler.close();

    std::cout << "File " << target_file_path_ << " was created successfully!" << std::endl;

    return;
}


/**
 * @brief Print \p file_data_ to the console
 */
void FileHandler::PrintFileData() const
{
    for (unsigned int i = 0; i < file_data_.size(); i++)
    {
        for (unsigned int j = 0; j < file_data_.at(i).size(); j++)
        {
            std::cout << file_data_.at(i).at(j) << " ";
        }
        std::cout << std::endl;
    }
}




/**
 * @brief Clear all matrices and vectors so they can be reused to read and write files
 */
void FileHandler::ClearMatrices()
{
    row_info_vec_.clear();
    col_info_vec_.clear();
    file_data_.clear();
}
