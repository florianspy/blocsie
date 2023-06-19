#ifndef FILE_HANDLER_H_
#define FILE_HANDLER_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>


/**
 * @class FileHandler
 * @authors Daniel Blümm, Florian Spieß
 * @version 1.0.0
 * @date August 25th 2022
 * @brief This class can be used to read data from or write data to files
 */
class FileHandler
{
public:
    FileHandler(std::string source_file_path, char delim=',', char sec_delim=':');
    FileHandler(std::string source_file_path, std::string target_file_path, char delim=',', char sec_delim=':');
    // Getter functions
    std::string source_file_path() const;
    std::string target_file_path() const;
    std::vector<double> row_info_vec() const;
    std::vector<double> col_info_vec() const;
    std::vector<std::vector<double>> file_data() const;
    // Setter functions
    void set_target_file_path();
    void set_target_file_path(std::string target_file_path);
    // Member functions
    void ReadFile(std::string source_file_path);
    void WriteFile(const std::vector<std::vector<double>>& mat);
    void PrintFileData() const;
private:
    std::string source_file_path_; /**< Path to the source file location */
    std::string target_file_path_; /**< Path to the target file location */
    char delim_; /**< Main delimiter of a file */
    char sec_delim_; /**< Delimiter used to identifie row information values */
    std::vector<double> row_info_vec_; /**< Information about the row values (i.e. angle, distance) */
    std::vector<double> col_info_vec_; /**< Information about the column values (i.e. angle, distance) */
    std::vector<std::vector<double>> file_data_; /**< The main data of the file (does not contain row and column information) */
    // Member functions (private)
    void ClearMatrices();
};

#endif