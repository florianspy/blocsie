#ifndef DATA_INTERPOLATOR_H_
#define DATA_INTERPOLATOR_H_

#include "FileHandler.h"


/**
 * @class DataInterpolator
 * @authors Daniel Blümm, Florian Spieß
 * @version 1.0.0
 * @date August 25th 2022
 * @brief This class can be used to interpolate data in different ways
 * @details Class used to interpolate data provided by a FileHandler object. The source file used by the FileHandler has to
 * fulfill certain conditions in order for the interpolation to work (see FileHandler::ReadFile() for more information)
 */
class DataInterpolator
{
public:
    DataInterpolator(const FileHandler& fh, double row_step_size, double col_step_size);
    // Getter functions
    std::vector<std::vector<double>> row_interpol_matrix() const;
    std::vector<std::vector<double>> col_interpol_matrix() const;
    std::vector<std::vector<double>> interpol_matrix() const;
    // Member functions
    void InterpolateRows(double lower_bound, double upper_bound, bool row_info=true, bool col_info=true);
    void InterpolateColumns(double lower_bound, double upper_bound, bool row_info=true, bool col_info=true);
    void InterpolateMatrix(double lb_rows, double ub_rows, double lb_cols, double ub_cols, bool row_info=true, bool col_info=true);
    void PrintMatrix(const std::vector<std::vector<double>>& mat) const;
private:
    const FileHandler fh_; /**< FileHandler providing file_data, row and column information */
    const double row_step_size_;  /**< Step size of row-wise interpolation */
    const double col_step_size_;  /**< Step size of column-wise interpolation */
    std::vector<std::vector<double>> row_interpol_matrix_;  /**< Matrix to store the data generated during row-wise interpolation*/
    std::vector<std::vector<double>> col_interpol_matrix_;  /**< Matrix to store the data generated during column-wise interpolation*/
    std::vector<std::vector<double>> interpol_matrix_;  /**< Matrix to store the data generated during matrix interpolation*/
    // Member functions (private)
    size_t GetVectorID(const std::vector<double>& vec, double curr_val) const;
    double CalculateSlopeRows(const size_t col, const size_t upper_pos) const;
    double CalculateSlopeCols(const size_t row, const size_t upper_pos) const;
    double CalculateSlopeMats(const size_t row, const size_t upper_pos) const;
    double CalculateOffsetRows(const size_t col, const size_t upper_pos, const double m) const;
    double CalculateOffsetCols(const size_t row, const size_t upper_pos, const double m) const;
    double CalculateOffsetMats(const size_t row, const size_t upper_pos, const double m) const;
    void ClearMatrices();
};

#endif
