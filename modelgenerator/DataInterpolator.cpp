#include "DataInterpolator.h"


/**
 * @brief Construct a new DataInterpolator:: Data Interpolator object
 * @note DataInterpolator objects can only be used to interpolate the matrix provided by FileHandler \p fh
 * 
 * @param fh Provides data for interpolation
 * @param row_step_size Step size for interpolating rows
 * @param col_step_size Step size for interpolating columns
 */
DataInterpolator::DataInterpolator(const FileHandler& fh, double row_step_size, double col_step_size) : fh_(fh), row_step_size_(row_step_size), col_step_size_(col_step_size)
{
    try
    {
        if (fh_.col_info_vec().size() == 0 || fh_.row_info_vec().size() == 0)
        {
            throw std::invalid_argument("FileHandler is not providing row or column information needed for the inperpolation! ");
        }
    }
    catch(const std::invalid_argument& ia)
    {
        std::cerr << ia.what() << "See documentation of FileHandler::ReadFile() for more information" << std::endl;
    }
}


#pragma region Getter functions
/**
 * @brief Getter function of member variable \p row_interpol_matrix_
 * 
 * @return std::vector<std::vector<double>> containing the data created during row-wise interpolation
 */
std::vector<std::vector<double>> DataInterpolator::row_interpol_matrix() const
{
    return row_interpol_matrix_;
}


/**
 * @brief Getter function of member variable \p col_interpol_matrix_
 * 
 * @return std::vector<std::vector<double>> containing the data created during column-wise interpolation
 */
std::vector<std::vector<double>> DataInterpolator::col_interpol_matrix() const
{
    return col_interpol_matrix_;
}


/**
 * @brief Getter function of member variable \p interpol_matrix_
 * 
 * @return std::vector<std::vector<double>> containing the data created during matrix interpolation
 */
std::vector<std::vector<double>> DataInterpolator::interpol_matrix() const
{
    return interpol_matrix_;
}
#pragma endregion


#pragma region Member functions
/**
 * @brief Uses \p row_step_size provided in constructor to interpolate rows of matrix provided by FileHandler \p fh
 * 
 * @param lower_bound Specifies where interpolation starts
 * @param upper_bound Specifies where interpolation ends
 * @param row_info Enables (true) or disables (false) insertion of row information at the beginning of each row (i.e. angle / distance)
 * @param col_info Enables (true) or disables (false) insertion of column information at the top of each column (i.e. angle / distance)
 */
void DataInterpolator::InterpolateRows(double lower_bound, double upper_bound, bool row_info, bool col_info)
{
    ClearMatrices();

    double curr_val = lower_bound;
    int row_count = 0;

    if(col_info)
    {
        row_count = 1;
        row_interpol_matrix_.push_back({});
    }

    while (curr_val <= upper_bound)
    {
        try
        {
            size_t pos = GetVectorID(fh_.row_info_vec(), curr_val);

            if (row_info)
            {
                row_interpol_matrix_.push_back({curr_val});
            }
            else
            {
                row_interpol_matrix_.push_back({});
            }

            for (size_t col = 0; col < fh_.file_data().at(0).size(); col++)
            {
                if (row_count == 1 && col_info)
                {
                    row_interpol_matrix_.at(0).push_back(fh_.col_info_vec().at(col));
                }

                try
                {
                    double m = CalculateSlopeRows(col, pos);
                    double t = CalculateOffsetRows(col, pos, m);

                    row_interpol_matrix_.at(row_count).push_back(m * curr_val + t);
                }
                catch(const std::out_of_range& oor)
                {
                    row_interpol_matrix_.at(row_count).push_back(INFINITY);
                }
            }

            row_count++;
            curr_val += row_step_size_;
        }
        catch(const std::out_of_range& oor)
        {
            std::cerr << "Out of range exception detected in function InterpolateRows!\n" << std::endl;

            exit(0);
        }
    }

    return;
}


/**
 * @brief Uses \p col_step_size provided in constructor to interpolate columns of matrix provided by FileHandler \p fh
 * 
 * @param lower_bound Specifies where interpolation starts
 * @param upper_bound Specifies where interpolation ends
 * @param row_info Enables (true) or disables (false) insertion of row information at the beginning of each row (i.e. angle / distance)
 * @param col_info Enables (true) or disables (false) insertion of column information at the top of each column (i.e. angle / distance)
 */
void DataInterpolator::InterpolateColumns(double lower_bound, double upper_bound, bool row_info, bool col_info)
{
    ClearMatrices();

    try
    {
        if (col_info)
        {
            col_interpol_matrix_.push_back({});
        }

        for (size_t row = 0; row < fh_.file_data().size(); row++)
        {
            double curr_val = lower_bound;

            if (row_info)
            {
                col_interpol_matrix_.push_back({fh_.row_info_vec().at(row)});
            }
            else
            {
                col_interpol_matrix_.push_back({});
            }

            while (curr_val <= upper_bound)
            {
                if (row == 1 && col_info)
                {
                    col_interpol_matrix_.at(0).push_back(curr_val);
                }

                size_t pos = GetVectorID(fh_.col_info_vec(), curr_val);

                double m = CalculateSlopeCols(row, pos);
                double t = CalculateOffsetCols(row, pos, m);

                if (col_info)
                {
                    col_interpol_matrix_.at(row+1).push_back(m * curr_val + t);
                }
                else
                {
                    col_interpol_matrix_.at(row).push_back(m * curr_val + t);
                }

                curr_val += col_step_size_;
            }
        }        
    }
    catch (const std::out_of_range& oor)
    {
        std::cerr << "Out of range exception detected in function InterpolateColumns!\n" << std::endl;
        exit(0);
    }

    return;
}


/**
 * @brief Uses \p row_step_size and \p col_step_size provided in constructor to interpolate rows and columns 
 * of matrix provided by FileHandler \p fh
 * 
 * @param lb_rows Specifies where row interpolation starts
 * @param ub_rows Specifies where row interpolation ends
 * @param lb_cols Specifies where column interpolation starts
 * @param ub_cols Specifies where column interpolation ends
 * @param row_info Enables (true) or disables (false) insertion of row information at the beginning of each row (i.e. angle / distance)
 * @param col_info Enables (true) or disables (false) insertion of column information at the top of each column (i.e. angle / distance)
 */
void DataInterpolator::InterpolateMatrix(double lb_rows, double ub_rows, double lb_cols, double ub_cols, bool row_info, bool col_info)
{
    InterpolateRows(lb_rows, ub_rows, row_info);
    
    try
    {
        if (col_info)
        {
            interpol_matrix_.push_back({});
        }

        for (size_t row = 0; row < row_interpol_matrix_.size(); row++)
        {
            double curr_val = lb_cols;

            interpol_matrix_.push_back({});

            // calculate interpolations (x = angle, y = sigma)
            while (curr_val <= ub_cols)
            {
                if (row == 1 && col_info)
                {
                    interpol_matrix_.at(0).push_back(curr_val);
                }

                size_t pos = GetVectorID(fh_.col_info_vec(), curr_val);

                double m = CalculateSlopeMats(row, pos);
                double t = CalculateOffsetMats(row, pos, m);

                if (col_info)
                {
                    interpol_matrix_.at(row+1).push_back(m * curr_val + t);
                }
                else
                {
                    interpol_matrix_.at(row).push_back(m * curr_val + t);
                }

                curr_val += col_step_size_;
            }
        }        
    }
    catch (const std::out_of_range& oor)
    {
        std::cerr << "Out of range exception detected in function InterpolateMatrix!\n" << std::endl;
        exit(0);
    }

    return;
}


/**
 * @brief Print \p mat to console
 * 
 * @param mat The matrix that will be printed to the console
 */
void DataInterpolator::PrintMatrix(const std::vector<std::vector<double>>& mat) const
{
    for (size_t i = 0; i < mat.size(); i++)
    {
        for (size_t j = 0; j < mat.at(i).size(); j++)
        {
            std::cout << mat.at(i).at(j) << " ";
        }
        std::cout << std::endl;
    }
}
#pragma endregion


#pragma region Member functions (private)
/**
 * @brief Determines index needed for interpolation based on a given value
 * 
 * @param vec std::vector providing values for comparison
 * @param curr_val Value to compare elements to 
 * 
 * @return Index needed for interpolation
 */
size_t DataInterpolator::GetVectorID(const std::vector<double>& vec, double curr_val) const
{
    try
    {
        if (curr_val <= vec.front())
        {
            return 1;
        }
        else if (curr_val > vec.front() && curr_val <= vec.back())
        {
            return std::lower_bound(vec.begin(), vec.end(), curr_val) - vec.begin();
        }
        else if (curr_val > vec.back())
        {
            return vec.size() - 1;
        }
        else
        {
            throw std::invalid_argument("Invalid argument detected in function InterpolateCols!\n");
        }
    }
    catch(const std::invalid_argument& e)
    {
        std::cerr << e.what() << '\n';
        exit(0);
    }
}


/**
 * @brief Calculates slope for row-wise interpolation
 * 
 * @param col Current column index of interpolation
 * @param pos Current position of interpolation
 * 
 * @return Slope at given \p col and \p pos
 */
double DataInterpolator::CalculateSlopeRows(const size_t col, const size_t pos) const
{
    return ((fh_.file_data().at(pos).at(col) - fh_.file_data().at(pos-1).at(col)) /
            (fh_.row_info_vec().at(pos) - fh_.row_info_vec().at(pos-1)));
}


/**
 * @brief Calculates slope for column-wise interpolation
 * 
 * @param row Current row index of interpolation 
 * @param pos Current position of interpolation
 * 
 * @return Slope at given \p row and \p pos
 */
double DataInterpolator::CalculateSlopeCols(const size_t row, const size_t pos) const
{
    return ((fh_.file_data().at(row).at(pos) - fh_.file_data().at(row).at(pos-1)) /
            (fh_.col_info_vec().at(pos) - fh_.col_info_vec().at(pos-1)));
}


/**
 * @brief Calculates slope for matrix interpolation
 * 
 * @param row Current row index of interpolation
 * @param pos Current position of interpolation
 * 
 * @return Slope at given \p row and \p pos
 */
double DataInterpolator::CalculateSlopeMats(const size_t row, const size_t pos) const
{
    return ((row_interpol_matrix_.at(row).at(pos) - row_interpol_matrix_.at(row).at(pos-1)) /
            (fh_.col_info_vec().at(pos) - fh_.col_info_vec().at(pos-1)));
}


/**
 * @brief Calculates offset for row-wise interpolation
 * 
 * @param col Current column index of interpolation
 * @param pos Current position of interpolation
 * @param m Current slope
 * 
 * @return Offset at given \p col and \p pos 
 */
double DataInterpolator::CalculateOffsetRows(const size_t col, const size_t pos, const double m) const
{
    return (fh_.file_data().at(pos).at(col) - m * fh_.row_info_vec().at(pos));
}


/**
 * @brief Calculates offset for column-wise interpolation
 * 
 * @param row Current row index of interpolation
 * @param pos Current position of interpolation
 * @param m Current slope
 * 
 * @return Offset at given \p row and \p pos 
 */
double DataInterpolator::CalculateOffsetCols(const size_t row, const size_t pos, const double m) const
{
    return (fh_.file_data().at(row).at(pos) - m * fh_.col_info_vec().at(pos));
}


/**
 * @brief Calculates offset for matrix interpolation
 * 
 * @param row Current row index of interpolation
 * @param pos Current position of interpolation
 * @param m Current slope
 * 
 * @return Offset at given \p row and \p pos 
 */
double DataInterpolator::CalculateOffsetMats(const size_t row, const size_t pos, const double m) const
{
    return (row_interpol_matrix_.at(row).at(pos) - m * fh_.col_info_vec().at(pos));
}


/**
 * @brief Clear all matrices so they can be reused to interpolate data
 */
void DataInterpolator::ClearMatrices()
{
    row_interpol_matrix_.clear();
    col_interpol_matrix_.clear();
    interpol_matrix_.clear();
}
#pragma endregion
