#ifndef SRC_UTILITY_H
#define SRC_UTILITY_H

#include <sensor_msgs/msg/laser_scan.hpp>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
namespace cass
{

    /// Function to Apply 1d Smoothing Filter (Averaging Filter)
    /// @param input_vector - Input Vector
    /// @param smoothing_filter_size - Size of the averaging required
    /// @return smoothened vector
    std::vector<float> apply_smoothing_filter(const std::vector<float> &input_vector, size_t smoothing_filter_size)
    {
        std::vector<float> smoothened_vector;
        for (size_t i = smoothing_filter_size; i < input_vector.size() - smoothing_filter_size; ++i)
        {
            double current_sum = 0;
            for (size_t j = i - smoothing_filter_size + 1; j < i + smoothing_filter_size; ++j)
            {
                current_sum += input_vector[j];
            }
            smoothened_vector.push_back(current_sum / (2.0 * smoothing_filter_size - 1.0));
        }
        return smoothened_vector;
    }

    /// Returns Indices of Start and End of the Lidar Scan for the input truncation angle converage
    /// @param scan_msg Lidar scansize_t
    /// @param truncation_angle_coverage (in rads)
    /// @return Start Index and End Index of Truncated Lidar Scan
    std::vector<float> truncate(const std::vector<float> ranges, const float angle_max, const float angle_min, const double truncation_angle_coverage)
    {
        const auto truncated_range_size = static_cast<size_t>(
            (truncation_angle_coverage / (angle_max - angle_min)) * ranges.size());
        const size_t start_index = (ranges.size() / 2) - (truncated_range_size / 2);
        const size_t end_index = (ranges.size() / 2) + (truncated_range_size / 2);
        return std::vector<float>(ranges.begin() + start_index, ranges.begin() + end_index);
    }

    unsigned int num_scans_for_width(double angle_increment, double r, double car_width)
    {
        // arc length of an angle increment ( (inc/2pi)*2pi*r) )
        // chord length of an angle increment 2* R * sin(inc/2))
        if (r < car_width * 0.1)
            return 0;
        double chord = 2 * r * sin(angle_increment / 2);
        double radiusd = car_width / chord;
        unsigned int count = (int)ceil(radiusd);
        return count;
    }

    std::vector<float> set_max(std::vector<float> vec, double max)
    {
        for (unsigned int i = 0; i < vec.size(); i++)
        {
            if (std::isinf(vec[i]))
            {
                double sum = 0;
                int count = 0, size = 2;
                int bound = i + size <= vec.size() ? i + size : vec.size();
                for (int f = i; f < bound; f++)
                {
                    if (!std::isinf(vec[f]))
                    {
                        count++;
                        sum += vec[f];
                    }
                }
                if (count > 0)
                {
                    vec[i] = sum / count;
                }
                else
                {
                    vec[i] = 0;
                }
            }
            if (std::isnan(vec[i]))
            {
                vec[i] = 0.0;
            }
            else if (vec[i] > max)
            {
                vec[i] = max;
            }
        }
        return vec;
    }

} // namespace cass

#endif //SRC_UTILITY_H
