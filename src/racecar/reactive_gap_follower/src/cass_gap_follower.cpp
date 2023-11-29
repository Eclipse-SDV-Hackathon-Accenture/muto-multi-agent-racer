#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include "math.h"
#include "utility/utility.hpp"
#include <memory>

// #define PI 3.1415927

using namespace cass;
class GapFollower : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr disp_pub_;

    double angle, headingRange;
    double forward_view_angle, scanner_forward_offsetangle;
    double disparity_threshold, wall_distance_threshold;
    bool disparity_publish;
    double car_width;
    std::string carname = "racecar1";
    int smoothing_filter_size, disparity_filter_size;
    std::map<std::string, double> ErrorBasedVelocities;
    std::map<std::string, double> ErrorBasedRanges;
    std::vector<float> ranges;

public:
    GapFollower() : Node("cass_gap_follower1")
    {


        this->declare_parameter<std::string>("racecar_namespace", "racecar1");
        carname = this->get_parameter("racecar_namespace").as_string();
        std::string racecar_namespace = "/" + carname;
        // RCLCPP_INFO(this->get_logger(), "GAP FOLLOWER FOR CAR: " + carname);

        this->declare_parameter<std::string>("scan_topic", "scan");
        this->declare_parameter<std::string>("drive_topic", "drive");

        std::string scan_topic = racecar_namespace + "/" + this->get_parameter("scan_topic").as_string();
        std::string drive_topic = racecar_namespace + "/" + this->get_parameter("drive_topic").as_string();

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, 10,
                                                                            std::bind(&GapFollower::laserScanCallback, this, std::placeholders::_1));
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

        this->declare_parameter<double>("error_based_velocities.low", 0.5);
        this->declare_parameter<double>("error_based_velocities.medium", 1.0);
        this->declare_parameter<double>("error_based_velocities.high", 1.5);
        this->declare_parameter<double>("error_based_ranges.low", 1.0);
        this->declare_parameter<double>("error_based_ranges.medium", 3.0);
        this->declare_parameter<double>("error_based_ranges.high", 400.0);

        this->declare_parameter<double>("forward_view_angle", 0.0);
        this->declare_parameter<double>("scanner_forward_offsetangle", 0.0);
        this->declare_parameter<double>("disparity_threshold", 0.0);
        this->declare_parameter<int>("disparity_filter_size", 0);
        this->declare_parameter<bool>("disparity_publish", false);
        this->declare_parameter<double>("wall_distance_threshold", 0.0);
        this->declare_parameter<double>("car_width", 0.0);
        this->declare_parameter<int>("smoothing_filter_size", 0);
   
        this->get_parameter("forward_view_angle", forward_view_angle);
        this->get_parameter("scanner_forward_offsetangle", scanner_forward_offsetangle);
        this->get_parameter("disparity_threshold", disparity_threshold);
        this->get_parameter("disparity_filter_size", disparity_filter_size);
        this->get_parameter("disparity_publish", disparity_publish);
        this->get_parameter("wall_distance_threshold", wall_distance_threshold);
        this->get_parameter("car_width", car_width);
        this->get_parameter("smoothing_filter_size", smoothing_filter_size);

        if (disparity_publish)
        {
            disp_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("disparities", 10);
        }
    }

    void laserScanCallback(sensor_msgs::msg::LaserScan::SharedPtr lidar_info)
    {
        this->preprocess_lidar(*lidar_info);
        this->reactive_control();
    }

    void preprocess_lidar(sensor_msgs::msg::LaserScan &lidar_info)
    {

        // RPI LIDAR IS DIFFERENT THAN HOKUYO - IVER=TRUE SHIFT BY 180 DEGREES
        std::vector<float> rotated = lidar_info.ranges;
        int offset = (int)std::floor(scanner_forward_offsetangle / lidar_info.angle_increment);
        if (offset > 0)
            std::rotate(rotated.begin(), rotated.begin() + offset, rotated.end());

        ranges = rotated;
        ranges = truncate(ranges, lidar_info.angle_max, lidar_info.angle_min, forward_view_angle);
        if (smoothing_filter_size > 1)
            ranges = apply_smoothing_filter(ranges, smoothing_filter_size);

        ranges = set_max(ranges, lidar_info.range_max);

        std::vector<unsigned int> disparity_indices = find_disparity_indices();
        std::vector<float> adjusted_range = extend_disparities(disparity_indices, lidar_info.angle_increment);

        sensor_msgs::msg::LaserScan disparityScan = lidar_info;

        std::size_t max_start = 0, max_end = 0;
        headingRange = 0.0;

        for (unsigned int i = 0; i < adjusted_range.size(); i++)
        {
            if (adjusted_range[i] >= headingRange)
            {
                headingRange = adjusted_range[i];
                max_end = i;
            }
        }
        for (std::size_t i = 0; i < adjusted_range.size(); i++)
        {
            if (std::fabs(adjusted_range[i] - headingRange) < disparity_threshold / 10.0)
            {
                max_start = i;
                break;
            }
        }
        double min_angle = (-1 * forward_view_angle / 2.0);
        angle = min_angle + (max_start + max_end) / 2.0 * lidar_info.angle_increment;

        if (disparity_publish)
        {
            sensor_msgs::msg::LaserScan disp = lidar_info;
            disp.ranges = adjusted_range;
            disp.angle_max = forward_view_angle / 2.0 + scanner_forward_offsetangle;
            disp.angle_min = -forward_view_angle / 2.0 + scanner_forward_offsetangle;
            disp_pub_;
        }
    }

    std::vector<unsigned int> find_disparity_indices()
    {
        std::vector<unsigned int> disparity_indices;
        unsigned int width = disparity_filter_size;

        for (unsigned int i = 0; i < ranges.size() - width; i++)
        {
            double diff = 0;
            for (std::size_t r = i + 1; (r <= i + width && r < ranges.size()); r++)
                diff += abs(ranges[r] - ranges[r - 1]);
            if (diff >= disparity_threshold * width)
            {
                disparity_indices.push_back(i);
            }
        }
        return disparity_indices;
    }

    std::vector<float> extend_disparities(std::vector<unsigned int> disparity_indices, double angle_increment)
    {
        std::vector<float> disparity_ranges = ranges;

        for (std::size_t i = 0; i < disparity_indices.size(); i++)
        {
            int di = disparity_indices[i];
            double value = disparity_ranges[di];
            double nextValue = disparity_ranges[di + 1];

            if (value < nextValue)
            {
                int rwidth = num_scans_for_width(angle_increment, value, car_width);

                int upper_bound = std::min(di + rwidth, (int)ranges.size());
                for (int j = di; j < upper_bound; j++)
                {
                    if (disparity_ranges[j] > value)
                        disparity_ranges[j] = value;
                }
            }
            else if (std::abs(value - nextValue) > 0.005)
            {
                int rwidth = num_scans_for_width(angle_increment, nextValue, car_width);
                int low_bound = std::max(di - rwidth, 0);
                for (int j = di; j > low_bound; j--)
                {
                    if (disparity_ranges[j] > nextValue)
                        disparity_ranges[j] = nextValue;
                }
            }
        }
        return disparity_ranges;
    }
    void reactive_control()
    {
        this->get_parameter("error_based_velocities.low", ErrorBasedVelocities["low"]);
        this->get_parameter("error_based_velocities.medium", ErrorBasedVelocities["medium"]);
        this->get_parameter("error_based_velocities.high", ErrorBasedVelocities["high"]);
        this->get_parameter("error_based_ranges.low", ErrorBasedRanges["low"]);
        this->get_parameter("error_based_ranges.medium", ErrorBasedRanges["medium"]);
        this->get_parameter("error_based_ranges.high", ErrorBasedRanges["high"]);
        ackermann_msgs::msg::AckermannDriveStamped ackermann_drive_result;
        ackermann_drive_result.drive.steering_angle = angle;
        ackermann_drive_result.header.frame_id = carname + "/base_link";

        if (headingRange < ErrorBasedRanges["low"])
        {
            ackermann_drive_result.drive.speed = ErrorBasedVelocities["low"];
        }
        else if (headingRange < ErrorBasedRanges["medium"])
        {
            ackermann_drive_result.drive.speed = ErrorBasedVelocities["medium"];
        }
        else
        {
            ackermann_drive_result.drive.speed = ErrorBasedVelocities["high"];
        }
        drive_pub_->publish(ackermann_drive_result);
        // RCLCPP_INFO(this->get_logger(), "Reactive steering angle: %f", angle);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GapFollower>());
    rclcpp::shutdown();
    return 0;
}
