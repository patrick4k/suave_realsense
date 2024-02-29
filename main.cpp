#include <iostream>
#include <librealsense2/rs.hpp>
#include <chrono>
#include <thread>
#include <mutex>

bool check_imu_is_supported()
{
    bool found_gyro = false;
    bool found_accel = false;
    rs2::context ctx;
    for (auto dev : ctx.query_devices())
    {
        // The same device should support gyro and accel
        found_gyro = false;
        found_accel = false;
        for (const auto& sensor : dev.query_sensors())
        {
            for (const auto& profile : sensor.get_stream_profiles())
            {
                if (profile.stream_type() == RS2_STREAM_GYRO)
                    found_gyro = true;

                if (profile.stream_type() == RS2_STREAM_ACCEL)
                    found_accel = true;
            }
        }
        if (found_gyro && found_accel)
            break;
    }
    return found_gyro && found_accel;
}

constexpr bool GYRO = true;
constexpr bool ACCEL = true;
constexpr bool DEPTH = true;

int main(int, char *[]) try
{
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Before running the example, check that a device supporting IMU is connected
    if (!check_imu_is_supported())
    {
        std::cerr << "Device supporting IMU not found";
        return EXIT_FAILURE;
    }

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Add streams of gyro and accelerometer to configuration
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16);

    // Start streaming with the given configuration;
    // Note that since we only allow IMU streams, only single frames are produced
    auto profile = pipe.start(cfg, [&](rs2::frame frame)
    {
        // Cast to motion and frameset
        auto motion = frame.as<rs2::motion_frame>();
        auto frames = frame.as<rs2::frameset>();

        // If casting succeeded and the arrived frame is from gyro stream
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
        {
            // Get gyro measures
            rs2_vector gyro_data = motion.get_motion_data();

            // PRINT GYRO HERE
            if (GYRO)
                std::cout << "GYRO: " << gyro_data.x << ", " << gyro_data.y << ", " << gyro_data.z << std::endl;
        }

        // If casting succeeded and the arrived frame is from accelerometer stream
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
        {
            // Get accelerometer measures
            rs2_vector accel_data = motion.get_motion_data();

            // PRINT ACCEL HERE
            if (ACCEL)
                std::cout << "ACCEL: " << accel_data.x << ", " << accel_data.y << ", " << accel_data.z << std::endl;
        }

        if (frames) {
            const auto depth = frames.get_depth_frame();
            const auto width = depth.get_width();
            const auto height = depth.get_height();
            const auto dist_to_center = depth.get_distance(width / 2, height / 2);
            if (DEPTH)
                std::cout << "DEPTH (" << width << ", " << height << "): " << dist_to_center << std::endl;
        }
    });

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    while (true)
        std::this_thread::sleep_for(std::chrono::minutes(1));
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    return EXIT_FAILURE;
}
