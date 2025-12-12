/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <thread>
#include <mutex>
#include <queue>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>

#include <System.h>
#include "ImuTypes.h"

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

// Use nlohmann/json for JSON parsing
// Download from: https://github.com/nlohmann/json
// Or install: sudo apt-get install nlohmann-json3-dev
#include <nlohmann/json.hpp>

typedef websocketpp::server<websocketpp::config::asio> server;

using namespace std;
using json = nlohmann::json;

// Shared queue for IMU data
queue<ORB_SLAM3::IMU::Point> imu_queue;
mutex imu_mutex;

// Time synchronization offset (calculated at startup)
static double time_offset = -1.0;
static mutex time_offset_mutex;

// WebSocket message handler - NOW PARSES JSON FORMAT
void on_message(server* s, websocketpp::connection_hdl hdl, server::message_ptr msg) {
    try {
        // Parse JSON payload
        json j = json::parse(msg->get_payload());
        
        // Extract accelerometer values (m/s²)
        auto acc_values = j["imu"]["Samsung Linear Acceleration Sensor"]["values"];
        float acc_x = acc_values[0].get<float>();
        float acc_y = acc_values[1].get<float>();
        float acc_z = acc_values[2].get<float>();
        
        // Extract gyroscope values
        auto gyro_values = j["imu"]["ICM42632M Gyroscope"]["values"];
        float gyro_x = gyro_values[0].get<float>();
        float gyro_y = gyro_values[1].get<float>();
        float gyro_z = gyro_values[2].get<float>();
        
        // Extract timestamp (Unix epoch time in seconds)
        double timestamp = j["timestamp"].get<double>();
        
        // *** CRITICAL: Convert gyroscope from deg/s to rad/s if needed ***
        // Most Android sensors output gyro in rad/s already, but verify!
        // Uncomment these lines if your gyro is in degrees/s:
        // gyro_x *= M_PI / 180.0;
        // gyro_y *= M_PI / 180.0;
        // gyro_z *= M_PI / 180.0;
        
        // Convert Unix timestamp to steady_clock-compatible time
        {
            lock_guard<mutex> time_lock(time_offset_mutex);
            if (time_offset < 0) {
                // First IMU message - calculate offset
                double steady_time = chrono::duration_cast<chrono::duration<double>>(
                    chrono::steady_clock::now().time_since_epoch()).count();
                time_offset = timestamp - steady_time;
                cout << "Time offset calculated: " << fixed << time_offset << " seconds" << endl;
            }
        }
        
        // Convert to steady_clock time base
        double timestamp_steady = timestamp - time_offset;
        
        // Debug output
        // cout << "IMU - Time: " << fixed << timestamp_steady 
        //      << ", Acc: [" << acc_x << ", " << acc_y << ", " << acc_z 
        //      << "], Gyro: [" << gyro_x << ", " << gyro_y << ", " << gyro_z << "]" << endl;
        
        // Push to queue with correct constructor:
        // Point(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, timestamp)
        lock_guard<mutex> lock(imu_mutex);
        imu_queue.push(ORB_SLAM3::IMU::Point(
            acc_x, acc_y, acc_z,           // Acceleration
            gyro_x, gyro_y, gyro_z,        // Angular velocity
            timestamp_steady               // Synchronized timestamp
        ));
        
    } catch (json::parse_error& e) {
        cerr << "JSON parse error: " << e.what() << endl;
        cerr << "Message was: " << msg->get_payload() << endl;
    } catch (json::type_error& e) {
        cerr << "JSON type error: " << e.what() << endl;
    } catch (json::out_of_range& e) {
        cerr << "JSON out of range: " << e.what() << endl;
    } catch (exception& e) {
        cerr << "Exception in on_message: " << e.what() << endl;
    }
}

// WebSocket server function
void run_server() {
    const int IMU_WEBSOCKET_PORT = 8001;
    server echo_server;

    try {
        echo_server.set_access_channels(websocketpp::log::alevel::app);
        echo_server.clear_access_channels(websocketpp::log::alevel::frame_payload);
        echo_server.init_asio();
        echo_server.set_message_handler(bind(&on_message, &echo_server, placeholders::_1, placeholders::_2));
        echo_server.listen(IMU_WEBSOCKET_PORT);
        echo_server.start_accept();
        cout << "IMU echo server running on ws://0.0.0.0:" << IMU_WEBSOCKET_PORT << endl;
        echo_server.run();
    } catch (websocketpp::exception const & e) {
        cerr << "WebSocket++ exception: " << e.what() << endl;
    } catch (...) {
        cerr << "Other exception in WebSocket server" << endl;
    }
}

int main(int argc, char *argv[])
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_inertial_live path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // Start the WebSocket server in a separate thread
    thread server_thread(run_server);

    // Wait for first IMU message to establish time offset
    cout << "Waiting for first IMU message to synchronize time..." << endl;
    while (time_offset < 0) {
        this_thread::sleep_for(chrono::milliseconds(100));
    }
    cout << "Time synchronized. Starting SLAM system..." << endl;

    // Create SLAM system
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    // Webcam setup (camera index 2 as per your original code)
    cv::VideoCapture cap(2);
    if (!cap.isOpened()) {
        cerr << "ERROR: Could not open camera 2" << endl;
        return 1;
    }

    // Set camera properties if needed (optional)
    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 752);
    // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    // cap.set(cv::CAP_PROP_FPS, 20);

    // Main loop
    cv::Mat im;
    double last_tframe = -1.0;
    int frame_count = 0;
    
    while (true)
    {
        cap >> im;
        if (im.empty()) {
            cerr << "ERROR: Failed to capture image from camera" << endl;
            break;
        }

        // Use steady_clock for frame timestamp (matches converted IMU timestamps)
        double tframe = chrono::duration_cast<chrono::duration<double>>(
            chrono::steady_clock::now().time_since_epoch()).count();

        if(last_tframe < 0.0)
        {
            last_tframe = tframe;
            continue;
        }

        if (imageScale != 1.f) {
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
        }

        // Collect IMU measurements between last frame and current frame
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        {
            lock_guard<mutex> lock(imu_mutex);
            while(!imu_queue.empty() && imu_queue.front().t <= tframe)
            {
                if(imu_queue.front().t >= last_tframe)
                {
                    vImuMeas.push_back(imu_queue.front());
                }
                imu_queue.pop();
            }
        }
        
        // Sort IMU measurements by timestamp (should already be sorted, but ensure)
        sort(vImuMeas.begin(), vImuMeas.end(), [](const ORB_SLAM3::IMU::Point &a, const ORB_SLAM3::IMU::Point &b) {
            return a.t < b.t;
        });

        // *** DIAGNOSTIC: Check IMU measurement count ***
        frame_count++;
        if (frame_count % 20 == 0) {  // Print every 20 frames
            cout << "Frame " << frame_count << ": " << vImuMeas.size() 
                 << " IMU measurements collected" << endl;
            
            if (vImuMeas.size() < 5) {
                cout << "WARNING: Low IMU frequency! Expected ~10-20 measurements per frame." << endl;
            }
            
            // Show IMU queue size
            lock_guard<mutex> lock(imu_mutex);
            cout << "IMU queue size: " << imu_queue.size() << endl;
        }

        // Track monocular with IMU
        SLAM.TrackMonocular(im, tframe, vImuMeas);

        last_tframe = tframe;
    }

    // Stop all threads
    cout << "Shutting down SLAM system..." << endl;
    SLAM.Shutdown();
    
    cout << "Stopping WebSocket server..." << endl;
    server_thread.join();

    // Save trajectory
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    
    cout << "Trajectories saved. Exiting." << endl;

    return 0;
}
