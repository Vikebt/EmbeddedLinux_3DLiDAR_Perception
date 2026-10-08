/******************************************************************************
 * This file is part of lslidar_ls_driver.
 *
 * Copyright 2022 LeiShen Intelligent Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <lslidar_ls_driver/lslidar_ls_driver.h>

#include <atomic>
#include <chrono>
#include <thread>

volatile sig_atomic_t flag = 1;

static void my_handler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) {
        flag = 0;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lslidar_ls_driver_node",
              ros::init_options::NoSigintHandler);
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");
    private_nh.setParam("ready", false);

    signal(SIGINT, my_handler);
    signal(SIGTERM, my_handler);

    std::atomic<bool> stop_watcher{false};
    std::thread shutdown_watcher([&stop_watcher] {
        while (!stop_watcher.load() && flag && ros::ok()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (!flag) {
            ros::shutdown();
        }
    });

    // start the driver
    ROS_INFO("namespace is %s", private_nh.getNamespace().c_str());
    lslidar_ch_driver::LslidarChDriver driver(node, private_nh);
    if (!driver.initialize()) {
        const int exit_code = flag ? 1 : 0;
        if (flag) {
            ROS_ERROR("Cannot initialize lslidar driver...");
        }
        stop_watcher.store(true);
        shutdown_watcher.join();
        return exit_code;
    }
    private_nh.setParam("ready", true);
    // loop until shut down or end of file
    while (ros::ok() && flag && driver.polling()) {
        ros::spinOnce();
    }
    private_nh.setParam("ready", false);
    ros::shutdown();
    stop_watcher.store(true);
    shutdown_watcher.join();
    return 0;
}
