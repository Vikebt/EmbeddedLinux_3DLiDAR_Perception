#include <gtest/gtest.h>

#include <lslidar_ls_driver/lslidar_ls_driver.h>

#include <csignal>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <thread>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

// The input library refers to the executable's signal flag, as the existing
// driver nodes do. The active-thread test uses only loopback UDP.
volatile sig_atomic_t flag = 1;

TEST(DriverLifecycle, UninitializedDriverCanBeDestroyed) {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    {
        lslidar_ch_driver::LslidarChDriver driver(nh, private_nh);
    }
}

TEST(DriverLifecycle, ActiveDifopThreadCanBeDestroyedWithoutGlobalRosShutdown) {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    int probe[2] = {::socket(AF_INET, SOCK_DGRAM, 0),
                    ::socket(AF_INET, SOCK_DGRAM, 0)};
    ASSERT_GE(probe[0], 0);
    ASSERT_GE(probe[1], 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ASSERT_EQ(::bind(probe[0], reinterpret_cast<sockaddr*>(&address), sizeof(address)), 0);
    ASSERT_EQ(::bind(probe[1], reinterpret_cast<sockaddr*>(&address), sizeof(address)), 0);
    socklen_t address_size = sizeof(address);
    ASSERT_EQ(::getsockname(probe[0], reinterpret_cast<sockaddr*>(&address), &address_size), 0);
    const int msop_port = ntohs(address.sin_port);
    address_size = sizeof(address);
    ASSERT_EQ(::getsockname(probe[1], reinterpret_cast<sockaddr*>(&address), &address_size), 0);
    const int difop_port = ntohs(address.sin_port);
    ::close(probe[0]);
    ::close(probe[1]);

    private_nh.setParam("lidar_ip", "127.0.0.1");
    private_nh.setParam("msop_port", msop_port);
    private_nh.setParam("difop_port", difop_port);
    private_nh.setParam("add_multicast", false);

    const int sender_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    ASSERT_GE(sender_fd, 0);
    std::atomic<bool> stop_sender{false};
    std::thread sender([&]() {
        sockaddr_in destination{};
        destination.sin_family = AF_INET;
        destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        destination.sin_port = htons(static_cast<uint16_t>(msop_port));
        unsigned char packet[1206]{};
        packet[1205] = 0x01;  // recognized single-echo mode
        while (!stop_sender.load()) {
            ::sendto(sender_fd, packet, sizeof(packet), 0,
                     reinterpret_cast<sockaddr*>(&destination), sizeof(destination));
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        ::close(sender_fd);
    });

    std::unique_ptr<lslidar_ch_driver::LslidarChDriver> driver(
        new lslidar_ch_driver::LslidarChDriver(nh, private_nh));
    const bool initialized = driver->initialize();
    stop_sender = true;
    sender.join();
    ASSERT_TRUE(initialized);

    std::atomic<bool> destroyed{false};
    std::thread destroyer([&]() {
        driver.reset();
        destroyed = true;
    });
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(6);
    while (!destroyed.load() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!destroyed.load()) {
        // Avoid leaving a hung test process behind if join cannot finish.
        std::fprintf(stderr, "DIFOP destructor did not finish within 6 seconds\n");
        std::_Exit(2);
    }
    destroyer.join();
    EXPECT_TRUE(ros::ok());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::M_string remappings;
    ros::init(remappings, "driver_lifecycle_test");
    const int result = RUN_ALL_TESTS();
    ros::shutdown();
    return result;
}
