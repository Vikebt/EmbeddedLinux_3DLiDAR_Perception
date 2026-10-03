#include <lslidar_ls_driver/core/bounded_queue.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

#define CHECK(expression) do { \
    if (!(expression)) { \
        std::cerr << "FAIL " << __FILE__ << ':' << __LINE__ << ": " #expression << '\n'; \
        return 1; \
    } \
} while (false)

using lslidar_ch_driver::BoundedQueue;
using lslidar_ch_driver::OverflowPolicy;

int main()
{
    BoundedQueue<int> latest(2, OverflowPolicy::DropOldest);
    CHECK(latest.push(1));
    CHECK(latest.push(2));
    CHECK(latest.push(3));
    CHECK(latest.dropped() == 1);
    int value = 0;
    CHECK(latest.pop(value, 0) && value == 2);
    CHECK(latest.pop(value, 0) && value == 3);

    BoundedQueue<int> blocking(1, OverflowPolicy::Block);
    std::atomic<bool> consumer_ok{false};
    std::thread consumer([&blocking, &consumer_ok] {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        int item = 0;
        consumer_ok.store(blocking.pop(item, 100) && item == 7);
    });
    const bool pushed_first = blocking.push(7);
    const bool pushed_second = blocking.push(8);
    consumer.join();
    CHECK(pushed_first);
    CHECK(pushed_second);
    CHECK(consumer_ok.load());
    CHECK(blocking.pop(value, 0) && value == 8);

    blocking.shutdown();
    CHECK(!blocking.push(9));
    blocking.reset();
    CHECK(blocking.push(10));
    return 0;
}
