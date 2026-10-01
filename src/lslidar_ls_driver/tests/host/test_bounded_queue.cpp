#include <lslidar_ls_driver/core/bounded_queue.h>

#include <cassert>
#include <chrono>
#include <thread>

using lslidar_ch_driver::BoundedQueue;
using lslidar_ch_driver::OverflowPolicy;

int main()
{
    BoundedQueue<int> latest(2, OverflowPolicy::DropOldest);
    assert(latest.push(1));
    assert(latest.push(2));
    assert(latest.push(3));
    assert(latest.dropped() == 1);
    int value = 0;
    assert(latest.pop(value, 0) && value == 2);
    assert(latest.pop(value, 0) && value == 3);

    BoundedQueue<int> blocking(1, OverflowPolicy::Block);
    std::thread consumer([&blocking] {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        int item = 0;
        assert(blocking.pop(item, 100));
        assert(item == 7);
    });
    assert(blocking.push(7));
    assert(blocking.push(8));
    consumer.join();
    assert(blocking.pop(value, 0) && value == 8);

    blocking.shutdown();
    assert(!blocking.push(9));
    blocking.reset();
    assert(blocking.push(10));
    return 0;
}
