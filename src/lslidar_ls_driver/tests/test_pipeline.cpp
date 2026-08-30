#include <gtest/gtest.h>
#include <lslidar_ls_driver/core/pipeline.h>

using namespace lslidar_ch_driver;

// 测试用的简单Stage
class TestStage : public PipelineStage {
public:
    TestStage() : PipelineStage("TestStage", 8) {}
    int process_count_{0};

protected:
    StampedPointCloud::Ptr process(StampedPointCloud::Ptr data) override {
        process_count_++;
        return data;
    }
};

TEST(PipelineTest, StageStartStop) {
    TestStage stage;
    EXPECT_TRUE(stage.start());
    EXPECT_TRUE(stage.start());  // 重复启动应该安全
    stage.stop();
    stage.stop();  // 重复停止应该安全
    SUCCEED();
}

TEST(PipelineTest, ThreadSafeQueue) {
    ThreadSafeQueue<int> queue(4);
    EXPECT_TRUE(queue.push(1));
    EXPECT_TRUE(queue.push(2));

    int val;
    EXPECT_TRUE(queue.pop(val, 100));
    EXPECT_EQ(val, 1);
    EXPECT_TRUE(queue.pop(val, 100));
    EXPECT_EQ(val, 2);

    queue.shutdown();
    EXPECT_FALSE(queue.push(3));
}

TEST(PipelineTest, PipelineMetrics) {
    PipelineMetrics metrics{"test"};
    metrics.update(10.5);
    metrics.update(20.3);
    EXPECT_GT(metrics.total_processed, 0);
    EXPECT_GT(metrics.avg_process_time_ms, 0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
