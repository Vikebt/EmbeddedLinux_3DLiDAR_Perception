# 面试证据索引：三维 LiDAR 感知

总讲义见 [模块化五项目面试讲义](https://github.com/Vikebt/EmbeddedLinux_IMX6ULL_ConditionMonitor/tree/main/docs/interview-handbook)，本项目重点对应 [进程线程](https://github.com/Vikebt/EmbeddedLinux_IMX6ULL_ConditionMonitor/blob/main/docs/interview-handbook/03-linux-process-thread.md)、[I/O 与网络](https://github.com/Vikebt/EmbeddedLinux_IMX6ULL_ConditionMonitor/blob/main/docs/interview-handbook/04-linux-io-network.md)、[P3 项目故事](https://github.com/Vikebt/EmbeddedLinux_IMX6ULL_ConditionMonitor/blob/main/docs/interview-handbook/06-project-stories.md#p3三维-lidar-感知) 与实验手册。固定证据标签为本仓库 `study-step-2-overload-policy`；总讲义固定标签为 `study-step-7-detailed-handbook`。

| 常见问题 | 代码证据 | 可以展开的回答 |
|---|---|---|
| `select/poll/epoll` 怎么选？ | `src/input.cc` | 单个雷达也可用 `poll`，这里保留 `epoll` 是为了统一未来多 socket 事件循环；使用非阻塞 socket 并处理 EINTR、超时和错误事件 |
| 文件描述符为什么会泄漏？ | `InputSocket` 构造/析构 | 构造中每个失败分支关闭已取得资源；析构分别关闭 epoll fd 和 socket fd，fd 用 `-1` 表示无所有权 |
| 有界队列满了怎么办？ | `core/bounded_queue.h` | 实时感知选择 DropOldest，保证采集有界延迟并暴露 dropped 指标；离线任务可选 Block |
| 条件变量为什么必须带谓词？ | `BoundedQueue::push/pop` | 防止虚假唤醒，并把 shutdown 纳入唤醒条件；关闭时 `notify_all` 解除所有等待者 |
| 线程怎么优雅退出？ | `PipelineStage::stop`、`PipelineController::stop` | atomic 运行标志 + queue shutdown + join；控制器按下游到上游顺序停止，避免生产者卡在已停止消费者前 |
| 什么是数据竞争？ | `PipelineStage::metrics_mutex_` | 处理线程写指标、ROS 定时器读指标，必须在同一互斥量下获取快照，不能靠普通结构体“碰巧可用” |
| PCAP 回放要检查什么？ | `InputPCAP::getPacket` | 检查截断帧长度、一次回放 EOF 语义、过滤器结果，并避免 `abort()` 破坏正常退出 |
| 点云帧怎样保留来源？ | `StampedPointCloud` 与增强驱动各 Stage | 传递字符串坐标系 ID 与递增帧序号；Marker/PointCloud2 使用输入帧 ID，不把输出写死为 `laser_link` |

## 验证边界

独立主机 Debug/Release 测试覆盖队列的丢旧保新、阻塞唤醒、关闭与复用，均 1/1 通过且设置 5 秒超时。全量 catkin 配置不再强制覆盖显式 Debug；但当前环境没有 ROS/PCL，增强驱动元数据修改尚未完成 ROS 编译验证。没有 LS1550 与 Jetson 时，不声称完成硬件吞吐、丢包率或温度功耗验证。
