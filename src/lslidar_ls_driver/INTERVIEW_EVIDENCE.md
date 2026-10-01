# 面试证据索引：三维 LiDAR 感知

| 常见问题 | 代码证据 | 可以展开的回答 |
|---|---|---|
| `select/poll/epoll` 怎么选？ | `src/input.cc` | 单个雷达也可用 `poll`，这里保留 `epoll` 是为了统一未来多 socket 事件循环；使用非阻塞 socket 并处理 EINTR、超时和错误事件 |
| 文件描述符为什么会泄漏？ | `InputSocket` 构造/析构 | 构造中每个失败分支关闭已取得资源；析构分别关闭 epoll fd 和 socket fd，fd 用 `-1` 表示无所有权 |
| 有界队列满了怎么办？ | `core/bounded_queue.h` | 实时感知选择 DropOldest，保证采集有界延迟并暴露 dropped 指标；离线任务可选 Block |
| 条件变量为什么必须带谓词？ | `BoundedQueue::push/pop` | 防止虚假唤醒，并把 shutdown 纳入唤醒条件；关闭时 `notify_all` 解除所有等待者 |
| 线程怎么优雅退出？ | `PipelineStage::stop`、`PipelineController::stop` | atomic 运行标志 + queue shutdown + join；控制器按下游到上游顺序停止，避免生产者卡在已停止消费者前 |
| 什么是数据竞争？ | `PipelineStage::metrics_mutex_` | 处理线程写指标、ROS 定时器读指标，必须在同一互斥量下获取快照，不能靠普通结构体“碰巧可用” |
| PCAP 回放要检查什么？ | `InputPCAP::getPacket` | 检查截断帧长度、一次回放 EOF 语义、过滤器结果，并避免 `abort()` 破坏正常退出 |

## 验证边界

独立主机测试覆盖队列的丢旧保新、阻塞唤醒、关闭与复用。完整 ROS/PCL 构建需要对应 ROS1 环境；没有 LS1550 与 Jetson 时，不声称完成硬件吞吐、丢包率或温度功耗验证。
