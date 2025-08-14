代码执行主流程（从进程启动到路径发布）：

ROS2Node 构造函数内部  
- 读取/声明参数 → 解析 mapConfig → TRGPlanner::setParams(配置 YAML)  
- 建立 tf2 Buffer/Listener  
- 建立订阅：egoPose / egoOdom / obsCloud / goal  
- 建立发布：preMap / outGoal / path 及 debug 四类  
- 调用 TRGPlanner::init():  
  a. 构造 TRG (图结构与规划算法)  
  b. 若配置启用预构建地图：加载 PCD → (可体素滤波) → setGlobalMap  
  c. 分配 obsPtr  
  d. 启动两个核心线程：runGraphFSM(), runPlanningFSM()  
- 启动本类线程：publishTimer（必定）与 debugTimer（若 isDebug=true）

线程与回调组成的并发结构  
(1) Executor 线程：驱动四个订阅回调（cbPose / cbOdom / cbCloud / cbGoal）  
(2) TRGPlanner::runGraphFSM 线程：按 graph_rate 周期运行图构建 / 更新有限状态机  
(3) TRGPlanner::runPlanningFSM 线程：按 planning_rate 周期运行路径规划有限状态机  
(4) ROS2Node::publishTimer 线程：按 publish_rate 发布地图 / 路径 / 目标点 / 路径信息  
(5) ROS2Node::debugTimer 线程：按 debug_rate 发布调试可视化（全局/局部图、障碍点云）
