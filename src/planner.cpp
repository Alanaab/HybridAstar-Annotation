#include "planner.h"

using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner() {
  // _____
  // TODOS
  //    initializeLookups();
  // Lookup::collisionLookup(collisionLookup);
  // ___________________
  // COLLISION DETECTION
  //    CollisionDetection configurationSpace;
  // _________________
  // TOPICS TO PUBLISH
  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);  // 用于发布起始点的箭头到rviz中

  // ___________________
  // TOPICS TO SUBSCRIBE
  if (Constants::manual) {
    subMap = n.subscribe("/map", 1, &Planner::setMap, this); // 接收 map_server 节点发布的静态图 /map 信息
  } else {
    subMap = n.subscribe("/occ_map", 1, &Planner::setMap, this); // 接收 动态图 /occ_map 信息
  }

  subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this); //接收目标的topic, "/move_base_simple/goal" 是 rviz里的 2D Nav Goal 绑定的topic
  subStart = n.subscribe("/initialpose", 1, &Planner::setStart, this); //接收起始点的topic, "/initialpose" 是 rviz里的 2D Pose Estimate 绑定的topic
}; 

//###################################################
//                                       LOOKUPTABLES
//###################################################
// 初始化查找表，注意 dubinsLookup 和 collisionLookup 都是inline函数，会在调用处内敛展开，但是这里没有被调用，不知道为什么也会执行？
void Planner::initializeLookups() {  
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);  
  }

  Lookup::collisionLookup(collisionLookup);
}

//###################################################
//                                                MAP
//###################################################
// nav_msgs::OccupancyGrid::Ptr 这个格式是 map_server 发布的静态地图的格式，map是一个值为0和100的一维栅格地图，0表示free，100表示障碍物
void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) { 
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }

  grid = map;
  //update the configuration space with the current map
  configurationSpace.updateGrid(map);
  //create array for Voronoi diagram
//  ros::Time t0 = ros::Time::now();
  int height = map->info.height;  // 
  int width = map->info.width;
  bool** binMap;  // 创建一个二维地图信息
  binMap = new bool*[width];

  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }
  // 这里是把一个二维栅格地图变成一个 二值化地图 true 或者 false 地图，区别在于binMap二维的，map是一维的
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      binMap[x][y] = map->data[y * width + x] ? true : false;  
    }
  }

  voronoiDiagram.initializeMap(width, height, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize();
//  ros::Time t1 = ros::Time::now();
//  ros::Duration d(t1 - t0);
//  std::cout << "created Voronoi Diagram in ms: " << d * 1000 << std::endl;

  // plan if the switch is not set to manual and a transform is available
  // 动态地图坐标之间的转换
  if (!Constants::manual && listener.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr)) {

    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

    // assign the values to start from base_link
    start.pose.pose.position.x = transform.getOrigin().x();
    start.pose.pose.position.y = transform.getOrigin().y();
    tf::quaternionTFToMsg(transform.getRotation(), start.pose.pose.orientation);

    if (grid->info.height >= start.pose.pose.position.y && start.pose.pose.position.y >= 0 &&
        grid->info.width >= start.pose.pose.position.x && start.pose.pose.position.x >= 0) {
      // set the start as valid and plan
      validStart = true;
    } else  {
      validStart = false;
    }

    plan();
  }
}

//###################################################
//                                   INITIALIZE START
//###################################################
// 初始化起始点信息
void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
  float x = initial->pose.pose.position.x / Constants::cellSize;
  float y = initial->pose.pose.position.y / Constants::cellSize;
  float t = tf::getYaw(initial->pose.pose.orientation);
  // publish the start without covariance for rviz
  // 把起始点的标记发布到相应的rvzi上去
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial->pose.pose.position;
  startN.pose.orientation = initial->pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validStart = true;
    start = *initial;

    if (Constants::manual) { plan();}  // 这里 setgoal和setstart都有，为了两个都执行完成后，能够执行plan

    // publish start for RViz
    pubStart.publish(startN);
  } else {
    std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
// 设置目标点信息
void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {
  // retrieving goal position
  float x = end->pose.position.x / Constants::cellSize;
  float y = end->pose.position.y / Constants::cellSize;
  float t = tf::getYaw(end->pose.orientation);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validGoal = true;
    goal = *end;

    if (Constants::manual) { plan();}

  } else {
    std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                      PLAN THE PATH
//###################################################
void Planner::plan() {
  // if a start as well as goal are defined go ahead and plan、
  // 如果起始点和目标点都有了，那么开始规划
  if (validStart && validGoal) {

    // ___________________________
    // LISTS ALLOWCATED ROW MAJOR ORDER
    int width = grid->info.width;  // 栅格地图的宽度
    int height = grid->info.height; // 栅格地图的长度
    int depth = Constants::headings;  // 栅格地图的深度，这里就是离散化的朝向角度的个数，72个，360/72=5 度，和论文的一致
    int length = width * height * depth;  // 整个三维地图的长度
    // define list pointers and initialize lists
    Node3D* nodes3D = new Node3D[length]();  // 三维节点，用于hybridastar
    Node2D* nodes2D = new Node2D[width * height](); // 二维节点，用于astar计算G值（即为完整约束的第二个代价）

    // ________________________
    // retrieving goal position
    float x = goal.pose.position.x / Constants::cellSize;
    float y = goal.pose.position.y / Constants::cellSize;
    float t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t); // 归一化偏航角，因为可能会出现超过 (0,2PI]的情况
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
    // __________
    // DEBUG GOAL
    //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);


    // _________________________
    // retrieving start position
    x = start.pose.pose.position.x / Constants::cellSize;
    y = start.pose.pose.position.y / Constants::cellSize;
    t = tf::getYaw(start.pose.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, nullptr);

    // ___________________________
    // START AND TIME THE PLANNING
    ros::Time t0 = ros::Time::now();

    // CLEAR THE VISUALIZATION
    visualization.clear();
    // CLEAR THE PATH
    path.clear();
    smoothedPath.clear(); 
    // FIND THE PATH
    // 开始规划！！！
    //核心步骤：
    // 1) 调用hybridAStar()函数获取一条路径
    // 2) 获取路径点(3D Node) -> 原始路径
    // 3) 对路径依据Voronoi图进行平滑->平滑路径

    // 返回最后一个节点
    Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization);
    // TRACE THE PATH
    smoother.tracePath(nSolution); // 根据最后一个节点，回溯提出所有的路径点
    // CREATE THE UPDATED PATH
    path.updatePath(smoother.getPath()); // 更新路径点，主要与rviz进行交互，显示线、路径点、车体等信息
    // SMOOTH THE PATH
    smoother.smoothPath(voronoiDiagram);  // 《 平滑路径，利用voronoiDiagra图 》
    // CREATE THE UPDATED PATH
    smoothedPath.updatePath(smoother.getPath());// 再一次更新路径点，主要与rviz进行交互，显示路径点、车体等信息
    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    std::cout << "TIME in ms: " << d * 1000 << std::endl;  // 计算规划时长

    // _________________________________
    // PUBLISH THE RESULTS OF THE SEARCH
    // 将结果发布到rviz上显示
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();
    smoothedPath.publishPath();
    smoothedPath.publishPathNodes();
    smoothedPath.publishPathVehicles();
    visualization.publishNode3DCosts(nodes3D, width, height, depth);
    visualization.publishNode2DCosts(nodes2D, width, height);


    delete [] nodes3D;
    delete [] nodes2D;

  } else {
    std::cout << "missing goal or start" << std::endl;
  }
}
