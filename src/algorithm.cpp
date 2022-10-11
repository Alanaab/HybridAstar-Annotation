#include "algorithm.h"

#include <boost/heap/binomial_heap.hpp>

using namespace HybridAStar;

float aStar(Node2D& start, Node2D& goal, Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization);
void updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization);
Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace);

//###################################################
//                                    NODE COMPARISON
//###################################################
/*!
   \brief A structure to sort nodes in a heap structure
*/
// 比较结构体，这是定义最小堆的排序用的，用到了3D节点和2D节点的排序，代价最小的在最前面
struct CompareNodes {
  /// Sorting 3D nodes by increasing C value - the total estimated cost
  bool operator()(const Node3D* lhs, const Node3D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
  /// Sorting 2D nodes by increasing C value - the total estimated cost
  bool operator()(const Node2D* lhs, const Node2D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
};

//###################################################
//                                        3D A*
//###################################################
Node3D* Algorithm::hybridAStar(Node3D& start,  // 起点
                               const Node3D& goal, // 终点
                               Node3D* nodes3D,  // 三维节点
                               Node2D* nodes2D,  // 二维节点
                               int width,  // 栅格宽度
                               int height,  // 栅格长度
                               CollisionDetection& configurationSpace,  // 构型空间
                               float* dubinsLookup,  // dubins曲线表
                               Visualize& visualization) { // 可视化

  // PREDECESSOR AND SUCCESSOR INDEX
  // 定义当前节点的index和下一个节点的index
  int iPred, iSucc; 
  float newG;
  // Number of possible directions, 3 for forward driving and an additional 3 for reversing
  int dir = Constants::reverse ? 6 : 3;  // 包含倒车的话，就是有6个方向，前后各三个
  // Number of iterations the algorithm has run for stopping based on Constants::iterations
  int iterations = 0; // 迭代次数

  // VISUALIZATION DELAY
  ros::Duration d(0.003);

  // OPEN LIST AS BOOST IMPLEMENTATION
  // 定义优先级队列 O
  typedef boost::heap::binomial_heap< Node3D*, boost::heap::compare<CompareNodes>> priorityQueue;
  priorityQueue O;

  // update h value
  // 更新启发代价，这里根据论文主要有两个代价
  updateH(start, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);
  // mark start as open
  start.open();
  // push on priority queue aka open list
  O.push(&start);
  iPred = start.setIdx(width, height); // 这个index计算的时候考虑到了heading，三维的index
  nodes3D[iPred] = start;

  // NODE POINTER
  Node3D* nPred;
  Node3D* nSucc;

  // float max = 0.f;

  // continue until O empty
  // 队列非空，一直执行下去
  while (!O.empty()) {

    // pop node with lowest cost from priority queue
    nPred = O.top(); // 弹出代价最小的节点
    // set index
    iPred = nPred->setIdx(width, height);  // 计算当前节点的index
    iterations++;  // 迭代次数增加

    // RViz visualization
    // Rviz 显示过程
    if (Constants::visualization) {
      visualization.publishNode3DPoses(*nPred);
      visualization.publishNode3DPose(*nPred);
      d.sleep();
    }

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    // 如果当前节点是已经拓展过的，则退出，继续下一个节点
    if (nodes3D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    // 如果没有拓展过，那么进行拓展
    else if (nodes3D[iPred].isOpen()) {
      // add node to closed list
      nodes3D[iPred].close();  // 先标记为拓展过
      // remove node from open list 
      O.pop(); // 节点弹出队列

      // _________
      // GOAL TEST
      // 首先判断是否到达目标点或者迭代次数是否已经很大了
      if (*nPred == goal || iterations > Constants::iterations) {
        // DEBUG
        return nPred;
      }

      // ____________________
      // CONTINUE WITH SEARCH
      else {
        // _______________________
        // SEARCH WITH DUBINS SHOT
        // 如果没到目标点，首先来判断是否离目标点很近了，能不能将路径直接解析算出来 dubin shot
        if (Constants::dubinsShot && nPred->isInRange(goal) && nPred->getPrim() < 3) {
          nSucc = dubinsShot(*nPred, goal, configurationSpace);

          if (nSucc != nullptr && *nSucc == goal) {
            //DEBUG
            // std::cout << "max diff " << max << std::endl;
            return nSucc;  // 如果可以，那么直接返回最后一个节点
          }
        }
     
        // ______________________________
        // SEARCH WITH FORWARD SIMULATION
        // 还不能直接解析算出，则开始向各个可能的方向拓展节点
        for (int i = 0; i < dir; i++) {
          // create possible successor
          // 计算邻节点，这个节点是人为设定的，可以自行根据机器人修改
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc->setIdx(width, height);

          // ensure successor is on grid and traversable
          // 确保该节点在规划的范围内，并且确保它不会碰到障碍物
          if (nSucc->isOnGrid(width, height) && configurationSpace.isTraversable(nSucc)) {

            // ensure successor is not on closed list or it has the same index as the predecessor
            // 如果该节点没有被拓展过，或者和当前节点在同一个cell里面，继续执行，否则删除该节点，退出
            if (!nodes3D[iSucc].isClosed() || iPred == iSucc) {

              // calculate new G value
              nSucc->updateG();  // 更新G值
              newG = nSucc->getG();

              // if successor not on open list or found a shorter way to the cell
              // 如果该节点本身不在open集合，并且cost-to-far比目前的小，或者和当前节点在同一个cell里面时继续执行
              if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc) {

                // calculate H value
                // 更新该节点的 启发式代价
                updateH(*nSucc, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);

                // if the successor is in the same cell but the C value is larger
                // 如果和当前节点在同一个cell里面，并且总代价C 比较大，忽略
                if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) {
                  delete nSucc;
                  continue;
                }
                // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                // 如果和当前节点在同一个cell里面，并且总代价C 比较小，则采纳，并把当前节点的父亲，设为该节点的父亲
                else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker) {
                  nSucc->setPred(nPred->getPred());
                }

                if (nSucc->getPred() == nSucc) {  // 循环？？？ 怎么会发生这个情况呢？
                  std::cout << "looping";
                }

                // put successor on open list
                nSucc->open();
                nodes3D[iSucc] = *nSucc;
                O.push(&nodes3D[iSucc]);
                delete nSucc;
              } else { delete nSucc; }
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  if (O.empty()) {
    return nullptr;
  }

  return nullptr;
}


//###################################################
//                                        2D A*
//###################################################
float aStar(Node2D& start,
            Node2D& goal,
            Node2D* nodes2D,
            int width,
            int height,
            CollisionDetection& configurationSpace,
            Visualize& visualization) {

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;

  // reset the open and closed list
  for (int i = 0; i < width * height; ++i) {
    nodes2D[i].reset();
  }

  // VISUALIZATION DELAY
  ros::Duration d(0.001);

  boost::heap::binomial_heap<Node2D*,
        boost::heap::compare<CompareNodes>> O;
  // update h value
  start.updateH(goal);
  // mark start as open
  start.open();
  // push on priority queue
  O.push(&start);
  iPred = start.setIdx(width);
  nodes2D[iPred] = start;

  // NODE POINTER
  Node2D* nPred;
  Node2D* nSucc;

  // continue until O empty
  while (!O.empty()) {
    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(width);

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes2D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes2D[iPred].isOpen()) {
      // add node to closed list
      nodes2D[iPred].close();
      nodes2D[iPred].discover();

      // RViz visualization
      if (Constants::visualization2D) {
        visualization.publishNode2DPoses(*nPred);
        visualization.publishNode2DPose(*nPred);
        //        d.sleep();
      }

      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST
      if (*nPred == goal) {
        return nPred->getG();
      }
      // ____________________
      // CONTINUE WITH SEARCH
      else {
        // _______________________________
        // CREATE POSSIBLE SUCCESSOR NODES
        for (int i = 0; i < Node2D::dir; i++) {
          // create possible successor
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc->setIdx(width);

          // ensure successor is on grid ROW MAJOR
          // ensure successor is not blocked by obstacle
          // ensure successor is not on closed list
          if (nSucc->isOnGrid(width, height) &&  configurationSpace.isTraversable(nSucc) && !nodes2D[iSucc].isClosed()) {
            // calculate new G value
            nSucc->updateG();
            newG = nSucc->getG();

            // if successor not on open list or g value lower than before put it on open list
            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG()) {
              // calculate the H value
              nSucc->updateH(goal);
              // put successor on open list
              nSucc->open();
              nodes2D[iSucc] = *nSucc;
              O.push(&nodes2D[iSucc]);
              delete nSucc;
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  // return large number to guide search away
  return 1000;
}

//###################################################
//                                         COST TO GO
//###################################################
// 代价1：Non-holonomic-without-obstacles ： 就是利用dubins曲线或者是reedsShepp曲线计算当前点到目标点的距离代价，不考虑障碍物
// 代价2：Holonomic-with-obstacles：其实就是考虑障碍物的代价，就是astar里的G值（cost——to——far）。 
void updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization) {
  float dubinsCost = 0;
  float reedsSheppCost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;

  // if dubins heuristic is activated calculate the shortest path
  // constrained without obstacles
  if (Constants::dubins) {

    // ONLY FOR dubinsLookup
    //    int uX = std::abs((int)goal.getX() - (int)start.getX());
    //    int uY = std::abs((int)goal.getY() - (int)start.getY());
    //    // if the lookup table flag is set and the vehicle is in the lookup area
    //    if (Constants::dubinsLookup && uX < Constants::dubinsWidth - 1 && uY < Constants::dubinsWidth - 1) {
    //      int X = (int)goal.getX() - (int)start.getX();
    //      int Y = (int)goal.getY() - (int)start.getY();
    //      int h0;
    //      int h1;

    //      // mirror on x axis
    //      if (X >= 0 && Y <= 0) {
    //        h0 = (int)(helper::normalizeHeadingRad(M_PI_2 - t) / Constants::deltaHeadingRad);
    //        h1 = (int)(helper::normalizeHeadingRad(M_PI_2 - goal.getT()) / Constants::deltaHeadingRad);
    //      }
    //      // mirror on y axis
    //      else if (X <= 0 && Y >= 0) {
    //        h0 = (int)(helper::normalizeHeadingRad(M_PI_2 - t) / Constants::deltaHeadingRad);
    //        h1 = (int)(helper::normalizeHeadingRad(M_PI_2 - goal.getT()) / Constants::deltaHeadingRad);

    //      }
    //      // mirror on xy axis
    //      else if (X <= 0 && Y <= 0) {
    //        h0 = (int)(helper::normalizeHeadingRad(M_PI - t) / Constants::deltaHeadingRad);
    //        h1 = (int)(helper::normalizeHeadingRad(M_PI - goal.getT()) / Constants::deltaHeadingRad);

    //      } else {
    //        h0 = (int)(t / Constants::deltaHeadingRad);
    //        h1 = (int)(goal.getT() / Constants::deltaHeadingRad);
    //      }

    //      dubinsCost = dubinsLookup[uX * Constants::dubinsWidth * Constants::headings * Constants::headings
    //                                + uY *  Constants::headings * Constants::headings
    //                                + h0 * Constants::headings
    //                                + h1];
    //    } else {

    /*if (Constants::dubinsShot && std::abs(start.getX() - goal.getX()) >= 10 && std::abs(start.getY() - goal.getY()) >= 10)*/
    //      // start
    //      double q0[] = { start.getX(), start.getY(), start.getT()};
    //      // goal
    //      double q1[] = { goal.getX(), goal.getY(), goal.getT()};
    //      DubinsPath dubinsPath;
    //      dubins_init(q0, q1, Constants::r, &dubinsPath);
    //      dubinsCost = dubins_path_length(&dubinsPath);

    ompl::base::DubinsStateSpace dubinsPath(Constants::r);  
    State* dbStart = (State*)dubinsPath.allocState();
    State* dbEnd = (State*)dubinsPath.allocState();
    dbStart->setXY(start.getX(), start.getY());
    dbStart->setYaw(start.getT());
    dbEnd->setXY(goal.getX(), goal.getY());
    dbEnd->setYaw(goal.getT());
    dubinsCost = dubinsPath.distance(dbStart, dbEnd); // 计算两个位姿之间的 dubin曲线距离
  }

  // if reversing is active use a
  if (Constants::reverse && !Constants::dubins) {  
    //    ros::Time t0 = ros::Time::now();
    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    State* rsStart = (State*)reedsSheppPath.allocState();
    State* rsEnd = (State*)reedsSheppPath.allocState();
    rsStart->setXY(start.getX(), start.getY());
    rsStart->setYaw(start.getT());
    rsEnd->setXY(goal.getX(), goal.getY());
    rsEnd->setYaw(goal.getT());
    reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);  // 计算两个位姿之间的 reedsShepp曲线距离
    //    ros::Time t1 = ros::Time::now();
    //    ros::Duration d(t1 - t0);
    //    std::cout << "calculated Reed-Sheep Heuristic in ms: " << d * 1000 << std::endl;
  }

  // if twoD heuristic is activated determine shortest path
  // unconstrained with obstacles
  if (Constants::twoD && !nodes2D[(int)start.getY() * width + (int)start.getX()].isDiscovered()) {
    //    ros::Time t0 = ros::Time::now();
    // create a 2d start node
    Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
    // create a 2d goal node
    Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);
    // run 2d astar and return the cost of the cheapest path for that node
    nodes2D[(int)start.getY() * width + (int)start.getX()].setG(aStar(goal2d, start2d, nodes2D, width, height, configurationSpace, visualization));
    //    ros::Time t1 = ros::Time::now();
    //    ros::Duration d(t1 - t0);
    //    std::cout << "calculated 2D Heuristic in ms: " << d * 1000 << std::endl;
  }

  if (Constants::twoD) {
    // offset for same node in cell
    // 这个twoDoffset没看懂，start.getX() - (long)start.getX()感觉是0把，em23333
    twoDoffset = sqrt(((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) * ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) +
                      ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())) * ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())));
    twoDCost = nodes2D[(int)start.getY() * width + (int)start.getX()].getG() - twoDoffset;

  }

  // return the maximum of the heuristics, making the heuristic admissable
  // reedsSheppCost 和 dubinsCost是冲突的，两个选一个，然后考虑障碍物的twoDCost 和 不考虑障碍物的（dubinsCost或reedsSheppCost） 选一个大的
  start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));
}

//###################################################
//                                        DUBINS SHOT
//###################################################
// 直接用dubin曲线把当前点和终点连接起来，并判断如果没有碰到障碍物，则成功
Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace) {
  // start
  double q0[] = { start.getX(), start.getY(), start.getT() };
  // goal
  double q1[] = { goal.getX(), goal.getY(), goal.getT() };
  // initialize the path
  DubinsPath path;
  // calculate the path
  dubins_init(q0, q1, Constants::r, &path);

  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);

  Node3D* dubinsNodes = new Node3D [(int)(length / Constants::dubinsStepSize) + 1];

  while (x <  length) {
    double q[3];
    dubins_path_sample(&path, x, q);
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(Helper::normalizeHeadingRad(q[2]));

    // collision check
    if (configurationSpace.isTraversable(&dubinsNodes[i])) {

      // set the predecessor to the previous step
      if (i > 0) {
        dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i].setPred(&start);
      }

      if (&dubinsNodes[i] == dubinsNodes[i].getPred()) {
        std::cout << "looping shot";
      }

      x += Constants::dubinsStepSize;
      i++;
    } else {
      //      std::cout << "Dubins shot collided, discarding the path" << "\n";
      // delete all nodes
      delete [] dubinsNodes;  
      return nullptr;
    }
  }

  //  std::cout << "Dubins shot connected, returning the path" << "\n";
  return &dubinsNodes[i - 1];
}
