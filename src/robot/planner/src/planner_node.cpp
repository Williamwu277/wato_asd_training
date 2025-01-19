#include "planner_node.hpp"


// ------------------- Supporting Structures -------------------
 
// 2D grid index
struct CellIndex
{
  int x;
  int y;
 
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
 
  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }
 
  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};
 
// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};
 
// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h
 
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};
 
// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};


// Constructor
PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  
  // Publishers
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/map", 10);

  // Subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10,
    std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1)
  );
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    10,
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1)
  );
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point",
    10,
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1)
  );

  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this)
  );

  // Other variables
  goal_received_ = false;

}


// helper functions
bool goalReached(double goal_x, double goal_y, double pose_x, double pose_y){
  double dx = goal_x - pose_x;
  double dy = goal_y - pose_y;
  return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}


int calculateF(CellIndex a, CellIndex b){
  return std::abs(a.x - b.x) + std::abs(a.y - b.y); // Use Manhattan distance instead
}


int convertToGridIndex(double cur, double resolution, double originLength){
  return (int)((cur - originLength) / resolution);
}


// Callback functions
void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}


void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
  robot_pose_ = msg->pose.pose;
}


void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg){
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}


void PlannerNode::timerCallback(){
  if(state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if(goalReached(goal_.point.x, goal_.point.y, robot_pose_.position.x, robot_pose_.position.y)) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
    }else {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      planPath();
    }
  }
}


void PlannerNode::planPath() {
  if(!goal_received_ || current_map_.data.empty()){
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return;
  }
  
  if(goal_.header.frame_id != "map"){
    RCLCPP_WARN(this->get_logger(), "Frame is not the same as map");
    return;
  }

  // A* Implementation here
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "map";

  // Calculate start and end points
  double mapRes = current_map_.info.resolution;
  int mapWidth = static_cast<int>(current_map_.info.width);
  int mapHeight = static_cast<int>(current_map_.info.height);
  int startX = convertToGridIndex(robot_pose_.position.x, mapRes, current_map_.info.origin.position.x);
  int startY = convertToGridIndex(robot_pose_.position.y, mapRes, current_map_.info.origin.position.y);
  int endX = convertToGridIndex(goal_.point.x, mapRes, current_map_.info.origin.position.x);
  int endY = convertToGridIndex(goal_.point.y, mapRes, current_map_.info.origin.position.y);
  CellIndex start(startX, startY);
  CellIndex end(endX, endY);

  // Do some checks to see if its in the grid
  if(!(
      0 <= startX && startX <= mapWidth &&
      0 <= startY && startY <= mapHeight &&
      0 <= endX && endX <= mapWidth &&
      0 <= endY && endY <= mapHeight
    )){
    RCLCPP_WARN(this->get_logger(), "Start or end locations not in grid");
    return;
  }

  // Initialize the visited set and priority queue
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> visited;
  std::unordered_map<CellIndex, double, CellIndexHash> cost;
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> pq;
  pq.push(AStarNode(start, 0));
  visited[start] = start;
  cost[start] = 0;

  // possible directions to move in
  int dx[8] = {0, 0, 1, 1, 1, -1, -1, -1};
  int dy[8] = {1, -1, 0, 1, -1, 0, 1, -1};

  // run A*
  while(!pq.empty()){
    AStarNode current = pq.top();
    pq.pop();
    if(current.index == end){
      break;
    }
    for(int i=0; i<8; i++){
      int nx = dx[i] + current.index.x;
      int ny = dy[i] + current.index.y;
      CellIndex nw(nx, ny);
      if(
          0 <= nx && nx <= mapWidth &&
          0 <= ny && ny <= mapHeight &&
          (
            cost.find(nw) == cost.end() ||
            cost[nw] > cost[current.index] + current_map_.data[nx * mapWidth + ny]
          )
        ){
          cost[nw] = cost[current.index] + cost[current.index] + current_map_.data[nx * mapWidth + ny];
          double priority = cost[nw] + calculateF(nw, end);
          pq.push(AStarNode(nw, priority));
          visited[nw] = current.index;
        }
    }
  }
  
  // Backtrack to add the path to the publisher
  CellIndex position = end;
  std::vector<geometry_msgs::msg::PoseStamped> ret;
  while(position != visited[position]){

    geometry_msgs::msg::PoseStamped waypoint;
    waypoint.pose.position.x = position.x * mapRes + current_map_.info.origin.position.x;
    waypoint.pose.position.y = position.y * mapRes + current_map_.info.origin.position.y;
    waypoint.pose.position.z = 0.0;    // Default values 
    waypoint.pose.orientation.x = 0.0;
    waypoint.pose.orientation.y = 0.0;
    waypoint.pose.orientation.z = 0.0;
    waypoint.pose.orientation.w = 1.0;
    ret.push_back(waypoint);

    position = visited[position];
  }

  // publish the path
  reverse(ret.begin(), ret.end());
  path.poses = ret;
  path_pub_->publish(path);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
