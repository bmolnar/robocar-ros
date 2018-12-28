#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>


static bool MapGetPos(const nav_msgs::OccupancyGrid& map, uint32_t x, uint32_t y, size_t& pos) {
  if ((x >= map.info.width) || (y >= map.info.height)) {
    return false;
  }
  pos = x + map.info.width * y;
  return true;
}
static void MapDrawRect(nav_msgs::OccupancyGrid& map, uint32_t x, uint32_t y, uint32_t width, uint32_t height, int8_t value) {
  for (uint32_t j = 0; j < height; j++) {
    for (uint32_t i = 0; i < width; i++) {
      size_t pos;
      if (MapGetPos(map, x+i, y+j, pos)) {
        map.data[pos] = value;
      }
    }
  }
}
static void MapClear(nav_msgs::OccupancyGrid& map, int8_t value) {
  MapDrawRect(map, 0, 0, map.info.width, map.info.height, value);
}




class MapExampleNode {
 public:
  MapExampleNode();
  void InitMap(nav_msgs::OccupancyGrid& map);
  void UpdateMap(nav_msgs::OccupancyGrid& map);

  int Run();
  static int main(int argc, char** argv);

 private:
  ros::NodeHandle nh_;
  ros::Publisher map_pub_;
  nav_msgs::OccupancyGrid map_;
};

MapExampleNode::MapExampleNode() {
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1000);

}

void MapExampleNode::InitMap(nav_msgs::OccupancyGrid& map) {
  map.info.map_load_time = ros::Time::now();
  map.info.resolution = (float) 1.0; // The map resolution [m/cell]
  map.info.width = (uint32_t) 400;
  map.info.height = (uint32_t) 400;
  map.info.origin.position.x = (double) 0;
  map.info.origin.position.y = (double) 0;
  map.info.origin.position.z = (double) 0;
  map.info.origin.orientation.x = (double) 0.0;
  map.info.origin.orientation.y = (double) 0.0;
  map.info.origin.orientation.z = (double) 0.0;
  map.info.origin.orientation.w = (double) 1.0;

  map.data.resize(map.info.width * map.info.height);

  MapClear(map, -1);
}


void MapExampleNode::UpdateMap(nav_msgs::OccupancyGrid& map) {
  map.info.map_load_time = ros::Time::now();

  MapClear(map, -1);
  uint64_t msec = ((uint64_t) map.info.map_load_time.sec) * 1000 + ((uint64_t) map.info.map_load_time.nsec) / 1000000;
  uint32_t pos = (uint32_t)(msec % 4000);

  uint32_t x = (pos * 300 / 4000) + 50;
  uint32_t y = 100;
  MapDrawRect(map, x, y, 10, 10, 50);
}

int MapExampleNode::Run() {
  InitMap(map_);

  ros::Rate loop_rate(10);
  while (ros::ok()) {

    UpdateMap(map_);

    map_pub_.publish(map_);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

int MapExampleNode::main(int argc, char** argv) {
  ros::init(argc, argv, "map_example");

  MapExampleNode node;
  return node.Run();
}

int main(int argc, char** argv) {
  return MapExampleNode::main(argc, argv);
}
