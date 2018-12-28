#include <algorithm>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <sstream>

#include <cmath>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>



template <typename T>
class Node {
private:
  Node* prev_;
  Node* next_;
  T data_;
 public:
  Node() : prev_(this), next_(this) {}
  Node(const T& data) : prev_(this), next_(this), data_(data) {}
  T& Data() { return this->data_; }
  Node<T>* Next() { return this->next_; }
  Node<T>* Prev() { return this->prev_; }
  Node<T>* InsertAfter(Node<T>* other) {
    Node<T>* result = other->prev_;
    other->prev_->next_ = this->next_;
    this->next_->prev_ = other->prev_;
    this->next_ = other;
    other->prev_ = this;
    return result;
  }
  Node<T>* InsertAfter(const T& data) {
    Node* ins = new Node<T>(data);
    return this->InsertAfter(ins);
  }
};
template <typename T>
class LList {
 public:
  typedef Node<T>* NodePtr;
  LList() : head_(NULL) {}
  Node<T>* Head() { return head_; }
  void Append(const T& item) {
    Node<T>* node = new Node<T>(item);
    if (head_ == NULL) {
      head_ = node;
    } else {
      head_->Prev()->InsertAfter(node);
    }
  }
 private:
  Node<T>* head_;
};














struct Pose {
  Eigen::Vector2d pos;
  double hdg;
};
struct PathPoint {
  Eigen::Vector2d pos;
  double hdg;
  double curv;
  double curvdot;
  double s_off;
  bool hdg_is_valid;
  bool curv_is_valid;
  bool curvdot_is_valid;
};

class Track {
 public:
  Track() {}

  void AddPoint(double x, double y) {
    PathPoint pp;
    pp.pos[0] = x;
    pp.pos[1] = y;
    this->path_.Append(pp);
  }
  void AddPoint(const Eigen::Vector2d& pos) {
    PathPoint pp;
    pp.pos = pos;
    this->path_.Append(pp);
  }


  double ComputeArcLength(Node<PathPoint>* start, Node<PathPoint>* end) {
    double result = 0.0;

    Node<PathPoint>* iter = start;
    for (; iter != end; iter = iter->Next()) {
      Eigen::Vector2d delta_pos = iter->Next()->Data().pos - iter->Data().pos;
      result += delta_pos.norm();
    }
    return result;
  }
  void ComputeCurvature(Node<PathPoint>* node) {
    Node<PathPoint>* prev = node->Prev();
    Node<PathPoint>* next = node->Next();
    Eigen::Vector2d front = next->Data().pos - node->Data().pos;
    Eigen::Vector2d back = node->Data().pos - prev->Data().pos;
    double stheta = (back[0] * front[1] - back[1] * front[0]) / (front.norm() * back.norm());
    if (stheta == 0.0) {
      node->Data().curv = 0.0;
    }
    Eigen::Vector2d front_unit = front / front.norm();
    Eigen::Vector2d front_unit_rot = Eigen::Vector2d(-front_unit[1], front_unit[0]);

    double ctheta = std::sqrt(1.0 - stheta*stheta);
    double radius = (back.norm() * back.norm() + front.norm() * front.norm() + 2.0 * back.norm() * front.norm() * ctheta) / (4.0 * stheta * stheta);
    double sin_beta = front.norm() / 2 * radius;
    double cos_beta = std::sqrt(1 - sin_beta * sin_beta);
    Eigen::Vector2d center = node->Data().pos + 0.5 * front + radius * cos_beta * front_unit_rot;

    node->Data().curv = (1.0 / radius) * (stheta > 0.0 ? 1.0 : -1.0);

  }
  void ComputeCurvDot(Node<PathPoint>* node) {
    Node<PathPoint>* prev = node->Prev();
    Node<PathPoint>* next = node->Next();

    double curvdelta_n = next->Data().curv - node->Data().curv;
    double arclength_n = ComputeArcLength(node, node->Next());
    double curvdelta_p = node->Data().curv - prev->Data().curv;
    double arclength_p = ComputeArcLength(node->Prev(), node);

    double curvdot_n = curvdelta_n / arclength_n;
    double curvdot_p = curvdelta_p / arclength_p;

    node->Data().curvdot = 0.5 * (curvdot_n + curvdot_p);
  }
  void ComputeHdg(Node<PathPoint>* node) {
    Node<PathPoint>* prev = node->Prev();
    Node<PathPoint>* next = node->Next();

    Eigen::Vector2d diff_n = next->Data().pos - node->Data().pos;
    Eigen::Vector2d diff_p = node->Data().pos - prev->Data().pos;
    Eigen::Vector2d bidir = next->Data().pos - prev->Data().pos;

    double hdg = std::atan2(bidir[1], bidir[0]);
    node->Data().hdg = hdg;
  }



  void Compute() {
    Node<PathPoint>* node = path_.Head();
    do {
      ComputeCurvature(node);
      node = node->Next();
    } while (node != path_.Head());

    node = path_.Head();
    do {
      ComputeCurvDot(node);
      node = node->Next();
    } while (node != path_.Head());

    node = path_.Head();
    do {
      ComputeHdg(node);
      node = node->Next();
    } while (node != path_.Head());

    double s_pos = 0.0;
    node = path_.Head();
    do {
      node->Data().s_off = s_pos;
      s_pos += ComputeArcLength(node, node->Next());
      node = node->Next();
    } while (node != path_.Head());
  }


  PathPoint GetPoint(double s) {
    Node<PathPoint>* prev = path_.Head();
    while (prev->Next() != path_.Head() && prev->Data().s_off > s) {
      prev = prev->Next();
    }

    Node<PathPoint>* next = prev->Next();

    double dist = next->Data().s_off - prev->Data().s_off;
    double dist_p = s - prev->Data().s_off;
    double dist_n = next->Data().s_off - s;

    double coeff_p = (dist - dist_p) / dist;
    double coeff_n = (dist - dist_n) / dist;

    PathPoint result;
    result.s_off = s;
    result.pos = prev->Data().pos * coeff_p + next->Data().pos * coeff_n;
    result.hdg = prev->Data().hdg * coeff_p + next->Data().hdg * coeff_n;
    result.curv = prev->Data().curv * coeff_p + next->Data().curv * coeff_n;
    result.curvdot = prev->Data().curvdot * coeff_p + next->Data().curvdot * coeff_n;
    return result;
  }

  void Center(const Eigen::Vector2d& point) {
    Node<PathPoint>* node = path_.Head();
    do {
      node->Data().pos = node->Data().pos - point;
      node = node->Next();
    } while (node != path_.Head());
  }

  void Scale(const Eigen::Vector2d& scale) {
    Node<PathPoint>* node = path_.Head();
    do {
      node->Data().pos[0] = node->Data().pos[0] * scale[0];
      node->Data().pos[1] = node->Data().pos[1] * scale[1];
      node = node->Next();
    } while (node != path_.Head());
  }

  void Print() {
    Node<PathPoint>* node = path_.Head();
    do {
      std::cout << "Node: pos=(" << node->Data().pos[0] << ", " << node->Data().pos[1] << ")"
                << ", hdg=" << node->Data().hdg << ", curv=" << node->Data().curv << ", curvdot=" << node->Data().curvdot
                << std::endl;
      node = node->Next();
    } while (node != path_.Head());
  }

  void GetPath(nav_msgs::Path& path) {
    Node<PathPoint>* node = path_.Head();

    //path.header.seq = 0;
    //path.header.stamp = 0;
    path.header.frame_id = "map";
    do {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = node->Data().pos[0];
      pose.pose.position.y = node->Data().pos[1];
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(node->Data().hdg / 2.0);
      pose.pose.orientation.w = std::cos(node->Data().hdg / 2.0);
      path.poses.push_back(pose);

      node = node->Next();
    } while (node != path_.Head());

  }

  void GetPathWithOffset(nav_msgs::Path& path, double t_coord = 0.0) {
    Node<PathPoint>* node = path_.Head();

    //path.header.seq = 0;
    //path.header.stamp = 0;
    path.header.frame_id = "map";
    do {
      Eigen::Vector2d unit_s(std::cos(node->Data().hdg), std::sin(node->Data().hdg));
      Eigen::Vector2d unit_t(-std::sin(node->Data().hdg), std::cos(node->Data().hdg));

      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = node->Data().pos[0] + t_coord * unit_t[0];
      pose.pose.position.y = node->Data().pos[1] + t_coord * unit_t[1];
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(node->Data().hdg / 2.0);
      pose.pose.orientation.w = std::cos(node->Data().hdg / 2.0);
      path.poses.push_back(pose);

      node = node->Next();
    } while (node != path_.Head());
  }

  void GetPoseArray(geometry_msgs::PoseArray& parr) {
    Node<PathPoint>* node = path_.Head();

    //path.header.seq = 0;
    //path.header.stamp = 0;
    parr.header.frame_id = "map";
    do {
      geometry_msgs::Pose pose;
      pose.position.x = node->Data().pos[0];
      pose.position.y = node->Data().pos[1];
      pose.position.z = 0.0;
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = std::sin(node->Data().hdg / 2.0);
      pose.orientation.w = std::cos(node->Data().hdg / 2.0);
      parr.poses.push_back(pose);

      node = node->Next();
    } while (node != path_.Head());
  }

  void GetCurvs(visualization_msgs::MarkerArray& markers) {
    Node<PathPoint>* node = path_.Head();

    int last_id = 0;
    //path.header.seq = 0;
    //path.header.stamp = 0;
    //markers.header.frame_id = "map";
    do {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.ns = "my_namespace";
      marker.id = last_id++;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = node->Data().pos[0];
      marker.pose.position.y = node->Data().pos[1];
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = std::sin((node->Data().hdg + (M_PI/2.0)) / 2.0);
      marker.pose.orientation.w = std::cos((node->Data().hdg + (M_PI/2.0)) / 2.0);
      marker.scale.x = node->Data().curv;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      //only if using a MESH_RESOURCE marker type:
      //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

      markers.markers.push_back(marker);

      node = node->Next();
    } while (node != path_.Head());
  }

  bool SaveToFile(const std::string& csvfile) {
    std::ofstream outfp(csvfile.c_str());
    outfp << std::setprecision(10);
    Node<PathPoint>* node = path_.Head();
    do {
      outfp << node->Data().pos[0] << "," << node->Data().pos[1] << std::endl;
      node = node->Next();
    } while (node != path_.Head());
  }


 private:
  LList<PathPoint> path_;
};









static double square_even(int period, int i) {
  int phase = (i % period);
  return (phase < period / 4 || phase >= period - (period / 4)) ? 1.0 : -1.0;
}
static double square_even(double period, double t) {
  double phase = fmod(t, period);
  return (phase < (period / 4.0) || phase >= period - (period / 4.0)) ? 1.0 : -1.0;
}




static void AddCurve(Track& track) {
  Eigen::Vector2d pos(0.0, 0.0);
  double hdg = 0.0;
  double curv = 1.0 / 10.0;
  double curvdot = 0.0;
  double delta_s = 0.1;

  for (double s = 0.0; s < 2 * M_PI * 10.0; s += delta_s) {
    //curvdot = 0.001 * square_even(10.0, s);
    curv = curv + delta_s * curvdot;
    hdg = hdg + delta_s * curv;
    pos = pos + delta_s * Eigen::Vector2d(std::cos(hdg), std::sin(hdg));
    track.AddPoint(pos);
  }
}


static void AddPath(Track& track) {
  Eigen::Vector2d pos(0.0, 0.0);
  double hdg = 0.0;
  double curv = 0.0;
  double curvdot = 0.0;
  double delta_s = 0.1;

  for (int i = 0; i < 128; i++) {
    //curvdot = 0.5 * square_even(16, i);
    curvdot = 0.01;
    curv = curv + delta_s * curvdot;
    hdg = hdg + delta_s * curv;
    pos = pos + delta_s * Eigen::Vector2d(std::cos(hdg), std::sin(hdg));
    track.AddPoint(pos);
  }
}


static void LoadFile(Track& track, const std::string& csvfile) {
  std::ifstream fs(csvfile.c_str(), std::ifstream::in);
  if (!fs) {
    std::cout << "File not open\n";
    return;
  }
  std::string line;
  const char delim = ',';


  std::vector<Eigen::Vector2d> points;

  while (std::getline(fs, line)) {
    //std::cout << "line: " << line << std::endl;

    std::istringstream ss(line);
    Eigen::Vector2d pos;
    ss >> pos[0];
    //ss.ignore(10, delim); 
    std::getline(ss, line, delim);
    ss >> pos[1];

    //std::cout << "pos: " << pos[0] << ", " << pos[1] << std::endl;
    if (ss) {
      points.push_back(pos);
    }
  }


  //std::reverse(points.begin(), points.end());

  for (std::vector<Eigen::Vector2d>::iterator iter = points.begin(); iter != points.end(); iter++) {
    track.AddPoint(*iter);
  }
}







int main(int argc, char** argv) {
  ros::init(argc, argv, "track_example");

  ros::NodeHandle nh;
  ros::Publisher pub_track_center = nh.advertise<nav_msgs::Path>("track_center", 1000);
  ros::Publisher pub_track_inside = nh.advertise<nav_msgs::Path>("track_inside", 1000);
  ros::Publisher pub_track_outside = nh.advertise<nav_msgs::Path>("track_outside", 1000);
  ros::Publisher track_poses_pub = nh.advertise<geometry_msgs::PoseArray>("track_poses", 1000);
  ros::Publisher pub_curvs = nh.advertise<visualization_msgs::MarkerArray>("track_curvs", 1000);

  Track track;
  //AddCurve(track);
  LoadFile(track, "track.csv");
  track.Compute();
  //track.Print();
  //track.Center(Eigen::Vector2d(0.0, -2.0));
  //track.Scale(Eigen::Vector2d(0.5, 0.5));
  //track.SaveToFile("track_output.csv");


  //for (double s = 0.0; s < 40.0; s += 0.1) {
  //  PathPoint pt = track.GetPoint(s);
  //  std::cout << "pt: pos=(" << pt.pos[0] << ", " << pt.pos[1] << ")"
  //            << ", hdg=" << pt.hdg << ", curv=" << pt.curv << ", curvdot=" << pt.curvdot
  //            << std::endl;
  //}

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    nav_msgs::Path path;
    track.GetPath(path);
    pub_track_center.publish(path);
    nav_msgs::Path path_inside;
    track.GetPathWithOffset(path_inside, 0.5);
    pub_track_inside.publish(path_inside);
    nav_msgs::Path path_outside;
    track.GetPathWithOffset(path_outside, -0.5);
    pub_track_outside.publish(path_outside);

    geometry_msgs::PoseArray poses;
    track.GetPoseArray(poses);
    track_poses_pub.publish(poses);

    visualization_msgs::MarkerArray markers;
    track.GetCurvs(markers);
    pub_curvs.publish(markers);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}



