#include "backward.hpp"
#include "corridor_generate.h"
#include "grid_map.h"
#include "traj_optimize3d.h"
#include "traj_search3d.h"
namespace backward {
backward::SignalHandling sh;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_optimization");
  ros::NodeHandle nh("~");

  // 创建 GridMapGenerator 对象
  std::vector<std::string> global_map_layers = {"elevation",
                                                "dynamic_obstacle"};
  std::shared_ptr<grid_map::GridMap> global_map_ptr =
      std::make_shared<grid_map::GridMap>(global_map_layers);
  std::shared_ptr<GridMapGenerator> grid_map_generator_ptr =
      std::make_shared<GridMapGenerator>(nh, global_map_ptr, global_map_layers);

  std::shared_ptr<AstarSearcher> grid_path_finder_ptr =
      std::make_shared<AstarSearcher>(nh, grid_map_generator_ptr);

  std::shared_ptr<CorridorGenerator> corridor_generator_ptr =
      std::make_shared<CorridorGenerator>(nh, grid_map_generator_ptr,
                                          grid_path_finder_ptr);
  ros::Rate rate(100);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  // 等待程序结束
  ros::waitForShutdown();
  return 0;
}
