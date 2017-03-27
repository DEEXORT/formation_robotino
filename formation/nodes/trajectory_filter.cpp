#include <motion_planning_msgs/JointTrajectoryWithLimits.h>
#include <motion_planning_msgs/FilterJointTrajectory.h>
#include <filters/filter_base.h>
#include <pluginlib/class_list_macros.h>

template <typename T>

class MyFilter: public filters::FilterBase<T>
{
public:
  MyFilter(){};
  ~MyFilter(){};

  bool configure() { return true; }
  bool update(const T& trajectory_in, T& trajectory_out);
};

template <typename T>
inline bool MyFilter<T>::update(const T& trajectory_in, 
                                   T& trajectory_out)
{
  bool success = true;
  int size = trajectory_in.trajectory.points.size();
  int num_traj = trajectory_in.trajectory.joint_names.size();
  trajectory_out = trajectory_in;

  // for every point in time:
  for (int i=0; i<size; ++i)
  {
    // for every (joint) trajectory
    for (int j=0; j<num_traj; ++j)
    {
      double x1 = trajectory_in.trajectory.points[i].positions[j];
       trajectory_out.trajectory.points[i].positions[j] = -x1;
    }
  }

  return success;
}

PLUGINLIB_REGISTER_CLASS(MyFilter, MyFilter<motion_planning_msgs::FilterJointTrajectory::Request>, filters::FilterBase<motion_planning_msgs::FilterJointTrajectory::Request>)