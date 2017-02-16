//Xinyu Feb, 2017
//Robot ik module header file 
#ifndef ROBOT_IK_H
#define ROBOT_IK_H

#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <ur_fk_ik/ur_kin.h>
#include <sensor_msgs/JointState.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>

using namespace std;
class robot_kin{
public:
	robot_kin();
	bool compute_IK_solutions(int sector_ID, 
								Eigen::Affine3d gripper_grasp_pose, 
								Eigen::Affine3d gripper_approach_pose,
								std::vector<double> &IK_soln_grasp, 
								std::vector<double> &IK_solon_approach);

	std::vector<double> map27dof(Eigen::VectorXd q6dof);
private:
	UR10IkSolver ik_solver;


};

#endif //ROBOT_IK_H