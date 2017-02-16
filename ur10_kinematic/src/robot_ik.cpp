//Xinyu  Feb, 2017
//Module for computing robot inverse kinematic including rail displacements.

#include <ur10_kinematic/robot_ik.h>

double g_linear_location;


robot_kin::robot_kin(){
	ROS_INFO("In robot kinematic constructor!!!");
}

bool robot_kin::compute_IK_solutions(int sector_ID, 
							Eigen::Affine3d gripper_grasp_pose, 
							Eigen::Affine3d gripper_approach_pose,
							std::vector<double> &IK_soln_grasp, 
							std::vector<double> &IK_solon_approach){
	bool found = false;
	switch (sector_ID){
		case 21://agv1; left end?
			g_linear_location = 1.0;
			break;
		case 22://agv2; right end?
			g_linear_location = 1.5;
			break;
		case 31://bin1;
			g_linear_location = 2.0;
			break;
		case 32://bin2;
			g_linear_location = 2.1;
			break;
		case 33://bin3;
			g_linear_location = 2.2;
			break;
		case 34://bin4;
			g_linear_location = 2.3;
			break;
		case 35://bin5;
			g_linear_location = 2.3;
			break;
		case 36://bin6;
			g_linear_location = 2.3;
			break;
		case 37://bin7;
			g_linear_location = 2.3;
			break;
		case 38://bin8;
			g_linear_location = 2.3;
			break;
	}

	int nsolns_approach, nsolns_grasp;
	std::vector<Eigen::VectorXd> q6dof_solns_approach, q6dof_solns_grasp;


	nsolns_approach = ik_solver.ik_solve(gripper_approach_pose, q6dof_solns_approach);
	nsolns_grasp = ik_solver.ik_solve(gripper_grasp_pose,q6dof_solns_grasp);

	//get desired grasp and approach pose;
	double dist = 20;
	double q_err;
	for (int i=0; i<nsolns_approach; i++){
		for (int j=0; j<nsolns_grasp; j++){
			Eigen::VectorXd q1 = q6dof_solns_approach[i];//q1 corresponds to approach pose;
			Eigen::VectorXd q2 = q6dof_solns_grasp[j];//q2 corresponds to grasp pose;
			q_err = (q1 - q2).norm();
			if (q_err < dist){
				dist = q_err;
				//joint space value should be repackaged again;
				IK_soln_grasp = map27dof(q1);
				IK_solon_approach = map27dof(q2);	
				found = true;
			}	
		}
	}

	return found;
}


std::vector<double> robot_kin::map27dof(Eigen::VectorXd q6dof){
	//q7dof turn is: 0elbow; 1linear; 2shoulder_lift; 3shoulder_pan; 4wrist1; 5wrist2; 6wrist3;
	//q6dof turn is: 0shoulder_pan; 1shoulder_lift; 2elbow; 3wrist1; 4wrist2; 5wrist3;
	std::vector<double> q7dof;
	q7dof.clear();

	q7dof.push_back(q6dof[2]);
	q7dof.push_back(g_linear_location);
	q7dof.push_back(q6dof[1]);
	q7dof.push_back(q6dof[0]);
	q7dof.push_back(q6dof[3]);
	q7dof.push_back(q6dof[4]);
	q7dof.push_back(q6dof[5]);

	return q7dof;
}