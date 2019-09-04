/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef MANIPULATOR_KINEMATICS_DYNAMICS_MANIPULATOR_KINEMATICS_DYNAMICS_H_
#define MANIPULATOR_KINEMATICS_DYNAMICS_MANIPULATOR_KINEMATICS_DYNAMICS_H_

#include <vector>
#include <cmath>
#include <map>
#include <string>
#include "link_data.h"
#include "manipulator_h_kinematics_dynamics_define.h"
#include "../../../ROBOTIS-Math/robotis_math/include/robotis_math/robotis_math_base.h"

//++
//#include "../../../ROBOTIS-MANIPULATOR-H/manipulator_h_base_module/include/manipulator_h_base_module/base_module.h"
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#define pi M_PI

namespace robotis_manipulator_h
{

enum TreeSelect
{
  ARM
};

class ManipulatorKinematicsDynamics
{
private:
  Eigen::MatrixXd DHTABLE;
  double d1,d2,d3,d4,a1,a2;
  double Lse, Lew;
  double RL_prm;
  
  Eigen::VectorXd JointAngle;
  Eigen::VectorXd tmp_JointAngle;

  Eigen::MatrixXd R03;
  Eigen::MatrixXd R04;
  Eigen::MatrixXd R07;
  Eigen::MatrixXd R47;

  /*****************new_fuction****************/
  Eigen::MatrixXd INV_DHTABLE;
  Eigen::MatrixXd R57;
  Eigen::MatrixXd R72;
  Eigen::MatrixXd R05_notheta3;
  /********************end*********************/
  
public:
  ManipulatorKinematicsDynamics();
  ManipulatorKinematicsDynamics(TreeSelect tree);
  ~ManipulatorKinematicsDynamics();

  LinkData *manipulator_link_data_[ ALL_JOINT_ID + 1];
  //+
  //BaseModule *basemodule;

  std::vector<int> findRoute(int to);
  std::vector<int> findRoute(int from, int to);

  double totalMass(int joint_ID);
  Eigen::MatrixXd calcMC(int joint_ID);
  Eigen::MatrixXd calcCOM(Eigen::MatrixXd MC);

  void forwardKinematics(int joint_ID);

  Eigen::MatrixXd calcJacobian(std::vector<int> idx);
  Eigen::MatrixXd calcJacobianCOM(std::vector<int> idx);
  Eigen::MatrixXd calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position,
                            Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation);

  bool inverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
                         double tar_phi, double tar_slide_pos, bool is_p2p, bool Line_Phi);
  bool inverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
                         int max_iter, double ik_err);
  Eigen::MatrixXd Trans( double &Theta, Eigen::VectorXd &DH );
  Eigen::Vector3d forwardKinematics_7(int joint_ID, Eigen::VectorXd angle);
  bool InverseKinematics_7( Eigen::VectorXd position, Eigen::Matrix3d rotation, double phi, 
                            double slide_position, Eigen::VectorXd Old_JointAngle, bool is_p2p);
  bool slideInverseKinematics(Eigen::Vector3d goal_position, Eigen::Matrix3d rotation, 
                                                            double slide_pos, double& goal_slide_pos);
  static Eigen::MatrixXd rotation2rpy( Eigen::MatrixXd rotation );
  void load_LinkParam();
  void getPhiAngle();
  double est_joint_limit_degree(double max_limit, double min_limit, double curr_angle);
  double est_wrist_singularity_degree(double Joint2, double Joint6);
  /*****************new_fuction****************/
  bool InverseKinematics_p2p( Eigen::VectorXd position, Eigen::Matrix3d rotation, double phi, 
                            double slide_position, Eigen::VectorXd Old_JointAngle, bool is_p2p, bool Line_Phi);
  bool limit_check(Eigen::Vector3d goal_position, Eigen::Matrix3d rotation);
  inline double get_d4(){return d4;};
  /********************end*********************/
//++
  bool do_wirst_avoid;
  void get_WirstAvoid(bool is_wirst);
  // for estimate joint limit
  

  Eigen::VectorXd JointAngle_for_est_lmt;
  bool is_est_joint_limit;  // if true, the result of kinematics will not be send to motor.

};

}

#endif /* MANIPULATOR_KINEMATICS_DYNAMICS_MANIPULATOR_KINEMATICS_DYNAMICS_H_ */
