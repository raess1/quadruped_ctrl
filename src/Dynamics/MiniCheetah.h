/*! @file MiniCheetah.h
 *  @brief Utility function to build a Mini Cheetah Quadruped object
 *
 * This file is based on MiniCheetahFullRotorModel_mex.m and builds a model
 * of the Mini Cheetah robot.  The inertia parameters of all bodies are
 * determined from CAD.
 *
 */

#ifndef PROJECT_MINICHEETAH_H
#define PROJECT_MINICHEETAH_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"
#include "ros/ros.h"

/*!
 * Generate a Quadruped model of Mini Cheetah
 */
template <typename T>
Quadruped<T> buildMiniCheetah() {
  Quadruped<T> cheetah;

  std::cout << "[GAIT] buildMiniCheetah" << std::endl;

  cheetah._robotType = RobotType::MINI_CHEETAH;

  cheetah._bodyMass = 8.0;
  cheetah._bodyLength = 0.246 * 2;
  cheetah._bodyWidth = 0.055 * 2;
  cheetah._bodyHeight = 0.095 * 2;
  cheetah._abadGearRatio = 9;
  cheetah._hipGearRatio = 9;
  cheetah._kneeGearRatio = 12.6;
  cheetah._abadLinkLength = 0.0768;
  cheetah._hipLinkLength = 0.279523;
  cheetah._kneeLinkY_offset = 0.00;
  cheetah._kneeLinkLength = 0.302314;
  cheetah._maxLegLength = 0.578;


  cheetah._motorTauMax = 6.f;
  cheetah._batteryV = 44;
  cheetah._motorKT = .095;  // this is flux linkage * pole pairs
  cheetah._motorR = 0.09;
  cheetah._jointDamping = .01;
  cheetah._jointDryFriction = 1;

  // ------- new code (do not forget ros include at the top) ------------
  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "quadruped_ctrl_lib");
  ros::NodeHandle nh;
  double v;
  if (nh.getParam("/robot/body_mass", v))
  {
    std::cout << "[GAIT] body_mass " << v << std::endl;
    cheetah._bodyMass = v;
  }
  double bl;
  if (nh.getParam("/robot/body_length", bl))
  {
    std::cout << "[GAIT] body_length " << bl << std::endl;
    cheetah._bodyLength = bl;
  }
  double bw;
  if (nh.getParam("/robot/body_width", bw))
  {
    std::cout << "[GAIT] body_width " << bw << std::endl;
    cheetah._bodyWidth = bw;
  }
  double bh;
  if (nh.getParam("/robot/body_height", bh))
  {
    std::cout << "[GAIT] body_height " << bh << std::endl;
    cheetah._bodyHeight = bh;
  }
  double agr;
  if (nh.getParam("/robot/abad_gear_ratio", agr))
  {
    std::cout << "[GAIT] abad_gear_ratio " << agr << std::endl;
    cheetah._abadGearRatio = agr;
  }
  // ---------------------------------------------------------------------

  // rotor inertia if the rotor is oriented so it spins around the z-axis
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX =
      RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY =
      RX * rotorRotationalInertiaZ * RX.transpose();

  // spatial inertias
  Mat3<T> abadRotationalInertia;
  abadRotationalInertia << 0.000887, -0.001, 1.436E-04, -0.001, 0.009, -2.746E-05, 1.436E-04, -2.746E-05, 0.009;
  abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<T> abadCOM(0.0531, 0.00055, 0.00058);  // LEFT (abduct <origin xyz="0.0 0.036 0."/>)
  SpatialInertia<T> abadInertia(1.11, abadCOM, abadRotationalInertia); // (abduct <mass value="0.54"/>)

  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<T> hipCOM(0.00047, 0.058, -0.029); // (thigh <origin xyz="0.0 0.016 -0.02"/>)
  SpatialInertia<T> hipInertia(1.114, hipCOM, hipRotationalInertia); // (thigh <mass value="0.634"/>)

  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
  Vec3<T> kneeCOM(0, 0, -0.061); //(shank <origin xyz="0.0 0.0 -0.209"/>) 
  SpatialInertia<T> kneeInertia(0.117, kneeCOM, kneeRotationalInertia); // (shank <mass value="0.064"/>)

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.065, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.065, rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0, 0, 0);
  SpatialInertia<T> bodyInertia(cheetah._bodyMass, bodyCOM,
                                bodyRotationalInertia);

  cheetah._abadInertia = abadInertia;
  cheetah._hipInertia = hipInertia;
  cheetah._kneeInertia = kneeInertia;
  cheetah._abadRotorInertia = rotorInertiaX;
  cheetah._hipRotorInertia = rotorInertiaY;
  cheetah._kneeRotorInertia = rotorInertiaY;
  cheetah._bodyInertia = bodyInertia;

  // locations
  cheetah._abadRotorLocation = Vec3<T>(0.155, 0.6, 0);
  cheetah._abadLocation =
      Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
  cheetah._hipLocation = Vec3<T>(0, cheetah._abadLinkLength, 0);
  cheetah._hipRotorLocation = Vec3<T>(0, 0.04, 0);
  cheetah._kneeLocation = Vec3<T>(0, 0, -cheetah._hipLinkLength);
  cheetah._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return cheetah;
}

#endif  // PROJECT_MINICHEETAH_H
