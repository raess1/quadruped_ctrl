#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include <Controllers/RobotParameters.h>
#include <WBC_Ctrl/MIT_UserParameters.h>
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/GaitScheduler.h"
#include "LegController.h"
#include "StateEstimatorContainer.h"
#include "Dynamics/Quadruped.h"
#include "DesiredStateCommand.h"

/**
 *
 */
template <typename T>
struct ControlFSMData {
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Quadruped<T>* _quadruped;
  StateEstimatorContainer<T>* _stateEstimator;
  LegController<T>* _legController;
  GaitScheduler<T>* _gaitScheduler;
  DesiredStateCommand<T>* _desiredStateCommand;
  RobotControlParameters* controlParameters;
  MIT_UserParameters* userParameters;
  // VisualizationData* visualizationData;
};

template struct ControlFSMData<double>;
template struct ControlFSMData<float>;

#endif  // CONTROLFSM_H
