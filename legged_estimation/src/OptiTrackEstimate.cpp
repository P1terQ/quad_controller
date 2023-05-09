#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "legged_estimation/OptiTrackEstimate.h"

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace legged {

OptiTrackStateEstimate::OptiTrackStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                        const PinocchioEndEffectorKinematics& eeKinematics)
    : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics) 
{
    ros::NodeHandle nh;
    OptiTrack_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/dog/pose0", 10, &OptiTrackStateEstimate::callback, this);

    eeKinematics_->setPinocchioInterface(pinocchioInterface_);



}

}

