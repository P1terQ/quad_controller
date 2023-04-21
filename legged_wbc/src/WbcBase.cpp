//
// Created by qiayuan on 2022/7/1.
//
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_wbc/WbcBase.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <utility>

namespace legged {
WbcBase::WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics)
    : pinocchioInterfaceMeasured_(pinocchioInterface),
      pinocchioInterfaceDesired_(pinocchioInterface),
      info_(std::move(info)),
      mapping_(info_),
      inputLast_(vector_t::Zero(info_.inputDim)),
      eeKinematics_(eeKinematics.clone()) 
{
  numDecisionVars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + info_.actuatedDofNum;  // 6 + 3*4 + 12 = 30
  qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
}

vector_t WbcBase::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode, scalar_t /*period*/) 
{
  // update contact
  contactFlag_ = modeNumber2StanceLeg(mode);
  numContacts_ = 0;
  for (bool flag : contactFlag_) {
    if (flag) {
      numContacts_++;
    }
  }

  // update qMeasured_, vMeasured_, j_ ,dj_
  updateMeasured(rbdStateMeasured);

  // update pinocchioInterfaceDesired_
  updateDesired(stateDesired, inputDesired);

  return {};
}

void WbcBase::updateMeasured(const vector_t& rbdStateMeasured) 
{
  qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);  // x y z Pos
  qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();  // r p y Ori
  qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);  // joint pos

  vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);
  vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));
  vMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

  const auto& model = pinocchioInterfaceMeasured_.getModel();
  auto& data = pinocchioInterfaceMeasured_.getData();

  // For floating base EoM task
  pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
  pinocchio::computeJointJacobians(model, data);  // data.J
  pinocchio::updateFramePlacements(model, data);  // placement of all operational frames
  pinocchio::crba(model, data, qMeasured_); // composite rigid body algorithm

  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();  // mass matrix
  pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);

  j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);  // support jacobian

  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) 
  {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>(); // update support jacobian
  }

  // For not contact motion task
  pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);

  dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum); // time derivative of support jacobian

  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) 
  {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();  // update time derivative of support jacobian
  }
}

void WbcBase::updateDesired(const vector_t& stateDesired, const vector_t& inputDesired) 
{
  const auto& model = pinocchioInterfaceDesired_.getModel();
  auto& data = pinocchioInterfaceDesired_.getData();

  mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);
  const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired); // 包括 x y z r p y 和 12个joint_pos
  pinocchio::forwardKinematics(model, data, qDesired);
  pinocchio::computeJointJacobians(model, data, qDesired);
  pinocchio::updateFramePlacements(model, data);
  updateCentroidalDynamics(pinocchioInterfaceDesired_, info_, qDesired);
  const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);
  pinocchio::forwardKinematics(model, data, qDesired, vDesired);
}

// dynamic consistency
Task WbcBase::formulateFloatingBaseEomTask() 
{
  auto& data = pinocchioInterfaceMeasured_.getData();

  matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);  // selection matrix 12 * 18
  s.block(0, 0, info_.actuatedDofNum, 6).setZero(); // 6个under-actuated DOF为0
  s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();  // 12个actuated DOF为Identity Matrix

  // M * ddot_q - J_T * F - S_T * tau = -h
  matrix_t a = (matrix_t(info_.generalizedCoordinatesNum, numDecisionVars_) << data.M, -j_.transpose(), -s.transpose()).finished();
  vector_t b = -data.nle;

  return {a, b, matrix_t(), vector_t()};
}

// torque limits
Task WbcBase::formulateTorqueLimitsTask() 
{
  matrix_t d(2 * info_.actuatedDofNum, numDecisionVars_); // 24 * 30
  d.setZero();
  matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);  // identity matrix
  d.block(0, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts,
          info_.actuatedDofNum, info_.actuatedDofNum) = i;

  d.block(info_.actuatedDofNum, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, 
          info_.actuatedDofNum, info_.actuatedDofNum) = -i;

  vector_t f(2 * info_.actuatedDofNum);
  for (size_t l = 0; l < 2 * info_.actuatedDofNum / 3; ++l) // 2 * 12 / 3 = 8
  {
    f.segment<3>(3 * l) = torqueLimits_;  // -tau_limit < d * x < tau_limit
  }

  return {matrix_t(), vector_t(), d, f};
}

// contact motion constraint
Task WbcBase::formulateNoContactMotionTask() 
{
  matrix_t a(3 * numContacts_, numDecisionVars_); // 12 * 30
  vector_t b(a.rows()); // 12
  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; i++) 
  {
    if (contactFlag_[i])  // only constraint contact foot
    {
      // J_i 为 3*6， j_ = matrix_t(3 * 4, 6);
      a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
      // dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);  (3,6) * (6,1), 再把4个stack在一起
      b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
      j++;
    }
  }

  return {a, b, matrix_t(), vector_t()};
}

// contact force limits
Task WbcBase::formulateFrictionConeTask() 
{
  matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_); // 一般行就是需要约束的状态维度，列是所有状态的维度，因为要右乘上状态变量
  a.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) 
  {
    if (!contactFlag_[i]) 
    {
      a.block(3 * j++, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3); // 没有接触的腿 足端力为0
    }
  }
  vector_t b(a.rows());
  b.setZero();

  matrix_t frictionPyramic(5, 3);  // clang-format off
  frictionPyramic << 0, 0, -1,  // 竖直方向的力不能超过上限 normal direction
                    // 摩擦锥约束
                     1, 0, -frictionCoeff_,   // heading direction
                    -1, 0, -frictionCoeff_,
                     0, 1, -frictionCoeff_, // lateral direction
                     0,-1, -frictionCoeff_;  // clang-format on

  matrix_t d(5 * numContacts_ + 3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  d.setZero();
  j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (contactFlag_[i]) {
      d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = frictionPyramic;
    }
  }
  vector_t f = Eigen::VectorXd::Zero(d.rows());

  return {a, b, d, f};
}

// desired torso momentum
Task WbcBase::formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period) {
  matrix_t a(6, numDecisionVars_);  // 需要约束的自由度为6
  a.setZero();
  a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6); // 追踪浮动基座6个自由度的运动

  vector_t jointAccel = centroidal_model::getJointVelocities(inputDesired - inputLast_, info_) / period;
  inputLast_ = inputDesired;
  mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);

  const auto& model = pinocchioInterfaceDesired_.getModel();
  auto& data = pinocchioInterfaceDesired_.getData();
  const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
  const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);

  const auto& A = getCentroidalMomentumMatrix(pinocchioInterfaceDesired_);

  const Matrix6 Ab = A.template leftCols<6>();  // 左边6列是base的CentrodialMomentumMatrix
  const auto AbInv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);

  const auto Aj = A.rightCols(info_.actuatedDofNum);  // 右边12列关节的 CentrodialMomentumMatrix
  const auto ADot = pinocchio::dccrba(model, data, qDesired, vDesired);
  Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(pinocchioInterfaceDesired_, info_, inputDesired);
  centroidalMomentumRate.noalias() -= ADot * vDesired;
  centroidalMomentumRate.noalias() -= Aj * jointAccel;

  Vector6 b = AbInv * centroidalMomentumRate;

  return {a, b, matrix_t(), vector_t()};
}

// swing foot motion acceleration tracking
Task WbcBase::formulateSwingLegTask() 
{
  eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);  // measured state
  std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());

  eeKinematics_->setPinocchioInterface(pinocchioInterfaceDesired_); // desired state
  std::vector<vector3_t> posDesired = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> velDesired = eeKinematics_->getVelocity(vector_t(), vector_t());

  matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) 
  {
    if (!contactFlag_[i]) 
    {
      vector3_t accel = swingKp_ * (posDesired[i] - posMeasured[i]) + swingKd_ * (velDesired[i] - velMeasured[i]);

      a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum); // foot_i的jacobian
      b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_; // 没看懂这边是什么意思
      j++;
    }
  }

  return {a, b, matrix_t(), vector_t()};
}

// track contact force
Task WbcBase::formulateContactForceTask(const vector_t& inputDesired) const 
{
  matrix_t a(3 * info_.numThreeDofContacts, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();

  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
  }
  b = inputDesired.head(a.rows());

  return {a, b, matrix_t(), vector_t()};
}

void WbcBase::loadTasksSetting(const std::string& taskFile, bool verbose) {
  // Load task file
  torqueLimits_ = vector_t(info_.actuatedDofNum / 4); // 3
  loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", torqueLimits_);
  if (verbose) {
    std::cerr << "\n #### Torque Limits Task:";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "\n #### HAA HFE KFE: " << torqueLimits_.transpose() << "\n";
    std::cerr << " #### =============================================================================\n";
  }
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "frictionConeTask.";
  if (verbose) {
    std::cerr << "\n #### Friction Cone Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, frictionCoeff_, prefix + "frictionCoefficient", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }
  prefix = "swingLegTask.";
  if (verbose) {
    std::cerr << "\n #### Swing Leg Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, swingKp_, prefix + "kp", verbose);
  loadData::loadPtreeValue(pt, swingKd_, prefix + "kd", verbose);
}

}  // namespace legged
