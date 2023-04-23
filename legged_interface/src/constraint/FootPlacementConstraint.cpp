#include "legged_interface/constraint/FootPlacementConstraint.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>


namespace ocs2{
namespace legged_robot{

FootPlacementConstraint::FootPlacementConstraint(
    const SwitchedModelReferenceManager& referenceManager, 
    const EndEffectorKinematics<scalar_t>& EndEffectorKinematics, 
    size_t contactPointIndex)    
    : StateInputConstraint(ConstraintOrder::Quadratic),   
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(EndEffectorKinematics.clone()),
      contactPointIndex_(contactPointIndex)
{
    if(endEffectorKinematicsPtr_->getIds().size() != 1)
    {
        throw std::runtime_error("[FootPlacementConstraint] class only accepts a single end-effector!");
    }
}

bool FootPlacementConstraint::isActive(scalar_t time) const
{
    const scalar_t trotGaitWholeCycle = T_trot;   
    const scalar_t currentTime = referenceManagerPtr_->get_currentTime_userdefined(); // Start time of the optimization horizon
    assert(time >= currentTime);
    assert(time <= currentTime + trotGaitWholeCycle);
    
    return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

FootPlacementConstraint::FootPlacementConstraint(const FootPlacementConstraint& rhs)
    : StateInputConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_)
{}

vector_t FootPlacementConstraint::getValue(scalar_t time, const vector_t& state, 
    const vector_t& input, const PreComputation& preComp) const
{
    const scalar_t trotGaitWholeCycle = T_trot; 
    const scalar_t currentTime = referenceManagerPtr_->get_currentTime_userdefined(); // Start time of the optimization horizon

    assert(time >= currentTime);
    assert(time <= currentTime + trotGaitWholeCycle);

    // 5. get swing time left 0.35~0
    scalar_t swingTimeLeft;
    auto eventTimes = referenceManagerPtr_->getModeSchedule().eventTimes;
    const size_t swingEndIndex = std::lower_bound(eventTimes.begin(), eventTimes.end(), time) - eventTimes.begin();
    swingTimeLeft = eventTimes[swingEndIndex] - time;
    // std::cout << "swingTimeLeft: " << swingTimeLeft << std::endl;

    assert(swingTimeLeft >= 0);
    // assert(swingTimeLeft <= trotGaitWholeCycle / 2);

    Eigen::Matrix<double, Num_Vertex, 3> A;
    Eigen::Matrix<double, Num_Vertex, 1> b;
    A = referenceManagerPtr_->get_FootPlacement_A(contactPointIndex_);
    b = referenceManagerPtr_->get_FootPlacement_b(contactPointIndex_);

    Eigen::Matrix<scalar_t, Num_Vertex, 1> s;
    s << swingTimeLeft, swingTimeLeft, swingTimeLeft, swingTimeLeft,
         swingTimeLeft, swingTimeLeft, swingTimeLeft, swingTimeLeft;
    s *= 5;

    // 6. Add constraint Ax + b + s >= 0
    vector3_t pEE_world = endEffectorKinematicsPtr_->getPosition(state).front();

    // std::cout << "pEE_world " << contactPointIndex_ <<" : " << pEE_world << std::endl;
    std::cout << "constraint value " << contactPointIndex_ <<" : " << A * pEE_world + b + s << std::endl;

    return A * pEE_world + b + s;
}

VectorFunctionLinearApproximation FootPlacementConstraint::getLinearApproximation(scalar_t time, const vector_t& state, 
        const vector_t& input, const PreComputation& preComp) const
{
    throw std::runtime_error(
        "[FootPlacementConstraint::getLinearApproximation] Linear approximation "
        "not implemented!");
}

VectorFunctionQuadraticApproximation FootPlacementConstraint::getQuadraticApproximation(scalar_t time, const vector_t& state, 
    const vector_t& input, const PreComputation& preComp) const
{
    const scalar_t trotGaitWholeCycle = T_trot; 
    const scalar_t currentTime = referenceManagerPtr_->get_currentTime_userdefined(); // Start time of the optimization horizon

    assert(time >= currentTime);
    assert(time <= currentTime + trotGaitWholeCycle);

    Eigen::Matrix<double, Num_Vertex, 3> A;
    Eigen::Matrix<double, Num_Vertex, 1> b;
    A = referenceManagerPtr_->get_FootPlacement_A(contactPointIndex_);
    b = referenceManagerPtr_->get_FootPlacement_b(contactPointIndex_);

    // std::cout << "A: " << std::endl << A << std::endl;
    // std::cout << "b: " << std::endl << b << std::endl;

    // 5. get swing time left 0.35~0
    scalar_t swingTimeLeft;
    auto eventTimes = referenceManagerPtr_->getModeSchedule().eventTimes;
    const size_t swingEndIndex = std::lower_bound(eventTimes.begin(), eventTimes.end(), time) - eventTimes.begin();
    swingTimeLeft = eventTimes[swingEndIndex] - time;
    // std::cout << "swingTimeLeft: " << swingTimeLeft << std::endl;

    assert(swingTimeLeft >= 0);
    // assert(swingTimeLeft <= trotGaitWholeCycle / 2);

    Eigen::Matrix<scalar_t, Num_Vertex, 1> s;
    s << swingTimeLeft, swingTimeLeft, swingTimeLeft, swingTimeLeft,
         swingTimeLeft, swingTimeLeft, swingTimeLeft, swingTimeLeft;
    s *= 5;

    VectorFunctionLinearApproximation positionLinearApproximation = 
        endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();

    const auto numConstraints = getNumConstraints(time);

    VectorFunctionQuadraticApproximation quadraticApproximation;
    quadraticApproximation.setZero(numConstraints, state.rows(), input.rows());

    quadraticApproximation.f = A * positionLinearApproximation.f + b + s;


    // std::cout << "quadraticApproximation.f: " << quadraticApproximation.f << std::endl;

    quadraticApproximation.dfdx = A * positionLinearApproximation.dfdx;

    // std::cout << "quadraticApproximation.dfdx: " << quadraticApproximation.dfdx << std::endl;

    return quadraticApproximation;    
}

}
}




