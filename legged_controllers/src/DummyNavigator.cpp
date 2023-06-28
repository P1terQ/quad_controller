#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#define sign(x) ( ((x) <0 )? -1 : ((x)> 0) )
using namespace ocs2;
using namespace ocs2_msgs;
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
scalar_t TARGET_DISPLACEMENT_VELOCITY;
scalar_t TARGET_DISPLACEMENT_VELOCITY_x;
scalar_t TARGET_DISPLACEMENT_VELOCITY_y;
scalar_t TARGET_ROTATION_VELOCITY;

scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) 
{
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(3);
  const scalar_t rotationTime = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY;
  return std::max(rotationTime, displacementTime);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dummy_navigator");
    ros::NodeHandle nh;

    bool if_dummyNavigator_;
    nh.getParam("/if_dummyNavigator", if_dummyNavigator_);

    bool pub_cmd_vel = false;

    ocs2::SystemObservation latestObservation_;
    vector3_t rel_goal;
    rel_goal << 0.0, 0.0, 0.0;
    vector3_t abs_goal;
    abs_goal << 0.0, 0.0, 0.0;

    vector3_t Error_Intergral;
    Error_Intergral << 0.0, 0.0, 0.0;
    vector3_t Error_last;
    Error_last << 0.0, 0.0, 0.0;

    std::string referenceFile;
    nh.getParam("/referenceFile", referenceFile);
    loadData::loadCppDataType(referenceFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
    loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);
    loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity_x", TARGET_DISPLACEMENT_VELOCITY_x);
    loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity_y", TARGET_DISPLACEMENT_VELOCITY_y);

    ros::Publisher dummy_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    geometry_msgs::Twist CMD_MSG;

    auto observationCallback = [&](const mpc_observation::ConstPtr& msg) 
    {
        latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    };
    ros::Subscriber observation_Sub = nh.subscribe<ocs2_msgs::mpc_observation>("legged_robot_mpc_observation",1,observationCallback);

    //todo: 加上 abs_target 和 rel_target 
    auto relgoalCallback = [&](const geometry_msgs::Pose2D::ConstPtr& msg)  //! 这边没加ConstPtr,找了半天的错...
    {   
        std::cout << "rel_target received!" << std::endl;
        abs_goal(0) += msg->x;
        abs_goal(1) += msg->y;
        abs_goal(2) += msg->theta;
        std::cout << "target Pose2D: " << abs_goal << std::endl;

        const vector_t currentPose = latestObservation_.state.segment<6>(6);
        vector3_t currentPose2D;
        currentPose2D << currentPose(0), currentPose(1), currentPose(3);
        scalar_t delta_T = estimateTimeToTarget(abs_goal - currentPose2D);
        std::cout << "time_need: " << delta_T << std::endl;

        TARGET_DISPLACEMENT_VELOCITY_x = abs((abs_goal(0) - currentPose2D(0)) / delta_T);
        TARGET_DISPLACEMENT_VELOCITY_y = abs((abs_goal(1) - currentPose2D(1)) / delta_T);
        TARGET_ROTATION_VELOCITY = abs((abs_goal(2) - currentPose2D(2)) / delta_T);

        Error_Intergral << 0.0, 0.0, 0.0;
        Error_last << 0.0, 0.0, 0.0;

        pub_cmd_vel = true;
    };
    ros::Subscriber relgoal_Sub = nh.subscribe<geometry_msgs::Pose2D>("/dummy_navigator_PTP/rel_goal", 1, relgoalCallback);

    auto absgoalCallback = [&](const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        std::cout << "abs_target received!" << std::endl;
        abs_goal(0) = msg->x;
        abs_goal(1) = msg->y;
        abs_goal(2) = msg->theta;
        std::cout << "target Pose2D: " << abs_goal << std::endl;

        const vector_t currentPose = latestObservation_.state.segment<6>(6);
        vector3_t currentPose2D;
        currentPose2D << currentPose(0), currentPose(1), currentPose(3);
        scalar_t delta_T = estimateTimeToTarget(abs_goal - currentPose2D);
        std::cout << "time_need: " << delta_T << std::endl;

        TARGET_DISPLACEMENT_VELOCITY_x = abs((abs_goal(0) - currentPose2D(0)) / delta_T);
        TARGET_DISPLACEMENT_VELOCITY_y = abs((abs_goal(1) - currentPose2D(1)) / delta_T);
        TARGET_ROTATION_VELOCITY = abs((abs_goal(2) - currentPose2D(2)) / delta_T);

        Error_Intergral << 0.0, 0.0, 0.0;
        Error_last << 0.0, 0.0, 0.0;

        pub_cmd_vel = true;
    };
    ros::Subscriber absgoal_Sub = nh.subscribe<geometry_msgs::Pose2D>("/dummy_navigator_PTP/abs_goal", 1, absgoalCallback);

    while(ros::ok())
    {
        if(if_dummyNavigator_)
        {
            if(pub_cmd_vel)
            {
                const vector_t currentPose = latestObservation_.state.segment<6>(6);
                vector3_t currentPose2D;
                currentPose2D << currentPose(0), currentPose(1), currentPose(3);

                // std::cout << "ok?" << std::endl;
                scalar_t delta_x = abs_goal(0) - currentPose2D(0);
                scalar_t delta_y = abs_goal(1) - currentPose2D(1);
                scalar_t delta_theta = abs_goal(2) - currentPose2D(2);

                // Error_Intergral(0) += delta_x;
                // Error_Intergral(1) += delta_y;
                // Error_Intergral(2) += delta_theta;
                // for(int i=0; i<=2; i++)
                // {
                //     if(abs(Error_Intergral(i))>100)
                //     {
                //         Error_Intergral(i) = 100;
                //     }
                // }
                // CMD_MSG.linear.x = 0.5 * delta_x + 0.001 * Error_Intergral(0) + 0.001 * (delta_x - Error_last(0));
                // CMD_MSG.linear.y = 0.5 * delta_y + 0.001 * Error_Intergral(1) + 0.001 * (delta_y - Error_last(1));
                // CMD_MSG.angular.z = 0.5 * delta_theta + 0.001 * Error_Intergral(2) + 0.001 * (delta_theta - Error_last(2));

                vector3_t zyx = currentPose.tail(3);
                //! 这个是在世界坐标系下的速度，如果要发cmd_vel需要转换到机器人坐标系
                // vector_t vel_cmd_world;
                // vector_t vel_cmd_base;
                Eigen::Matrix<scalar_t, 4, 1> vel_cmd_world;
                Eigen::Matrix<scalar_t, 3, 1> vel_cmd_base;

                vel_cmd_world << 0.5 * delta_x, 0.5 * delta_y, 0.0, 0.5 * delta_theta;
                // vel_cmd_world(0.5*delta_x, 0.5 * delta_y, 0.0, 0.5 * delta_theta);
                // vel_cmd_world<<0.5, 0.5, 0.0, 0.5;

                vel_cmd_base = getRotationMatrixFromZyxEulerAngles(zyx).transpose() * vel_cmd_world.head(3);
                CMD_MSG.linear.x = vel_cmd_base(0);
                CMD_MSG.linear.y = vel_cmd_base(1);
                CMD_MSG.angular.z = vel_cmd_world(3);

                if(abs(CMD_MSG.linear.x) > TARGET_DISPLACEMENT_VELOCITY_x)
                {
                    CMD_MSG.linear.x = sign(CMD_MSG.linear.x) * TARGET_DISPLACEMENT_VELOCITY_x;
                }
                if(abs(CMD_MSG.linear.y) > TARGET_DISPLACEMENT_VELOCITY_y)
                {
                    CMD_MSG.linear.y = sign(CMD_MSG.linear.y) * TARGET_DISPLACEMENT_VELOCITY_y;
                }
                if(abs(CMD_MSG.angular.z) > TARGET_ROTATION_VELOCITY)
                {
                    CMD_MSG.angular.z = sign(CMD_MSG.angular.z) * TARGET_ROTATION_VELOCITY;
                }

                std::cout << "target Pose2D: " << abs_goal << std::endl;
                // std::cout << "current Pose2D: " << currentPose2D << std::endl;
                // std::cout << "vel: " <<  CMD_MSG.linear.x << " " << CMD_MSG.linear.y << " " << CMD_MSG.angular.z << std::endl;
                // std::cout << "delta_x: "<<  delta_x <<" vel_x: " << CMD_MSG.linear.x << std::endl;
                // std::cout << "delta_y: "<<  delta_y << "vel_y: " << CMD_MSG.linear.y << std::endl;
                // std::cout << "delta_theta: "<<  delta_theta <<"vel_w: " << CMD_MSG.angular.z << std::endl;
                
                dummy_vel_pub.publish(CMD_MSG);
            }
        }
        else
        {
            return 0;
        }

        ros::spinOnce();
    }

}



