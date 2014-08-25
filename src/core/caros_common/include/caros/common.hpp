#ifndef CAROS_COMMON_H_
#define CAROS_COMMON_H_

#include <rw/math.hpp>

#include <caros_common_msgs/Q.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

/**
 * \brief CAROS specific functionality
 */
namespace caros {
    /**
     * \addtogroup TypeConversion CAROS Type Conversion
     * Overloaded utility functions for converting between the different system types (e.g. from ROS to RobWork)
     * @{
     */

    //! convert Q to Q
    rw::math::Q toRw(const caros_common_msgs::Q& q);

    //! convert Q to Q
    caros_common_msgs::Q toRos(const rw::math::Q& q);

    //! convert Transform3D to Transform
    geometry_msgs::Transform toRos(const rw::math::Transform3D<>& transform);

    //! convert Transform to Transform3D
    rw::math::Transform3D<> toRw(const geometry_msgs::Transform& transform);

    //! convert Pose to Transform3D
    rw::math::Transform3D<> toRw(const geometry_msgs::Pose& transform);

    //! convert Wrench to Wrench6D
    rw::math::Wrench6D<> toRw(const geometry_msgs::Wrench& wrench);

    //! convert Wrench6D to Wrench
    geometry_msgs::Wrench toRos(const rw::math::Wrench6D<>& w);

    //! convert Twist to VelocityScrew6D
    rw::math::VelocityScrew6D<> toRw(const geometry_msgs::Twist& twist);

    //! convert VelocityScrew6D to Twist
    geometry_msgs::Twist toRos(const rw::math::VelocityScrew6D<>& vs);

    /**
     * @} end of group
     */
} // namespace

#endif /* CAROS_COMMON_H_ */
