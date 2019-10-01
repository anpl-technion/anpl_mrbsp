//
// Created by andrej on 11/15/17.
//


/*
        void integrateAndPublish(const tf::Transform& delta_transform, const ros::Time& timestamp)
        {
            if (sensor_frame_id_.empty())
            {
                ROS_ERROR("[odometer] update called with unknown sensor frame id!");
                return;
            }
            if (timestamp < last_update_time_)
            {
                ROS_WARN("[odometer] saw negative time change in incoming sensor data, resetting pose.");
                integrated_pose_.setIdentity();
                tf_listener_.clear();
            }
            integrated_pose_ *= delta_transform;

            // transform integrated pose to base frame
            tf::StampedTransform base_to_sensor;
            std::string error_msg;
            if (tf_listener_.canTransform(base_link_frame_id_, sensor_frame_id_, timestamp, &error_msg))
            {
                tf_listener_.lookupTransform(
                        base_link_frame_id_,
                        sensor_frame_id_,
                        timestamp, base_to_sensor);
            }
            else
            {
                ROS_WARN_THROTTLE(10.0, "The tf from '%s' to '%s' does not seem to be available, "
                                          "will assume it as identity!",
                                  base_link_frame_id_.c_str(),
                                  sensor_frame_id_.c_str());
                ROS_DEBUG("Transform error: %s", error_msg.c_str());
                base_to_sensor.setIdentity();
            }

            tf::Transform base_transform = base_to_sensor * integrated_pose_ * base_to_sensor.inverse();

            nav_msgs::Odometry odometry_msg;
            odometry_msg.header.stamp = timestamp;
            odometry_msg.header.frame_id = odom_frame_id_;
            odometry_msg.child_frame_id = base_link_frame_id_;
            tf::poseTFToMsg(base_transform, odometry_msg.pose.pose);

            // calculate twist (not possible for first run as no delta_t can be computed)
            tf::Transform delta_base_transform = base_to_sensor * delta_transform * base_to_sensor.inverse();
            if (!last_update_time_.isZero())
            {
                double delta_t = (timestamp - last_update_time_).toSec();
                if (delta_t)
                {
                    odometry_msg.twist.twist.linear.x = delta_base_transform.getOrigin().getX() / delta_t;
                    odometry_msg.twist.twist.linear.y = delta_base_transform.getOrigin().getY() / delta_t;
                    odometry_msg.twist.twist.linear.z = delta_base_transform.getOrigin().getZ() / delta_t;
                    tf::Quaternion delta_rot = delta_base_transform.getRotation();
                    tfScalar angle = delta_rot.getAngle();
                    tf::Vector3 axis = delta_rot.getAxis();
                    tf::Vector3 angular_twist = axis * angle / delta_t;
                    odometry_msg.twist.twist.angular.x = angular_twist.x();
                    odometry_msg.twist.twist.angular.y = angular_twist.y();
                    odometry_msg.twist.twist.angular.z = angular_twist.z();
                }
            }

            odometry_msg.pose.covariance = pose_covariance_;
            odometry_msg.twist.covariance = twist_covariance_;
            odom_pub_.publish(odometry_msg);

            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.stamp = odometry_msg.header.stamp;
            pose_msg.header.frame_id = odometry_msg.header.frame_id;
            pose_msg.pose = odometry_msg.pose.pose;

            pose_pub_.publish(pose_msg);

            if (publish_tf_)
            {
                if (invert_tf_)
                {
                    tf_broadcaster_.sendTransform(
                            tf::StampedTransform(base_transform.inverse(), timestamp,
                                                 base_link_frame_id_, odom_frame_id_));
                }
                else
                {
                    tf_broadcaster_.sendTransform(
                            tf::StampedTransform(base_transform, timestamp,
                                                 odom_frame_id_, base_link_frame_id_));
                }
            }

            last_update_time_ = timestamp;
        }

*/
