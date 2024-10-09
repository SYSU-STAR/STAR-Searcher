#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "apriltag_tf_listener");

    ros::NodeHandle nh;

    tf::TransformListener tf_listener;

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("apriltag_pose", 1);

    ros::Rate rate(10.0);
    while (nh.ok())
    {
        tf::StampedTransform transform;
        try
        {
            // Wait for the transform between the camera and apriltag to be available
            tf_listener.waitForTransform("/camera_color_optical_frame", "/tag_0", ros::Time(0), ros::Duration(1.0));
            // Get the transform from camera to apriltag
            tf_listener.lookupTransform("/camera_color_optical_frame", "/tag_0", ros::Time(0), transform);
            // Create a PoseStamped message with the pose information
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "/camera_color_optical_frame";
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = transform.getOrigin().x();
            pose.pose.position.y = transform.getOrigin().y();
            pose.pose.position.z = transform.getOrigin().z();
            pose.pose.orientation.x = transform.getRotation().x();
            pose.pose.orientation.y = transform.getRotation().y();
            pose.pose.orientation.z = transform.getRotation().z();
            pose.pose.orientation.w = transform.getRotation().w();
            // Publish the pose message
            pose_pub.publish(pose);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }

        rate.sleep();
    }

    return 0;
}
