#include <ros/ros.h>

#include "LSPC.h"
#include "MessageTypes.h"
#include <string>
#include <thread>
#include <future>
#include <boost/bind.hpp>

#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
/*#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>*/
#include <geometry_msgs/TransformStamped.h>

/* Include messages */
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"


void LSPC_Callback_StateEstimates(ros::Publisher& pubOdom, tf::TransformBroadcaster& tfBroadcaster, std::shared_ptr<tf::TransformListener> tfListener, const std::vector<uint8_t>& payload)
{
    const lspc::MessageTypesToPC::StateEstimates_t * msg = reinterpret_cast<const lspc::MessageTypesToPC::StateEstimates_t *>(payload.data());

    tf::Quaternion q_attitude;
    q_attitude.setW(msg->q.w);
    q_attitude.setX(msg->q.x);
    q_attitude.setY(msg->q.y);
    q_attitude.setZ(msg->q.z);

    tf::Quaternion dq;
    dq.setW(msg->dq.w);
    dq.setX(msg->dq.x);
    dq.setY(msg->dq.y);
    dq.setZ(msg->dq.z);

    tf::Transform attitudeTf;
    attitudeTf.setIdentity();
    attitudeTf.setRotation(q_attitude);

    /* Lookup static transform between contact_point to ball (will be the height/z-offset from contact point to center of ball */
    tf::Vector3 ball_offset(0,0,0);
    tf::StampedTransform transformStatic;
    try{
        tfListener->lookupTransform("contact_point", "ball", ros::Time(0), transformStatic);
        ball_offset.setX(transformStatic.getOrigin().x());
        ball_offset.setY(transformStatic.getOrigin().y());
        ball_offset.setZ(transformStatic.getOrigin().z());
    }
    catch (tf::TransformException &ex) {
        //ROS_WARN("%s",ex.what());
    }

    /* Send transfrom from "odom" frame to "contact_point" frame */
    geometry_msgs::TransformStamped tf_contact_point_msg;
    tf_contact_point_msg.header.frame_id = "odom";
    tf_contact_point_msg.child_frame_id = "contact_point";
    tf_contact_point_msg.header.stamp = ros::Time::now();
    tf_contact_point_msg.transform.translation.x = msg->pos.x; // inertial frame position
    tf_contact_point_msg.transform.translation.y = msg->pos.y;
    tf_contact_point_msg.transform.translation.z = 0.0;
    tf_contact_point_msg.transform.rotation.w = 1;
    tf_contact_point_msg.transform.rotation.x = 0;
    tf_contact_point_msg.transform.rotation.y = 0;
    tf_contact_point_msg.transform.rotation.z = 0;
    tfBroadcaster.sendTransform(tf_contact_point_msg);

    /* Compute heading from received attitude quaternion */
    // We define the heading angle as the angle to the projection of the body (base_link) x-axis, projected down to the 2D plane in the inertial frame
    tf::Vector3 xVector = attitudeTf(tf::Vector3(1,0,0));
    float heading = atan2(xVector.y(), xVector.x());
    tf::Quaternion q_heading = tf::createQuaternionFromYaw(heading); // rotate around z-axis

    /* Send transfrom from "ball" frame to "heading" frame */
    geometry_msgs::TransformStamped tf_heading_msg;
    tf_heading_msg.header.frame_id = "ball";
    tf_heading_msg.child_frame_id = "heading";
    tf_heading_msg.header.stamp = ros::Time::now();
    tf_heading_msg.transform.translation.x = 0;
    tf_heading_msg.transform.translation.y = 0;
    tf_heading_msg.transform.translation.z = 0.0;
    tf_heading_msg.transform.rotation.w = q_heading.w();
    tf_heading_msg.transform.rotation.x = q_heading.x();
    tf_heading_msg.transform.rotation.y = q_heading.y();
    tf_heading_msg.transform.rotation.z = q_heading.z();
    tfBroadcaster.sendTransform(tf_heading_msg);

    /* Compute roll/pitch quaternion which represents the tilt in heading frame */
    // Remember quaternions are multipled from left to right, start with Unit quaternion all the way from left and then right multiplying
    // The combined attitude quaternion is constructed as:
    //   q = q_heading o q_tilt
    //   Since we need to first rotate into heading frame and then tilt from there
    // Hence the tilt quaternion is compute as:
    //   q_tilt = inv(q_heading) o q
    tf::Quaternion tiltQuaternion = q_heading.inverse() * q_attitude;

    /* Send transfrom from "heading" frame to "base_link" frame */
    geometry_msgs::TransformStamped tf_tilt_msg;
    tf_tilt_msg.header.frame_id = "heading";
    tf_tilt_msg.child_frame_id = "base_link";
    tf_tilt_msg.header.stamp = ros::Time::now();
    tf_tilt_msg.transform.translation.x = 0;
    tf_tilt_msg.transform.translation.y = 0;
    tf_tilt_msg.transform.translation.z = 0.0;
    tf_tilt_msg.transform.rotation.w = tiltQuaternion.w();
    tf_tilt_msg.transform.rotation.x = tiltQuaternion.x();
    tf_tilt_msg.transform.rotation.y = tiltQuaternion.y();
    tf_tilt_msg.transform.rotation.z = tiltQuaternion.z();
    tfBroadcaster.sendTransform(tf_tilt_msg);

    /* Compute angular velocity from quaternion derivative */
    // We need the angular velocity in inertial frame
    // dq = 1/2 * q o q_omega_body
    // q_omega_body = 2*inv(q) o dq
    tf::Quaternion q_omega_body = q_attitude.inverse() * dq * 2;

    /* Send odometry message */
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = msg->pos.x; // inertial frame position
    odom_msg.pose.pose.position.y = msg->pos.y;
    odom_msg.pose.pose.position.z = ball_offset.z();
    odom_msg.pose.pose.orientation.w = q_attitude.w();
    odom_msg.pose.pose.orientation.x = q_attitude.x();
    odom_msg.pose.pose.orientation.y = q_attitude.y();
    odom_msg.pose.pose.orientation.z = q_attitude.z();
    odom_msg.twist.twist.linear.x = msg->vel.x; // inertial frame velocity
    odom_msg.twist.twist.linear.y = msg->vel.y;
    odom_msg.twist.twist.angular.x = q_omega_body.x();
    odom_msg.twist.twist.angular.y = q_omega_body.y();
    odom_msg.twist.twist.angular.z = q_omega_body.z();
    pubOdom.publish(odom_msg);
}

void LSPC_Callback_GetParameter(const std::vector<uint8_t>& payload)
{
    std::string message(payload.data(), payload.data() + payload.size());
    ROS_DEBUG_STREAM("Get parameter info received");
}

void LSPC_Callback_SetParameterAck(const std::vector<uint8_t>& payload)
{
    std::string message(payload.data(), payload.data() + payload.size());
    ROS_DEBUG_STREAM("Set parameter acknowledged");
}

void LSPC_Callback_SystemInfo(const std::vector<uint8_t>& payload)
{
    std::string message(payload.data(), payload.data() + payload.size());
    ROS_DEBUG_STREAM("System info received");
}

void LSPC_Callback_ControllerInfo(const std::vector<uint8_t>& payload)
{
    std::string message(payload.data(), payload.data() + payload.size());
    ROS_DEBUG_STREAM("Controller info received");
}

void LSPC_Callback_RawSensor_IMU_MPU9250(const std::vector<uint8_t>& payload)
{
    std::string message(payload.data(), payload.data() + payload.size());
    ROS_DEBUG_STREAM("Received Raw Sensor - IMU MPU9250");
}

void LSPC_Callback_RawSensor_IMU_MTI200(const std::vector<uint8_t>& payload)
{
    std::string message(payload.data(), payload.data() + payload.size());
    ROS_DEBUG_STREAM("Received Raw Sensor - IMU MTI200");
}

void LSPC_Callback_RawSensor_Battery(const std::vector<uint8_t>& payload)
{
    std::string message(payload.data(), payload.data() + payload.size());
    ROS_DEBUG_STREAM("Received Raw Sensor - Battery");
}

void LSPC_Callback_RawSensor_Encoders(const std::vector<uint8_t>& payload)
{
    std::string message(payload.data(), payload.data() + payload.size());
    ROS_DEBUG_STREAM("Received Raw Sensor - Encoders");
}

void LSPC_Callback_CPUload(const std::vector<uint8_t>& payload)
{
    std::string message(payload.data(), payload.data() + payload.size());
    ROS_DEBUG_STREAM("CPU Load:\n" << message);
}

void LSPC_Callback_Debug(const std::vector<uint8_t>& payload)
{
    std::string message(payload.data(), payload.data() + payload.size());
    ROS_DEBUG_STREAM("Debug message:\n" << message);
}

void LSPC_ConnectionThread(boost::shared_ptr<ros::NodeHandle> n, std::string serial_port, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj, std::future<void> exitSignalObj)
{
    /* Configure ROS publishers (based on received LSPC messages) */
    ros::Publisher pub_odom = n->advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster tf_broadcaster; // maybe wrap with unique_ptr ?
    std::shared_ptr<tf::TransformListener> tf_listener = std::make_shared<tf::TransformListener>();

    lspcMutex->lock();
    while (exitSignalObj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout) {
        *lspcObj = new lspc::Socket; // recreate a new LSPC object
        while (!(*lspcObj)->isOpen() && ros::ok()) {
            try {
                ROS_DEBUG("Trying to connect to Kugle");
                (*lspcObj)->open("/dev/kugle");
            }
            catch (boost::system::system_error &e) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        if (!ros::ok()) break;

        ROS_DEBUG("Connected to Kugle");
        lspcMutex->unlock();

        /* Register callbacks */
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::GetParameter, &LSPC_Callback_GetParameter);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::SetParameterAck, &LSPC_Callback_SetParameterAck);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::SystemInfo, &LSPC_Callback_SystemInfo);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::ControllerInfo, &LSPC_Callback_ControllerInfo);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::StateEstimates, boost::bind(&LSPC_Callback_StateEstimates, pub_odom, tf_broadcaster, tf_listener, _1));
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::RawSensor_IMU_MPU9250, &LSPC_Callback_RawSensor_IMU_MPU9250);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::RawSensor_IMU_MTI200, &LSPC_Callback_RawSensor_IMU_MTI200);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::RawSensor_Battery, &LSPC_Callback_RawSensor_Battery);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::RawSensor_Encoders, &LSPC_Callback_RawSensor_Encoders);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::CPUload, &LSPC_Callback_CPUload);
        (*lspcObj)->registerCallback(lspc::MessageTypesToPC::Debug, &LSPC_Callback_Debug);

        while ((*lspcObj)->isOpen() && ros::ok()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (!ros::ok())
            break;

        ROS_WARN("Connection lost to Kugle");
        lspcMutex->lock();
        delete(*lspcObj); // call destructor to disconnect completely and clean up used ressources
    }
}

void ROS_Callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj) {
    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(1))) return; // could not get lock

    if ((*lspcObj)->isOpen()) {
        ROS_DEBUG_STREAM("Sending cmd_vel to Kugle");
        lspc::MessageTypesFromPC::VelocityReference_Heading_t payload;
        payload.vel.x = msg->linear.x;
        payload.vel.y = msg->linear.y;
        payload.vel.yaw = msg->angular.z;
        std::vector<uint8_t> payloadPacked((uint8_t *)&payload, (uint8_t *)&payload+sizeof(payload)); // this method of "serializing" requires that PC runs Little Endian (which most PC processors do = Intel x86, AMD 64 etc.)
        (*lspcObj)->send(lspc::MessageTypesFromPC::VelocityReference_Heading, payloadPacked);
    }

    lspcMutex->unlock();
}

void ROS_Callback_cmd_vel_inertial(const geometry_msgs::Twist::ConstPtr& msg, std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj) {
    if (!lspcMutex->try_lock_for(std::chrono::milliseconds(1))) return; // could not get lock

    if ((*lspcObj)->isOpen()) {
        ROS_DEBUG_STREAM("Sending cmd_vel_inertial to Kugle");
        lspc::MessageTypesFromPC::VelocityReference_Inertial_t payload;
        payload.vel.x = msg->linear.x;
        payload.vel.y = msg->linear.y;
        payload.vel.yaw = msg->angular.z;
        std::vector<uint8_t> payloadPacked((uint8_t *)&payload, (uint8_t *)&payload+sizeof(payload)); // this method of "serializing" requires that PC runs Little Endian (which most PC processors do = Intel x86, AMD 64 etc.)
        (*lspcObj)->send(lspc::MessageTypesFromPC::VelocityReference_Inertial, payloadPacked);
    }

    lspcMutex->unlock();
}

void ROS_Callback_Heartbeat(std::shared_ptr<std::timed_mutex> lspcMutex, std::shared_ptr<lspc::Socket *> lspcObj, std::shared_ptr<ros::Timer> timeoutTimer, const ros::TimerEvent& e) {
    // Maybe use this function to check for reference timeouts etc. (last reference update timestamp)

    /* Reset velocity reference by sending 0 reference to the embedded controller */
    ROS_DEBUG_STREAM("Resetting velocity reference due to timeout");
    if ((*lspcObj)->isOpen()) {
        lspc::MessageTypesFromPC::VelocityReference_Inertial_t payload;
        payload.vel.x = 0;
        payload.vel.y = 0;
        payload.vel.yaw = 0;
        std::vector<uint8_t> payloadPacked((uint8_t *)&payload, (uint8_t *)&payload+sizeof(payload)); // this method of "serializing" requires that PC runs Little Endian (which most PC processors do = Intel x86, AMD 64 etc.)
        (*lspcObj)->send(lspc::MessageTypesFromPC::VelocityReference_Inertial, payloadPacked);
    }
}

int main(int argc, char **argv) {
	std::string nodeName = "kugle_driver";
	ros::init(argc, argv, nodeName.c_str());
    boost::shared_ptr<ros::NodeHandle> n(new ros::NodeHandle);  // ros::NodeHandle n
	ros::NodeHandle nParam("~"); // default/current namespace node handle
	
	ROS_DEBUG_STREAM("Starting Kugle driver...");

	std::string serial_port = "/dev/kugle";
	if (!nParam.getParam("serial_port", serial_port)) {
		ROS_WARN_STREAM("Serial port not set (Parameter: serial_port). Defaults to: /dev/kugle");
	}

	/* Create LSPC connection thread */
    std::shared_ptr<std::timed_mutex> lspcMutex = std::make_shared<std::timed_mutex>();
    std::shared_ptr<lspc::Socket *> lspcObj = std::make_shared<lspc::Socket *>(nullptr);  // read about shared_ptr here: https://www.acodersjourney.com/top-10-dumb-mistakes-avoid-c-11-smart-pointers/

    std::promise<void> exitSignal;
    std::future<void> exitSignalObj = exitSignal.get_future();
    std::thread connectionThread = std::thread(&LSPC_ConnectionThread, n, serial_port, lspcMutex, lspcObj, std::move(exitSignalObj));

    /* Configure subscribers */
    //std::shared_ptr<ros::Timer> reference_timeout_timer = std::make_shared<ros::Timer>(n->createTimer(ros::Duration(0.1), boost::bind(&ROS_Callback_Heartbeat, lspcMutex, lspcObj, reference_timeout_timer, _1))); // Configure 10 Hz heartbeat timer
    ros::Subscriber sub_cmd_vel = n->subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&ROS_Callback_cmd_vel, _1, lspcMutex, lspcObj));
    ros::Subscriber sub_cmd_vel_inertial = n->subscribe<geometry_msgs::Twist>("cmd_vel_inertial", 1, boost::bind(&ROS_Callback_cmd_vel_inertial, _1, lspcMutex, lspcObj));

    ros::spin();

    std::cout << "Shutting down Kugle driver" << std::endl;

    //reference_timeout_timer->stop();

    exitSignal.set_value();
    if (connectionThread.joinable())
        connectionThread.join();
}
