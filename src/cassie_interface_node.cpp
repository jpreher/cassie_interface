/*
 * MIT License
 * 
 * Copyright (c) 2020 Jenna Reher (jreher@caltech.edu)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <string>
#include <fstream>

// Agility includes
#include <cassie_interface/udp.h>
#include <cassie_interface/cassie_out_t.h>
#include <cassie_interface/cassie_user_in_t.h>

// ROS includes
#include <ros/ros.h>
#include <ros_utilities/timing.hpp>
#include <control_eigen_utilities/limits.hpp>
#include <cassie_description/cassie_model.hpp>
#include <cassie_estimation/heelspring_solver.hpp>
#include <cassie_estimation/contact_classifier.hpp>
#include <yaml_eigen_utilities/yaml_eigen_utilities.hpp>
#include <cassie_estimation/kinematics_hip_velocity_estimator.hpp>
#include <cassie_common_toolbox/RadioButtonMap.hpp>

// Load in message types
#include <cassie_common_toolbox/callback_helpers.hpp>
#include <cassie_common_toolbox/cassie_control_msg.h>
#include <cassie_common_toolbox/cassie_proprioception_msg.h>


using namespace cassie_model;

// Variable for control
static VectorXd u(10);
static ros_utilities::Timer timeout_timer(true);

// Callback for controller subscriber
void controller_callback(const cassie_common_toolbox::cassie_control_msg::ConstPtr& controlmsg) {
    timeout_timer.restart();
    for (unsigned long i=0; i<10; i++)
    {
        u[i] = controlmsg->motor_torque[i];
    }
}

// Main node
int main(int argc, char *argv[])
{
    // Establish the current ROS node and associated timing
    ros::init(argc, argv, "cassie_interface");
    ros::NodeHandle nh("/cassie/interface");

    // Check if the simulated joystick command is enabled!
    bool isSim = false;
    ros::param::get("/cassie/is_simulation", isSim);
    timeout_timer.start();

    // Get the safe torque limit from the parameter server
    std::string stl;
    VectorXd safe_torque_limit(10);
    ros::param::get("/cassie/interface/safe_torque_limit", stl);
    yaml_utilities::yaml_read_string(stl, safe_torque_limit);
    std::cout << "Using safe torque limit: " << safe_torque_limit.transpose() << std::endl;

    // Option variables and flags
    std::string remote_addr_str, remote_port_str, iface_addr_str, iface_port_str;
    if (isSim) {
        ROS_INFO("Using a simulated environment... Connecting to Gazebo Cassie network");
        remote_addr_str = "127.0.0.1";
        remote_port_str = "25000";
        iface_addr_str = "0.0.0.0";
        iface_port_str = "25001";
    } else {
        ROS_INFO("Using a hardware environment... Connecting to Cassie network");
        remote_addr_str = "10.10.10.3";
        remote_port_str = "25000";
        iface_addr_str = "10.10.10.105";
        iface_port_str = "25001";
    }

    // Setup ROS publisher/subscriber networks
    ros::Publisher proprioception_pub = nh.advertise<cassie_common_toolbox::cassie_proprioception_msg>("/cassie_proprioception", 1);
    ros::Subscriber controller_sub = nh.subscribe("/cassie_control", 1, controller_callback, ros::TransportHints().tcpNoDelay(true));
    cassie_common_toolbox::cassie_proprioception_msg proprioception_msg;

    // Bind to network interface
    std::cout << "Bind to network at: " << (const char *)remote_addr_str.c_str() <<std::endl;
    int sock = udp_init_client(remote_addr_str.c_str(),
                               remote_port_str.c_str(),
                               iface_addr_str.c_str(),
                               iface_port_str.c_str());
    if (-1 == sock)
        exit(EXIT_FAILURE);

    // Create packet input/output buffers
    const int dinlen = CASSIE_OUT_T_PACKED_LEN;
    const int doutlen = CASSIE_USER_IN_T_PACKED_LEN;
    const int recvlen = PACKET_HEADER_LEN + dinlen;
    const int sendlen = PACKET_HEADER_LEN + doutlen;
    unsigned char *recvbuf = new unsigned char[recvlen];
    unsigned char *sendbuf = new unsigned char[sendlen];

    // Separate input/output buffers into header and payload
    const unsigned char *header_in = recvbuf;
    const unsigned char *data_in = &recvbuf[PACKET_HEADER_LEN];
    unsigned char *header_out = sendbuf;
    unsigned char *data_out = &sendbuf[PACKET_HEADER_LEN];

    // Create standard input/output structs
    cassie_user_in_t cassie_user_in = {0};
    cassie_out_t cassie_out;

    // Create header information struct
    packet_header_info_t header_info = {0};

    // Prepare initial null command packet to start communication
    printf("Running the interface node...\n");
    memset(sendbuf, 0, sendlen);
    bool received_data = false;

    // Run our own characterization of the shin and heel spring offsets
    cassie_model::Cassie robot;
    ContactClassifier contact(nh, robot, 0.0005);
    HeelspringSolver achillesSolver(nh, robot);
    KinematicsHipVelocityEstimator velocityEstimator(nh, robot, true);

    // Whether to log data
    VectorXd log = VectorXd::Zero(43);
    std::fstream logfile;
    bool log_estimation = false;
    ros::param::get("/cassie/log_estimation", log_estimation);
    if ( log_estimation ) {
        std::string home=getenv("HOME");
        std::string path= home+"/datalog/estimation_log.bin";
        logfile.open(path, std::ios::out | std::ios::binary);
    }

    // Listen/respond loop
    while (ros::ok()) {
        // Spin ROS once to get updated control values
        ros::spinOnce();

        // Check the timeout and see if the torque must be zeroed
        double timeout_duration = 0.01; // seconds
        if (timeout_timer.elapsed() > timeout_duration) {
            for (unsigned long i=0; i<10; i++)
                u[i] = 0.0;
        } else {
            // Apply saturation
            control_eigen_utilities::clamp(u, -safe_torque_limit, safe_torque_limit);
        }

        if (isSim) {
            // Check for data
            if (!received_data) {
                // Send null commands until the simulator responds
                ssize_t nbytes;
                do {
                    send_packet(sock, sendbuf, sendlen, nullptr, 0);
                    usleep(1000);
                    nbytes = get_newest_packet(sock, recvbuf, recvlen, nullptr, nullptr);
                } while (recvlen != nbytes);
                received_data = true;
                printf("Connected!\n\n");
            } else {
                // Wait for a new packet
                wait_for_packet(sock, recvbuf, recvlen, nullptr, nullptr);
            }
        } else {
            wait_for_packet(sock, recvbuf, recvlen, nullptr, nullptr);
        }

        // Process incoming header and write outgoing header
        process_packet_header(&header_info, header_in, header_out);

        // Unpack received data into cassie user input struct
        unpack_cassie_out_t(data_in, &cassie_out);

        for (unsigned int i=0; i<10; i++)
        {
            cassie_user_in.torque[i] = u[i];
        }
        pack_cassie_user_in_t(&cassie_user_in, data_out);

        // Send response
        send_packet(sock, sendbuf, sendlen, nullptr, 0);

        // Pack and publish the proprioception data
        proprioception_msg.orientation.w = cassie_out.pelvis.vectorNav.orientation[0];
        proprioception_msg.orientation.x = cassie_out.pelvis.vectorNav.orientation[1];
        proprioception_msg.orientation.y = cassie_out.pelvis.vectorNav.orientation[2];
        proprioception_msg.orientation.z = cassie_out.pelvis.vectorNav.orientation[3];
        proprioception_msg.angular_velocity.x = cassie_out.pelvis.vectorNav.angularVelocity[0];
        proprioception_msg.angular_velocity.y = cassie_out.pelvis.vectorNav.angularVelocity[1];
        proprioception_msg.angular_velocity.z = cassie_out.pelvis.vectorNav.angularVelocity[2];
        proprioception_msg.linear_acceleration.x = cassie_out.pelvis.vectorNav.linearAcceleration[0];
        proprioception_msg.linear_acceleration.y = cassie_out.pelvis.vectorNav.linearAcceleration[1];
        proprioception_msg.linear_acceleration.z = cassie_out.pelvis.vectorNav.linearAcceleration[2];
        proprioception_msg.motor_torque[0] = cassie_out.leftLeg.hipRollDrive.torque;
        proprioception_msg.motor_torque[1] = cassie_out.leftLeg.hipYawDrive.torque;
        proprioception_msg.motor_torque[2] = cassie_out.leftLeg.hipPitchDrive.torque;
        proprioception_msg.motor_torque[3] = cassie_out.leftLeg.kneeDrive.torque;
        proprioception_msg.motor_torque[4] = cassie_out.leftLeg.footDrive.torque;
        proprioception_msg.motor_torque[5] = cassie_out.rightLeg.hipRollDrive.torque;
        proprioception_msg.motor_torque[6] = cassie_out.rightLeg.hipYawDrive.torque;
        proprioception_msg.motor_torque[7] = cassie_out.rightLeg.hipPitchDrive.torque;
        proprioception_msg.motor_torque[8] = cassie_out.rightLeg.kneeDrive.torque;
        proprioception_msg.motor_torque[9] = cassie_out.rightLeg.footDrive.torque;
        proprioception_msg.encoder_position[0] = cassie_out.leftLeg.hipRollDrive.position;
        proprioception_msg.encoder_position[1] = cassie_out.leftLeg.hipYawDrive.position;
        proprioception_msg.encoder_position[2] = cassie_out.leftLeg.hipPitchDrive.position;
        proprioception_msg.encoder_position[3] = cassie_out.leftLeg.kneeDrive.position;
        proprioception_msg.encoder_position[4] = cassie_out.leftLeg.shinJoint.position;
        proprioception_msg.encoder_position[5] = cassie_out.leftLeg.tarsusJoint.position;
        proprioception_msg.encoder_position[6] = cassie_out.leftLeg.footJoint.position;
        proprioception_msg.encoder_position[7] = cassie_out.rightLeg.hipRollDrive.position;
        proprioception_msg.encoder_position[8] = cassie_out.rightLeg.hipYawDrive.position;
        proprioception_msg.encoder_position[9] = cassie_out.rightLeg.hipPitchDrive.position;
        proprioception_msg.encoder_position[10] = cassie_out.rightLeg.kneeDrive.position;// + 0.03;
        proprioception_msg.encoder_position[11] = cassie_out.rightLeg.shinJoint.position;// + 0.02;
        proprioception_msg.encoder_position[12] = cassie_out.rightLeg.tarsusJoint.position;
        proprioception_msg.encoder_position[13] = cassie_out.rightLeg.footJoint.position;
        proprioception_msg.encoder_velocity[0] = cassie_out.leftLeg.hipRollDrive.velocity;
        proprioception_msg.encoder_velocity[1] = cassie_out.leftLeg.hipYawDrive.velocity;
        proprioception_msg.encoder_velocity[2] = cassie_out.leftLeg.hipPitchDrive.velocity;
        proprioception_msg.encoder_velocity[3] = cassie_out.leftLeg.kneeDrive.velocity;
        proprioception_msg.encoder_velocity[4] = cassie_out.leftLeg.shinJoint.velocity;
        proprioception_msg.encoder_velocity[5] = cassie_out.leftLeg.tarsusJoint.velocity;
        proprioception_msg.encoder_velocity[6] = cassie_out.leftLeg.footJoint.velocity;
        proprioception_msg.encoder_velocity[7] = cassie_out.rightLeg.hipRollDrive.velocity;
        proprioception_msg.encoder_velocity[8] = cassie_out.rightLeg.hipYawDrive.velocity;
        proprioception_msg.encoder_velocity[9] = cassie_out.rightLeg.hipPitchDrive.velocity;
        proprioception_msg.encoder_velocity[10] = cassie_out.rightLeg.kneeDrive.velocity;
        proprioception_msg.encoder_velocity[11] = cassie_out.rightLeg.shinJoint.velocity;
        proprioception_msg.encoder_velocity[12] = cassie_out.rightLeg.tarsusJoint.velocity;
        proprioception_msg.encoder_velocity[13] = cassie_out.rightLeg.footJoint.velocity;

        // Get encoder positions
        robot.q.setZero();
        robot.dq.setZero();
        get_proprioception_encoders(proprioception_msg, robot.q, robot.dq);
        get_proprioception_orientation(proprioception_msg, robot.q, robot.dq, robot.quat_pelvis);

        if (cassie_out.isCalibrated) {
            // Update contact / achilles and export
            achillesSolver.update();
            contact.update();

            // Run the velocity estimation
            velocityEstimator.update();
            proprioception_msg.linear_velocity.x = robot.dq(BasePosX);
            proprioception_msg.linear_velocity.y = robot.dq(BasePosY);
            proprioception_msg.linear_velocity.z = robot.dq(BasePosZ);

            proprioception_msg.q_achilles[0] = robot.q(LeftHeelSpring);
            proprioception_msg.q_achilles[1] = robot.q(RightHeelSpring);
            proprioception_msg.dq_achilles[0] = 0.; // The velocities are quite violent, set to zero.
            proprioception_msg.dq_achilles[1] = 0.;
            proprioception_msg.contact[0] = robot.leftContact;
            proprioception_msg.contact[1] = robot.rightContact;

            if (isSim) {
                // Robot is in simulation
                cassie_out.pelvis.radio.channel[SA] = 1.0;
                cassie_out.pelvis.radio.channel[SB] = 0.0;
                cassie_out.pelvis.radio.channel[LS] = 1.0;

                // Don't use springs right away
                if (ros::Time::now().toSec() < 1.0) {
                    proprioception_msg.contact[0] = 0.;
                    proprioception_msg.contact[1] = 0.;
                }

                // Cycle dynamic crouches
                double ts = ros::Time::now().toSec();
                double crouch_wait_sec = 5.0;
                double updown_duration = 2 * crouch_wait_sec;
                if (ros::Time::now().toSec() > 15.0) {
                    cassie_out.pelvis.radio.channel[SH] = -1.0;
                    //if ( fmod(ts,updown_duration) > crouch_wait_sec )
                    //    cassie_out.pelvis.radio.channel[LS] = 0.0;
                }

            }

            // Re-update the message for shins in case achillesSolver added an offset.
            proprioception_msg.encoder_position[4]  = robot.q(LeftShinPitch);
            proprioception_msg.encoder_position[11] = robot.q(RightShinPitch);

            // Radio is all zero until the robot is calibrated. Extra safety precaution.
            for (unsigned int i=0; i<16; i++)
                proprioception_msg.radio[i] = cassie_out.pelvis.radio.channel[i];

            // Add header
            proprioception_msg.header.stamp = ros::Time::now();
            proprioception_pub.publish(proprioception_msg);

            // Write log
            if ( log_estimation ) {
                log << ros::Time::now().toSec(),                                                                                                                // 1
                        proprioception_msg.orientation.w, proprioception_msg.orientation.x, proprioception_msg.orientation.y, proprioception_msg.orientation.z, // 4
                        proprioception_msg.angular_velocity.x, proprioception_msg.angular_velocity.y, proprioception_msg.angular_velocity.z,                    // 3
                        proprioception_msg.linear_velocity.x, proprioception_msg.linear_velocity.y, proprioception_msg.linear_velocity.z,                       // 3
                        Map<VectorXd>(proprioception_msg.encoder_position.data(), proprioception_msg.encoder_position.size()),                                  // 14
                        Map<VectorXd>(proprioception_msg.encoder_velocity.data(), proprioception_msg.encoder_position.size()),                                  // 14
                        Map<VectorXd>(proprioception_msg.q_achilles.data(), proprioception_msg.q_achilles.size()),                                              // 2
                        Map<VectorXd>(proprioception_msg.contact.data(), proprioception_msg.contact.size());                                                    // 2
                logfile.write(reinterpret_cast<char *>(log.data()), (log.size())*sizeof(double));
            }
        }
    }
    if ( log_estimation ) {
        logfile.close();
    }
}
