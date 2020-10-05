/*
 * Copyright (c) 2018 Agility Robotics
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
// Modified by Jenna Reher (jreher@caltech.edu)

#include <cassie_interface/CassiePlugin.hpp>


CassiePlugin::CassiePlugin() :
    kMotorBits_{13, 13, 13, 13, 18, 13, 13, 13, 13, 18},
    kGearRatio_{25, 25, 16, 16, 50, 25, 25, 16, 16, 50},
    kMaxSpeed_{2900, 2900, 1300, 1300, 5500, 2900, 2900, 1300, 1300, 5500},
    kMaxTorque_{4.5, 4.5, 12.2, 12.2, 0.9, 4.5, 4.5, 12.2, 12.2, 0.9},
    kMotorOffset_{0, 0, 0, -0.785398, 0, 0, 0, 0, -0.785398, 0},
    kJointBits_{18, 18, 13, 18, 18, 13},
    kJointOffset_{0, 1.012291, 0, 0, 1.012291, 0},
    motorFilterB_{2727, 534, -2658, -795, 72, 110, 19, -6, -3},
    jointFilterB_{12.348, 12.348, -12.348, -12.348},
    jointFilterA_{1.0, -1.7658, 0.79045},
    cassieOut_{},
    headerInfo_{},
    cassieUserIn_{}
{
    static_joint_attached = true;

    // Pointers to each drive struct in cassieOut
    driveOut_ = {
        &cassieOut_.leftLeg.hipRollDrive,
        &cassieOut_.leftLeg.hipYawDrive,
        &cassieOut_.leftLeg.hipPitchDrive,
        &cassieOut_.leftLeg.kneeDrive,
        &cassieOut_.leftLeg.footDrive,
        &cassieOut_.rightLeg.hipRollDrive,
        &cassieOut_.rightLeg.hipYawDrive,
        &cassieOut_.rightLeg.hipPitchDrive,
        &cassieOut_.rightLeg.kneeDrive,
        &cassieOut_.rightLeg.footDrive
    };

    // Pointers to each joint struct in cassieOut
    jointOut_ = {
        &cassieOut_.leftLeg.shinJoint,
        &cassieOut_.leftLeg.tarsusJoint,
        &cassieOut_.leftLeg.footJoint,
        &cassieOut_.rightLeg.shinJoint,
        &cassieOut_.rightLeg.tarsusJoint,
        &cassieOut_.rightLeg.footJoint
    };

    // Initialize cassieOut
    initCassieOut(&cassieOut_);

    // Create cassie core
    corePtr_ = cassie_core_sim_alloc();
    cassie_core_sim_setup(corePtr_);

    // Separate UDP input/output buffers into header and payload
    headerInPtr_ = recvBuf_;
    dataInPtr_ = &recvBuf_[PACKET_HEADER_LEN];
    headerOutPtr_ = sendBuf_;
    dataOutPtr_ = &sendBuf_[PACKET_HEADER_LEN];

    runSim_ = false;
}

CassiePlugin::~CassiePlugin()
{
}

void CassiePlugin::Reset() {
    // Fix pelvis to world
    auto model = this->worldPtr_->ModelByName("cassie");
    gazebo::physics::JointPtr x_joint = model->GetJoint("BasePosX");
    gazebo::physics::JointPtr y_joint = model->GetJoint("BasePosY");
    gazebo::physics::JointPtr z_joint = model->GetJoint("BasePosZ");
    gazebo::physics::JointPtr roll_joint = model->GetJoint("BaseRotX");
    gazebo::physics::JointPtr pitch_joint = model->GetJoint("BaseRotY");
    x_joint->SetLowerLimit(0,0);
    x_joint->SetUpperLimit(0,0);
    y_joint->SetLowerLimit(0,0);
    y_joint->SetUpperLimit(0,0);
    z_joint->SetLowerLimit(0,0);
    z_joint->SetUpperLimit(0,0);
    roll_joint->SetLowerLimit(0,0);
    roll_joint->SetUpperLimit(0,0);
    pitch_joint->SetLowerLimit(0,0);
    pitch_joint->SetUpperLimit(0,0);
    static_joint_attached = true;

    // Reset simulation flag
    runSim_ = false;
}


void CassiePlugin::setMotorEncoder(elmo_out_t *drive, double position,
                                   motor_filter_t *filter, const int bits)
{
    // Position
    // Get digital encoder value
    int encoderValue = position / (2 * M_PI) * (1 << bits);
    double scale     = (2 * M_PI) / (1 << bits);
    drive->position  = encoderValue * scale;

    // Velocity
    // Initialize unfiltered signal array to prevent bad transients
    bool allzero  = true;
    for (size_t i = 0; i < MOTOR_FILTER_NB; ++i)
        allzero  &= filter->x[i] == 0;
    if (allzero) {
        // If all filter values are zero, initialize the signal array
        // with the current encoder value
        for (size_t i = 0; i < MOTOR_FILTER_NB; ++i)
            filter->x[i] = encoderValue;
    }

    // Shift and update unfiltered signal array
    for (size_t i = MOTOR_FILTER_NB - 1; i > 0; --i)
        filter->x[i] = filter->x[i - 1];
    filter->x[0] = encoderValue;

    // Compute filter value
    int y = 0;
    for (size_t i = 0; i < MOTOR_FILTER_NB; ++i)
        y += filter->x[i] * motorFilterB_[i];
    drive->velocity = y * scale / M_PI;
}


void CassiePlugin::setJointEncoder(cassie_joint_out_t *joint, double position,
                                   joint_filter_t *filter, const int bits)
{
    // Position
    // Get digital encoder value
    int encoderValue = position / (2 * M_PI) * (1 << bits);
    double scale     = (2 * M_PI) / (1 << bits);
    joint->position  = encoderValue * scale;

    // Velocity
    // Initialize unfiltered signal array to prevent bad transients
    bool allzero  = true;
    for (size_t i = 0; i < JOINT_FILTER_NB; ++i)
        allzero &= filter->x[i] == 0;
    if (allzero) {
        // If all filter values are zero, initialize the signal array
        // with the current encoder value
        for (size_t i = 0; i < JOINT_FILTER_NB; ++i)
            filter->x[i] = joint->position;
    }

    // Shift and update signal arrays
    for (size_t i = JOINT_FILTER_NB - 1; i > 0; --i)
        filter->x[i] = filter->x[i - 1];
    filter->x[0] = joint->position;
    for (size_t i = JOINT_FILTER_NA - 1; i > 0; --i)
        filter->y[i] = filter->y[i - 1];

    // Compute filter value
    filter->y[0] = 0;
    for (size_t i = 0; i < JOINT_FILTER_NB; ++i)
        filter->y[0] += filter->x[i] * jointFilterB_[i];
    for (size_t i = 1; i < JOINT_FILTER_NA; ++i)
        filter->y[0] -= filter->y[i] * jointFilterA_[i];
    joint->velocity = filter->y[0];
}


double CassiePlugin::setMotor(gazebo::physics::JointPtr outjoint, double u,
                              const bool sto, const double ratio,
                              const double tmax, const double wmax)
{
    // Get rotor velocity
    double w = outjoint->GetVelocity(0) * ratio;

    // Calculate torque limit based on motor speed
    double tlim = 2 * tmax * (1 - fabs(w) / wmax);
    tlim = fmax(fmin(tlim, tmax), 0);

    // Apply STO
    if (sto)
        u = 0;

    // Compute output-side torque
    double tau = ratio * copysign(fmin(fabs(u / ratio), tlim), u);
    outjoint->SetForce(0, tau);

    // Return limited output-side torque
    return tau;
}


void CassiePlugin::initElmoOut(elmo_out_t *elmoOut, double torqueLimit,
                               const double gearRatio)
{
    elmoOut->statusWord       = 0x0637;
    elmoOut->dcLinkVoltage    = 48;
    elmoOut->driveTemperature = 30;
    elmoOut->torqueLimit      = torqueLimit;
    elmoOut->gearRatio        = gearRatio;
}

void CassiePlugin::initCassieLegOut(cassie_leg_out_t *legOut)
{
    legOut->medullaCounter = 1;
    legOut->medullaCpuLoad = 94;
    initElmoOut(&legOut->hipRollDrive,  140.63, 25);
    initElmoOut(&legOut->hipYawDrive,   140.63, 25);
    initElmoOut(&legOut->hipPitchDrive, 216.16, 16);
    initElmoOut(&legOut->kneeDrive,     216.16, 16);
    initElmoOut(&legOut->footDrive,      45.14, 50);
}


void CassiePlugin::initCassieOut(cassie_out_t *cassieOut)
{
    // Zero-initialize the struct
    *cassieOut = {};

    // Calibrated
    cassieOut->isCalibrated = true;

    // Pelvis
    cassieOut->pelvis.medullaCounter = 1;
    cassieOut->pelvis.medullaCpuLoad = 159;
    cassieOut->pelvis.vtmTemperature = 40;

    // Target PC
    cassieOut->pelvis.targetPc.etherCatStatus[1] = 8;
    cassieOut->pelvis.targetPc.etherCatStatus[4] = 1;
    cassieOut->pelvis.targetPc.taskExecutionTime = 2e-4;
    cassieOut->pelvis.targetPc.cpuTemperature    = 60;

    // Battery
    cassieOut->pelvis.battery.dataGood = true;
    cassieOut->pelvis.battery.stateOfCharge = 1;
    for (size_t i = 0; i < 4; ++i)
        cassieOut->pelvis.battery.temperature[i] = 30;
    for (size_t i = 0; i < 12; ++i)
        cassieOut->pelvis.battery.voltage[i] = 4.2;

    // Radio
    cassieOut->pelvis.radio.radioReceiverSignalGood = true;
    cassieOut->pelvis.radio.receiverMedullaSignalGood = true;
    cassieOut->pelvis.radio.channel[8] = 1;

    // VectorNav
    cassieOut->pelvis.vectorNav.dataGood = true;
    cassieOut->pelvis.vectorNav.pressure = 101.325;
    cassieOut->pelvis.vectorNav.temperature = 25;

    // Legs
    initCassieLegOut(&cassieOut->leftLeg);
    initCassieLegOut(&cassieOut->rightLeg);
}


void CassiePlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    // Store a pointer to the world
    this->worldPtr_ = model->GetWorld();

    // Store pointers to each model joint corresponding to a motor output
    motor_ = {
        model->GetJoint("left-roll-op"),
        model->GetJoint("left-yaw-op"),
        model->GetJoint("left-pitch-op"),
        model->GetJoint("left-knee-op"),
        model->GetJoint("left-foot-op"),
        model->GetJoint("right-roll-op"),
        model->GetJoint("right-yaw-op"),
        model->GetJoint("right-pitch-op"),
        model->GetJoint("right-knee-op"),
        model->GetJoint("right-foot-op")
    };

    // Store pointers to each model joint corresponding to a measured joint
    joint_ = {
        model->GetJoint("left-knee-shin-joint"),
        model->GetJoint("left-shin-tarsus-joint"),
        model->GetJoint("left-foot-op"),
        model->GetJoint("right-knee-shin-joint"),
        model->GetJoint("right-shin-tarsus-joint"),
        model->GetJoint("right-foot-op")
    };

    // Store a pointer to the pelvis link
    pelvisPtr_ = model->GetLink("pelvis");

    // Update Rate
    this->updateRate_ = 2000;
    this->updatePeriod_ = 1.0 / this->updateRate_;
    lastUpdateTime_ = this->worldPtr_->SimTime();

    // Open UDP socket
    sock_ = udp_init_host("0.0.0.0", "25000");

    // Listen to the update event which is broadcast every simulation iteration
    this->updateConnectionPtr_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&CassiePlugin::onUpdate, this));

    // Set the imu subscriber
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init("default");
    std::string topicName = "/gazebo/default/cassie/pelvis/imu/imu";
    this->imuSubscriber = node->Subscribe(topicName, &CassiePlugin::onSensorUpdate, this);
}

void CassiePlugin::onUpdate()
{
    // Get current time and calculate last update time
    auto currentTime = worldPtr_->SimTime();
    auto secondsSinceLastUpdate = (currentTime - lastUpdateTime_).Double();

    // Run at the specified period
    if (secondsSinceLastUpdate < updatePeriod_*0.95)
        return;

    // Zero the input if no packets have been received in a while
    if ((currentTime - lastPacketTime_).Double() > 0.01)
        cassieUserIn_ = {};

    // Get newest packet, or return -1 if no new packets are available
    addrLen_ = sizeof srcAddr_;
    ssize_t nbytes = get_newest_packet(sock_, recvBuf_, RECVLEN,
                                       (struct sockaddr *) &srcAddr_,
                                       &addrLen_);

    // If a new packet was received, process and unpack it
    if (RECVLEN == nbytes) {
        // Process incoming header and write outgoing header
        process_packet_header(&headerInfo_, headerInPtr_, headerOutPtr_);

        // Unpack received data into cassie user input struct
        unpack_cassie_user_in_t(dataInPtr_, &cassieUserIn_);

        // Start running Cassie core after the first valid packet is received
        if (!runSim_) {
            runSim_ = true;
            firstPacketTime_ = currentTime;
        }

        // Record time
        lastPacketTime_ = currentTime;
    }

    // Detatch pelvis after receiving data
    // Slowly lower and detach robot for easy initialization
    const double LOWER_TIME = 5.0;
    const double HOLD_TIME  = 7.4;
    const double DETACH_TIME = 9.0;
    const double DAMPER_TIME = 12.0;
    if (static_joint_attached) {
        if (((currentTime - firstPacketTime_).Double() > LOWER_TIME) && ((currentTime - firstPacketTime_).Double() < HOLD_TIME)) {
            // Lower pelvis x seconds after receiving data
            lowerPelvis();
        }
        else if ((currentTime - firstPacketTime_).Double() > HOLD_TIME && ((currentTime - firstPacketTime_).Double() < DETACH_TIME)) {
            // DO NOTHING
        }
        else if ((currentTime - firstPacketTime_).Double() > DETACH_TIME) {
            // Detatch pelvis x seconds after receiving data
            detachPelvis();
        }
    }
    if ((currentTime - firstPacketTime_).Double() > DETACH_TIME && ((currentTime - firstPacketTime_).Double() < DAMPER_TIME)) {
        gazebo::physics::LinkPtr pelvis = this->worldPtr_->ModelByName("cassie")->GetLink("pelvis");
        ignition::math::Vector3d vpelvis = pelvis->WorldLinearVel();
        ignition::math::Vector3d wpelvis = pelvis->WorldAngularVel();
        ignition::math::Vector3d fpelvis = vpelvis;
        ignition::math::Vector3d mpelvis = wpelvis;
        fpelvis.X() *= -300;
        fpelvis.Y() *= -10;
        fpelvis.Z() *= -100;
        mpelvis.X() *= -25;
        mpelvis.Y() *= -25;
        mpelvis.Z() *= -40;

        pelvis->AddForce(fpelvis);
        pelvis->AddTorque(mpelvis);
    } else {
        gazebo::physics::LinkPtr pelvis = this->worldPtr_->ModelByName("cassie")->GetLink("pelvis");
        ignition::math::Vector3d vpelvis = pelvis->WorldLinearVel();
        ignition::math::Vector3d wpelvis = pelvis->WorldAngularVel();
        ignition::math::Vector3d fpelvis = vpelvis;
        ignition::math::Vector3d mpelvis = wpelvis;
        fpelvis.X() *= -0;
        fpelvis.Y() *= -0;
        fpelvis.Z() *= -0;
        mpelvis.X() *= -0;
        mpelvis.Y() *= -0;
        mpelvis.Z() *= -0;


        pelvis->AddForce(fpelvis);
        pelvis->AddTorque(mpelvis);
    }

    if (runSim_) {
        // Run simulator and pack output struct into outgoing packet
        cassie_in_t cassieIn;
        cassie_out_t output = cassieOut_;
        cassie_core_sim_step(corePtr_, &cassieUserIn_, &output, &cassieIn);
        applyTorques(&cassieIn);
        updateCassieOut();
        lastUpdateTime_ += gazebo::common::Time(updatePeriod_);

        pack_cassie_out_t(&output, dataOutPtr_);

        // Send response
        send_packet(sock_, sendBuf_, SENDLEN,
                    (struct sockaddr *) &srcAddr_, addrLen_);
    }
}

void CassiePlugin::onSensorUpdate(ConstIMUPtr &_msg) {
    this->gyroscope     = _msg->angular_velocity();
    this->accelerometer = _msg->linear_acceleration();
    this->orientation   = _msg->orientation();
}

void CassiePlugin::updateCassieOut()
{
    // Motor encoders
    for (size_t i = 0; i < 10; ++i) {
        setMotorEncoder(driveOut_[i], motor_[i]->Position(0) + kMotorOffset_[i],
                        &motorFilter_[i], kMotorBits_[i]);

        driveOut_[i]->position = motor_[i]->Position(0) + kMotorOffset_[i];
        driveOut_[i]->velocity = motor_[i]->GetVelocity(0);
    }

    // Joint encoders
    for (size_t i = 0; i < 6; ++i) {
        setJointEncoder(jointOut_[i], joint_[i]->Position(0) + kJointOffset_[i],
                        &jointFilter_[i], kJointBits_[i]);

        jointOut_[i]->position = joint_[i]->Position(0) + kJointOffset_[i];
        jointOut_[i]->velocity = joint_[i]->GetVelocity(0);
    }

    // IMU
    /*
    auto pose = pelvisPtr_->WorldPose();

    auto worldAccel = pelvisPtr_->WorldLinearAccel() - worldPtr_->Gravity();
    auto worldGyro = pelvisPtr_->WorldAngularVel();
    auto worldMag = ignition::math::Vector3d(0, 1, 0);

    auto rot = pose.Rot().Inverse();
    auto accel = rot.RotateVector(worldAccel);
    auto gyro = rot.RotateVector(worldGyro);
    auto mag = rot.RotateVector(worldMag);

    // Set computed orientation
    cassieOut_.pelvis.vectorNav.orientation[0] = pose.Rot().W();
    cassieOut_.pelvis.vectorNav.orientation[1] = pose.Rot().X();
    cassieOut_.pelvis.vectorNav.orientation[2] = pose.Rot().Y();
    cassieOut_.pelvis.vectorNav.orientation[3] = pose.Rot().Z();

    // Set accelerometer/gyro/magnetometer measurement
    for (size_t i = 0; i < 3; ++i) {
        if (accel[i] > 30)
            accel[i] = 30;
        if (accel[i] < -30)
            accel[i] = -30;

        cassieOut_.pelvis.vectorNav.linearAcceleration[i] = accel[i];
        cassieOut_.pelvis.vectorNav.angularVelocity[i] = gyro[i];
        cassieOut_.pelvis.vectorNav.magneticField[i] = mag[i];
    }
    */

    auto pose = pelvisPtr_->WorldPose();
    auto rot = pose.Rot().Inverse();
    auto worldMag = ignition::math::Vector3d(0, 1, 0);
    auto mag = rot.RotateVector(worldMag);

    // Set computed orientation
    cassieOut_.pelvis.vectorNav.orientation[0] = pose.Rot().W();
    cassieOut_.pelvis.vectorNav.orientation[1] = pose.Rot().X();
    cassieOut_.pelvis.vectorNav.orientation[2] = pose.Rot().Y();
    cassieOut_.pelvis.vectorNav.orientation[3] = pose.Rot().Z();
    //    cassieOut_.pelvis.vectorNav.orientation[0] = this->orientation.w();
    //    cassieOut_.pelvis.vectorNav.orientation[1] = this->orientation.x();
    //    cassieOut_.pelvis.vectorNav.orientation[2] = this->orientation.y();
    //    cassieOut_.pelvis.vectorNav.orientation[3] = this->orientation.z();

    cassieOut_.pelvis.vectorNav.angularVelocity[0] = this->gyroscope.x();
    cassieOut_.pelvis.vectorNav.angularVelocity[1] = this->gyroscope.y();
    cassieOut_.pelvis.vectorNav.angularVelocity[2] = this->gyroscope.z();

    cassieOut_.pelvis.vectorNav.linearAcceleration[0] = this->accelerometer.x();
    cassieOut_.pelvis.vectorNav.linearAcceleration[1] = this->accelerometer.y();
    cassieOut_.pelvis.vectorNav.linearAcceleration[2] = this->accelerometer.z();

    //cassieOut_.pelvis.vectorNav.magneticField[0] = mag[0];
    //cassieOut_.pelvis.vectorNav.magneticField[1] = mag[1];
    //cassieOut_.pelvis.vectorNav.magneticField[2] = mag[2];

    gazebo::physics::LinkPtr pelvis = this->worldPtr_->ModelByName("cassie")->GetLink("pelvis");
    ignition::math::Vector3d vpelvis = pelvis->WorldLinearVel();
    cassieOut_.pelvis.vectorNav.magneticField[0] = vpelvis.X();
    cassieOut_.pelvis.vectorNav.magneticField[1] = vpelvis.Y();
    cassieOut_.pelvis.vectorNav.magneticField[2] = vpelvis.Z();
}

void CassiePlugin::applyTorques(const cassie_in_t *cassieIn)
{
    // STO
    bool sto = this->cassieOut_.pelvis.radio.channel[8] < 1;

    // Get torque commands
    const double torque[] = {
        cassieIn->leftLeg.hipRollDrive.torque,
        cassieIn->leftLeg.hipYawDrive.torque,
        cassieIn->leftLeg.hipPitchDrive.torque,
        cassieIn->leftLeg.kneeDrive.torque,
        cassieIn->leftLeg.footDrive.torque,
        cassieIn->rightLeg.hipRollDrive.torque,
        cassieIn->rightLeg.hipYawDrive.torque,
        cassieIn->rightLeg.hipPitchDrive.torque,
        cassieIn->rightLeg.kneeDrive.torque,
        cassieIn->rightLeg.footDrive.torque
    };

    // Set and output limited torque commands
    for (size_t i = 0; i < 10; ++i) {
        driveOut_[i]->torque = setMotor(motor_[i], torque[i], sto,
                                        kGearRatio_[i], kMaxTorque_[i],
                                        kMaxSpeed_[i]);
    }
}

void CassiePlugin::detachPelvis()
{
    // Set large limits to effectively detach
    gazebo::physics::JointPtr x_joint = this->worldPtr_->ModelByName("cassie")->GetJoint("BasePosX");
    x_joint->SetLowerLimit(0,-1000);
    x_joint->SetUpperLimit(0,1000);

    gazebo::physics::JointPtr y_joint = this->worldPtr_->ModelByName("cassie")->GetJoint("BasePosY");
    y_joint->SetLowerLimit(0,-1000);
    y_joint->SetUpperLimit(0,1000);

    gazebo::physics::JointPtr z_joint = this->worldPtr_->ModelByName("cassie")->GetJoint("BasePosZ");
    z_joint->SetLowerLimit(0,-1000);
    z_joint->SetUpperLimit(0,1000);

    gazebo::physics::JointPtr roll_joint = this->worldPtr_->ModelByName("cassie")->GetJoint("BaseRotX");
    roll_joint->SetLowerLimit(0,-1000);
    roll_joint->SetUpperLimit(0,1000);

    gazebo::physics::JointPtr pitch_joint = this->worldPtr_->ModelByName("cassie")->GetJoint("BaseRotY");
    pitch_joint->SetLowerLimit(0,-1000);
    pitch_joint->SetUpperLimit(0,1000);

    gazebo::physics::JointPtr yaw_joint = this->worldPtr_->ModelByName("cassie")->GetJoint("BaseRotZ");
    yaw_joint->SetLowerLimit(0,-1000);
    yaw_joint->SetUpperLimit(0,1000);

    static_joint_attached = false;
}

void CassiePlugin::lowerPelvis()
{
    // Lower by small amount each timestep
    gazebo::physics::JointPtr joint = this->worldPtr_->ModelByName("cassie")->GetJoint("BasePosZ");
    joint->SetLowerLimit(0, joint->LowerLimit()-0.00005);
}

// Register plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(CassiePlugin)
