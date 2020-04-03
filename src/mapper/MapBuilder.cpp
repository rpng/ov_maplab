/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "MapBuilder.h"

using namespace ov_msckf;


MapBuilder::MapBuilder(ros::NodeHandle &nh, VioManager* app, const VioManagerOptions &params) : _app(app), _params(params) {


    //==========================================================================
    // IMU PROPERTIES
    //==========================================================================

    // Load the IMU noises
    vi_map::ImuSigmas imu_sigmas;
    imu_sigmas.gyro_noise_density = _params.imu_noises.sigma_w;
    imu_sigmas.acc_noise_density = _params.imu_noises.sigma_a;
    imu_sigmas.gyro_bias_random_walk_noise_density = _params.imu_noises.sigma_wb;
    imu_sigmas.acc_bias_random_walk_noise_density = _params.imu_noises.sigma_ab;

    //==========================================================================
    // CAMERA PROPERTIES
    //==========================================================================

    // Camera calibration values
    aslam::TransformationVector T_ItoCi;
    std::vector<std::shared_ptr<aslam::Camera>> cameras;

    // Loop through through, and load each of the cameras
    for(int i=0; i<_params.state_options.num_cameras; i++) {

        // Camera intrinsic properties
        Eigen::Matrix<double,4,1> cam_proj = _params.camera_intrinsics.at(i).block(0,0,4,1);
        Eigen::Matrix<double,4,1> cam_dist = _params.camera_intrinsics.at(i).block(4,0,4,1);

        // If our distortions are fisheye or not!
        aslam::Distortion* distptr;
        if(_params.camera_fisheye.at(i)) {
            distptr = new aslam::EquidistantDistortion(cam_dist);
        } else{
            distptr = new aslam::RadTanDistortion(cam_dist);
        }

        // Finally create the camera object!
        aslam::Distortion::UniquePtr uniqdistptr(distptr);
        std::shared_ptr<aslam::Camera> camera = std::make_shared<aslam::PinholeCamera>(cam_proj,_params.camera_wh.at(i).first,_params.camera_wh.at(i).second,uniqdistptr);
        camera->setId(aslam::CameraId::Random());
        cameras.push_back(camera);

        // Our camera extrinsics transform
        Eigen::Matrix4d T_CtoI = Eigen::Matrix4d::Identity();
        T_CtoI.block(0,0,3,3) = quat_2_Rot(_params.camera_extrinsics.at(i).block(0,0,4,1)).transpose();
        T_CtoI.block(0,3,3,1) = -T_CtoI.block(0,0,3,3)*_params.camera_extrinsics.at(i).block(4,0,3,1);
        Eigen::Matrix4d T_ItoC = T_CtoI.inverse();
        T_ItoCi.push_back(aslam::Transformation::constructAndRenormalizeRotation(T_ItoC));

        // Debug print
        camera->printParameters(cout,"CAMERA "+std::to_string(i));
        cout << "T_ItoC" << std::endl << T_ItoCi.at(i) << std::endl;

    }

    //==========================================================================
    // CREATE OUR MAP OBJECTS!
    //==========================================================================

    // Create our IMU object
    constexpr char kImuHardwareId[] = "imu0";
    vi_map::SensorId imu_sensor_id;
    common::generateId(&imu_sensor_id);
    vi_map::Imu::UniquePtr imu_sensor = aligned_unique<vi_map::Imu>(imu_sensor_id, static_cast<std::string>(kImuHardwareId));
    imu_sensor->setImuSigmas(imu_sigmas);

    // Create our camera object
    std::string cam_label = "ncameras";
    camera_rig = std::make_shared<aslam::NCamera>(aslam::NCameraId::Random(), T_ItoCi, cameras, cam_label);

    // Create the master map object
    nh.param<std::string>("map_save_folder", save_folder, "/home/patrick/datasets/maplab/test/");
    map = new vi_map::VIMap(save_folder);

    // Create our map builder
    builder = new online_map_builders::StreamMapBuilder(camera_rig, std::move(imu_sensor), map);


}





void MapBuilder::feed_measurement_camera(double timestamp, std::vector<cv::Mat> &images, std::vector<size_t> &cam_ids) {


    //==========================================================================
    // Check if the system is valid
    //==========================================================================

    if(!_app->initialized())
        return;

    //==========================================================================
    // ADD THE CURRENT STATE TO THIS TIME
    //==========================================================================

    // Return if the current state has not been updated to this time
    if(_app->get_state()->_timestamp != timestamp) {
        ROS_WARN("[MAP]: the vio state timestamp is not the passed image time (are you initialized??, dt = %.4f)",timestamp-_app->get_state()->_timestamp);
        return;
    }

    // Get the current timestamp in the imu clock frame of reference
    double t_ItoC = _app->get_state()->_calib_dt_CAMtoIMU->value()(0);
    double timestamp_inI = _app->get_state()->_timestamp + t_ItoC;

    // Our transformation
    Eigen::Matrix4d T_ItoG = Eigen::Matrix4d::Identity();
    T_ItoG.block(0,0,3,3) = _app->get_state()->_imu->q()->Rot().transpose();
    T_ItoG.block(0,3,3,1) = _app->get_state()->_imu->pos();

    // Create our VIO state object
    vio::VioUpdate update;
    update.timestamp_ns = (int64_t)(1e9*timestamp_inI);
    update.vinode = vio::ViNodeState(aslam::Transformation::constructAndRenormalizeRotation(T_ItoG),
            _app->get_state()->_imu->vel(), _app->get_state()->_imu->bias_a(), _app->get_state()->_imu->bias_g());

    // Create the camera frame system
    aslam::VisualNFrame::Ptr nframe(new aslam::VisualNFrame(camera_rig));
    nframe->setId(aslam::NFramesId::Random());
    for(size_t i=0; i<images.size(); i++) {
        std::shared_ptr<aslam::VisualFrame> frame = std::make_shared<aslam::VisualFrame>();
        frame->setCameraGeometry(camera_rig->getCameraShared(i));
        frame->setId(aslam::FrameId::Random());
        frame->setTimestampNanoseconds((int64_t)(1e9*timestamp_inI));
        frame->clearKeypointChannels();
        frame->setRawImage(images.at(i).clone());
        nframe->setFrame(cam_ids.at(i),frame);
    }
    assert(nframe->areAllFramesSet());
    assert(nframe->hasRawImagesInAllFrames());

    // Save the current frame to our vio update object
    vio::SynchronizedNFrameImu sync_nframe;
    sync_nframe.nframe = nframe;
    sync_nframe.motion_wrt_last_nframe = vio::MotionType::kGeneralMotion;

    // return if this is the first ever pose
    if(last_timestamp == -1) {
        // Send it to our map builder and store the raw images for later use
        update.keyframe_and_imudata = std::make_shared<const vio::SynchronizedNFrameImu>(sync_nframe);
        builder->apply(update);
        for (size_t frame_idx = 0u; frame_idx < nframe->getNumFrames(); frame_idx++) {
            map->storeRawImage(nframe->getFrame(frame_idx).getRawImage(), frame_idx, map->getVertexPtr(builder->getLastVertexId()));
        }
        // Save this timestamp
        last_timestamp = timestamp;
        last_prop_time_offset = _app->get_state()->_calib_dt_CAMtoIMU->value()(0);
        return;
    }


    //==========================================================================
    // GET IMU BETWEEN LAST AND CURRENT AND LINK THE NODES
    //==========================================================================

    // Get what our IMU-camera offset should be (t_imu = t_cam + calib_dt)
    double t_off_new = _app->get_state()->_calib_dt_CAMtoIMU->value()(0);

    // First lets construct an IMU vector of measurements we need
    double time0 = last_timestamp+last_prop_time_offset;
    double time1 = timestamp+t_off_new;
    vector<Propagator::IMUDATA> prop_data = Propagator::select_imu_readings(imu_data, time0, time1);

    // If empty, then we should skip this camera measurement
    if(prop_data.size()<2)
        return;

    // Convert into the maplab format (order is acc, gyro??)
    sync_nframe.imu_timestamps.resize(1,prop_data.size());
    sync_nframe.imu_measurements.resize(6,prop_data.size());
    for (size_t i=0; i < prop_data.size(); i++){
        sync_nframe.imu_timestamps(i) = (int64_t)(1e9*prop_data.at(i).timestamp);
        sync_nframe.imu_measurements.block(0,i,3,1) = prop_data.at(i).am;
        sync_nframe.imu_measurements.block(3,i,3,1) = prop_data.at(i).wm;
    }

    // Send it to our map builder and store the raw images for later use
    update.keyframe_and_imudata = std::make_shared<const vio::SynchronizedNFrameImu>(sync_nframe);
    builder->apply(update);
    for (size_t frame_idx = 0u; frame_idx < nframe->getNumFrames(); frame_idx++) {
        map->storeRawImage(nframe->getFrame(frame_idx).getRawImage(), frame_idx, map->getVertexPtr(builder->getLastVertexId()));
    }

    // Record the time we last propagated to
    last_timestamp = timestamp;
    last_prop_time_offset = t_off_new;

}



void MapBuilder::save_to_disk() {

    // Go through and extract things
    ROS_INFO("[MAP]: going to extract features from the raw images...");
    feature_tracking::VOFeatureTrackingPipeline trackpipe;
    trackpipe.runTrackingAndTriangulationForAllMissions(map);

    // Check for a good map
    builder->checkConsistency();

    // Save to file
    ROS_INFO("[MAP]: saving to file!!!");
    backend::SaveConfig save_config;
    save_config.overwrite_existing_files = true;
    vi_map::serialization::saveMapToFolder(save_folder, save_config, map);
    ROS_INFO("[MAP]: saved to %s",save_folder.c_str());

}


