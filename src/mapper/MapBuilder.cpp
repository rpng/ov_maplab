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


MapBuilder::MapBuilder(ros::NodeHandle &nh, VioManager* app) : _app(app) {


    //==========================================================================
    // IMU PROPERTIES
    //==========================================================================

    // Load the IMU noises
    vi_map::ImuSigmas imu_sigmas;
    nh.param<double>("gyroscope_noise_density", imu_sigmas.gyro_noise_density, 1.6968e-04);
    nh.param<double>("accelerometer_noise_density", imu_sigmas.acc_noise_density, 2.0000e-3);
    nh.param<double>("gyroscope_random_walk", imu_sigmas.gyro_bias_random_walk_noise_density , 1.9393e-05);
    nh.param<double>("accelerometer_random_walk", imu_sigmas.acc_bias_random_walk_noise_density, 3.0000e-03);

    //==========================================================================
    // CAMERA PROPERTIES
    //==========================================================================

    // Camera calibration values
    aslam::TransformationVector T_ItoCi;
    std::vector<std::shared_ptr<aslam::Camera>> cameras;

    // Loop through through, and load each of the cameras
    for(int i=0; i<_app->get_state()->options().num_cameras; i++) {

        // Camera image size
        std::vector<int> matrix_wh;
        std::vector<int> matrix_wd_default = {752,480};
        nh.param<std::vector<int>>("cam"+std::to_string(i)+"_wh", matrix_wh, matrix_wd_default);

        // Camera intrinsic properties
        Eigen::Matrix<double,4,1> cam_proj;
        Eigen::Matrix<double,4,1> cam_dist;
        std::vector<double> matrix_k, matrix_d;
        std::vector<double> matrix_k_default = {458.654,457.296,367.215,248.375};
        std::vector<double> matrix_d_default = {-0.28340811,0.07395907,0.00019359,1.76187114e-05};
        nh.param<std::vector<double>>("cam"+std::to_string(i)+"_k", matrix_k, matrix_k_default);
        nh.param<std::vector<double>>("cam"+std::to_string(i)+"_d", matrix_d, matrix_d_default);
        cam_proj << matrix_k.at(0),matrix_k.at(1),matrix_k.at(2),matrix_k.at(3);
        cam_dist << matrix_d.at(0),matrix_d.at(1),matrix_d.at(2),matrix_d.at(3);

        // If our distortions are fisheye or not!
        bool is_fisheye;
        nh.param<bool>("cam"+std::to_string(i)+"_is_fisheye", is_fisheye, false);
        aslam::Distortion* distptr;
        if(is_fisheye) {
            distptr = new aslam::EquidistantDistortion(cam_dist);
        } else{
            distptr = new aslam::RadTanDistortion(cam_dist);
        }

        // Finally create the camera object!
        aslam::Distortion::UniquePtr uniqdistptr(distptr);
        std::shared_ptr<aslam::Camera> camera = std::make_shared<aslam::PinholeCamera>(cam_proj,matrix_wh.at(0),matrix_wh.at(1),uniqdistptr);
        camera->setId(aslam::CameraId::Random());
        cameras.push_back(camera);

        // Our camera extrinsics transform
        // Read in from ROS, and save into our eigen mat
        Eigen::Matrix4d T_CtoI;
        std::vector<double> matrix_TCtoI;
        std::vector<double> matrix_TtoI_default = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
        nh.param<std::vector<double>>("T_C"+std::to_string(i)+"toI", matrix_TCtoI, matrix_TtoI_default);
        T_CtoI << matrix_TCtoI.at(0),matrix_TCtoI.at(1),matrix_TCtoI.at(2),matrix_TCtoI.at(3),
                matrix_TCtoI.at(4),matrix_TCtoI.at(5),matrix_TCtoI.at(6),matrix_TCtoI.at(7),
                matrix_TCtoI.at(8),matrix_TCtoI.at(9),matrix_TCtoI.at(10),matrix_TCtoI.at(11),
                matrix_TCtoI.at(12),matrix_TCtoI.at(13),matrix_TCtoI.at(14),matrix_TCtoI.at(15);

        // Append this transform
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

    if(!_app->intialized())
        return;

    //==========================================================================
    // ADD THE CURRENT STATE TO THIS TIME
    //==========================================================================

    // Return if the current state has not been updated to this time
    if(_app->get_state()->timestamp() != timestamp) {
        ROS_WARN("[MAP]: the vio state timestamp is not the passed image time (are you initialized??, dt = %.4f)",timestamp-_app->get_state()->timestamp());
        return;
    }

    // Our transformation
    Eigen::Matrix4d T_ItoG = Eigen::Matrix4d::Identity();
    T_ItoG.block(0,0,3,3) = _app->get_state()->imu()->q()->Rot().transpose();
    T_ItoG.block(0,3,3,1) = _app->get_state()->imu()->pos();

    // Create our VIO state object
    vio::VioUpdate update;
    update.timestamp_ns = (int64_t)(1e9*timestamp);
    update.vinode = vio::ViNodeState(aslam::Transformation::constructAndRenormalizeRotation(T_ItoG),
            _app->get_state()->imu()->vel(), _app->get_state()->imu()->bias_a(), _app->get_state()->imu()->bias_g());

    // Create the camera frame system
    aslam::VisualNFrame::Ptr nframe(new aslam::VisualNFrame(camera_rig));
    nframe->setId(aslam::NFramesId::Random());
    for(size_t i=0; i<images.size(); i++) {
        std::shared_ptr<aslam::VisualFrame> frame = std::make_shared<aslam::VisualFrame>();
        frame->setCameraGeometry(camera_rig->getCameraShared(i));
        frame->setId(aslam::FrameId::Random());
        frame->setTimestampNanoseconds((int64_t)(1e9*timestamp));
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
        last_prop_time_offset = _app->get_state()->calib_dt_CAMtoIMU()->value()(0);
        return;
    }


    //==========================================================================
    // GET IMU BETWEEN LAST AND CURRENT AND LINK THE NODES
    //==========================================================================

    // First lets construct an IMU vector of measurements we need
    vector<Propagator::IMUDATA> prop_data;

    // Get what our IMU-camera offset should be (t_imu = t_cam + calib_dt)
    double t_off_new = _app->get_state()->calib_dt_CAMtoIMU()->value()(0);

    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Ensure we have some measurements in the first place!
    if(imu_data.empty()) {
        std::cerr << "MapBuilder(): There are no IMU measurements!!!!!" << std::endl;
        std::cerr << "MapBuilder(): IMU-CAMERA are likely messed up, check time offset value!!!" << std::endl;
        std::cerr << __FILE__ << " on line " << __LINE__ << std::endl;
        return;
    }

    // Loop through and find all the needed measurements to propagate with
    // Note we split measurements based on the given state time, and the update timestamp
    for(size_t i=0; i<imu_data.size()-1; i++) {

        // START OF THE INTEGRATION PERIOD
        // If the next timestamp is greater then our current state time
        // And the current is not greater then it yet...
        // Then we should "split" our current IMU measurement
        if(imu_data.at(i+1).timestamp > last_timestamp+last_prop_time_offset && imu_data.at(i).timestamp < last_timestamp+last_prop_time_offset) {
            Propagator::IMUDATA data = interpolate_data(imu_data.at(i),imu_data.at(i+1), last_timestamp+last_prop_time_offset);
            prop_data.push_back(data);
            continue;
        }

        // MIDDLE OF INTEGRATION PERIOD
        // If our imu measurement is right in the middle of our propagation period
        // Then we should just append the whole measurement time to our propagation vector
        if(imu_data.at(i).timestamp >= last_timestamp+last_prop_time_offset && imu_data.at(i+1).timestamp <= timestamp+t_off_new) {
            prop_data.push_back(imu_data.at(i));
            continue;
        }

        // END OF THE INTEGRATION PERIOD
        // If the current timestamp is greater then our update time
        // We should just "split" the NEXT IMU measurement to the update time,
        // NOTE: we add the current time, and then the time at the end of the interval (so we can get a dt)
        // NOTE: we also break out of this loop, as this is the last IMU measurement we need!
        if(imu_data.at(i+1).timestamp > timestamp+t_off_new) {
            prop_data.push_back(imu_data.at(i));
            // If the added IMU message doesn't end exactly at the camera time
            // Then we need to add another one that is right at the ending time
            if(prop_data.at(prop_data.size()-1).timestamp != timestamp+t_off_new) {
                Propagator::IMUDATA data = interpolate_data(imu_data.at(i), imu_data.at(i+1), timestamp+t_off_new);
                prop_data.push_back(data);
            }
            break;
        }

    }

    // Check that we have at least one measurement to propagate with
    if(prop_data.empty()) {
        std::cerr << "MapBuilder(): There are not enough measurements to propagate with " << (int)prop_data.size() << " of 2" << std::endl;
        std::cerr << "MapBuilder(): IMU-CAMERA are likely messed up, check time offset value!!!" << std::endl;
        std::cerr << __FILE__ << " on line " << __LINE__ << std::endl;
        return;
    }


    // If we did not reach the whole integration period (i.e., the last inertial measurement we have is smaller then the time we want to reach)
    // Then we should just "stretch" the last measurement to be the whole period
    if(imu_data.at(imu_data.size()-1).timestamp <= timestamp+t_off_new) {
        Propagator::IMUDATA data = interpolate_data(imu_data.at(imu_data.size()-2),imu_data.at(imu_data.size()-1),timestamp+t_off_new);
        prop_data.push_back(data);
    }


    // Loop through and ensure we do not have an zero dt values
    // This would cause the noise covariance to be Infinity
    for (size_t i=0; i < prop_data.size()-1; i++){
        if (std::abs(prop_data.at(i+1).timestamp-prop_data.at(i).timestamp) < 1e-8){
            std::cerr << "MapBuilder(): Zero DT between " << i << " and " << i+1 << " measurements (dt = " << (prop_data.at(i+1).timestamp-prop_data.at(i).timestamp) << ")" << std::endl;
            prop_data.erase(prop_data.begin()+i);
            i--;
        }
    }

    // Check that we have at least one measurement to propagate with
    if(prop_data.size() < 2) {
        std::cerr << "MapBuilder(): There are not enough measurements to propagate with " << (int)prop_data.size() << " of 2" << std::endl;
        std::cerr << "MapBuilder(): IMU-CAMERA are likely messed up, check time offset value!!!" << std::endl;
        std::cerr << __FILE__ << " on line " << __LINE__ << std::endl;
        return;
    }


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


