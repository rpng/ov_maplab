/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2021 Patrick Geneva
 * Copyright (C) 2021 Guoquan Huang
 * Copyright (C) 2021 OpenVINS Contributors
 * Copyright (C) 2019 Kevin Eckenhoff
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

MapBuilder::MapBuilder(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<VioManager> app, const VioManagerOptions &params)
    : _app(app), _params(params) {

  // Load the IMU noises
  vi_map::ImuSigmas imu_sigmas;
  imu_sigmas.gyro_noise_density = _params.imu_noises.sigma_w;
  imu_sigmas.acc_noise_density = _params.imu_noises.sigma_a;
  imu_sigmas.gyro_bias_random_walk_noise_density = _params.imu_noises.sigma_wb;
  imu_sigmas.acc_bias_random_walk_noise_density = _params.imu_noises.sigma_ab;

  // Camera calibration values
  aslam::TransformationVector T_ItoCi, T_CitoI;
  std::vector<std::shared_ptr<aslam::Camera>> cameras;

  // Loop through, and load each of the cameras
  std::stringstream ss;
  for (int i = 0; i < _params.state_options.num_cameras; i++) {

    // Camera intrinsic properties
    auto camptr = _params.camera_intrinsics.at(i);
    Eigen::Matrix<double, 4, 1> cam_proj = camptr->get_value().block(0, 0, 4, 1);
    Eigen::Matrix<double, 4, 1> cam_dist = camptr->get_value().block(4, 0, 4, 1);

    // If our distortions are fisheye or not!
    aslam::Distortion *distptr;
    bool is_fisheye = (std::dynamic_pointer_cast<ov_core::CamEqui>(camptr) != nullptr);
    if (is_fisheye) {
      distptr = new aslam::EquidistantDistortion(cam_dist);
    } else {
      distptr = new aslam::RadTanDistortion(cam_dist);
    }

    // Finally create the camera object!
    aslam::Distortion::UniquePtr uniqdistptr(distptr);
    std::shared_ptr<aslam::Camera> camera = std::make_shared<aslam::PinholeCamera>(cam_proj, camptr->w(), camptr->h(), uniqdistptr);
    camera->setId(aslam::createRandomId<aslam::SensorId>());
    cameras.push_back(camera);

    // Our camera extrinsics transform
    Eigen::Matrix4d T_CtoI = Eigen::Matrix4d::Identity();
    T_CtoI.block(0, 0, 3, 3) = ov_core::quat_2_Rot(_params.camera_extrinsics.at(i).block(0, 0, 4, 1)).transpose();
    T_CtoI.block(0, 3, 3, 1) = -T_CtoI.block(0, 0, 3, 3) * _params.camera_extrinsics.at(i).block(4, 0, 3, 1);
    Eigen::Matrix4d T_ItoC = ov_core::Inv_se3(T_CtoI);
    T_ItoCi.push_back(aslam::Transformation::constructAndRenormalizeRotation(T_ItoC));
    T_CitoI.push_back(aslam::Transformation::constructAndRenormalizeRotation(T_CtoI));

    // Debug print
    camera->printParameters(ss, "CAMERA " + std::to_string(i));
    ss << "T_ItoC" << std::endl << T_ItoCi.at(i) << std::endl;
  }
  PRINT_DEBUG(ss.str().c_str());

  //==========================================================================
  // CREATE OUR MAPLAB MAP OBJECTS!
  //==========================================================================

  // Create our IMU object
  constexpr char kImuHardwareId[] = "imu0";
  aslam::SensorId imu_sensor_id = aslam::createRandomId<aslam::SensorId>();
  vi_map::Imu::UniquePtr imu_sensor = aligned_unique<vi_map::Imu>(imu_sensor_id, static_cast<std::string>(kImuHardwareId));
  imu_sensor->setImuSigmas(imu_sigmas);
  imu_sensor->setGravityMagnitude(_params.gravity_mag);

  // Create our camera object
  std::string cam_label = "ncameras";
  aslam::NCameraId ncamera_id = aslam::createRandomId<aslam::NCameraId>();
  aslam::NCamera::UniquePtr ncamera = aligned_unique<aslam::NCamera>(ncamera_id, T_ItoCi, cameras, cam_label);

  // Sensor manager object
  // NOTE: Camera transformations are already relative to base sensor, so identity transform here
  sensor_manager = new vi_map::SensorManager();
  sensor_manager->addSensorAsBase<vi_map::Imu>(std::move(imu_sensor));
  // sensor_manager->addSensor<aslam::NCamera>(std::move(ncamera), imu_sensor_id, T_CitoI.at(0));
  sensor_manager->addSensor<aslam::NCamera>(std::move(ncamera), imu_sensor_id, aslam::Transformation());

  // Create the master map object
  nh->param<std::string>("map_save_folder", save_folder, "/datasets/maplab/");
  PRINT_INFO("[ov_maplab]: saving into %s\n", save_folder.c_str());
  if (boost::filesystem::exists(save_folder)) {
    PRINT_INFO(YELLOW "[ov_maplab]: removing old map folder...\n" RESET);
    boost::filesystem::remove_all(save_folder);
  }
  map = new vi_map::VIMap(save_folder);

  // Feature extractor
  aslam::NCamera::Ptr camera_rig = vi_map::getSelectedNCamera(*sensor_manager);
  feature_tracking::FeatureTrackingExtractorSettings extractor_settings;
  extractor_settings.rotation_invariant = true;
  feature_tracking::FeatureTrackingDetectorSettings detector_settings;
  feature_tracking::FeatureTrackingOutlierSettings outlier_settings;
  trackpipe = new feature_tracking::VOFeatureTrackingPipeline(camera_rig, extractor_settings, detector_settings, outlier_settings);

  // Create our map builder
  // NOTE: always dump the images to disk
  // FLAGS_map_builder_save_image_as_resources = true;
  builder = new online_map_builders::StreamMapBuilder(*sensor_manager, map);
}

void MapBuilder::callback_inertial(const sensor_msgs::Imu::ConstPtr &msg) {

  // convert into correct format
  ov_core::ImuData message;
  message.timestamp = msg->header.stamp.toSec();
  message.wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
  message.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

  // Append it to our vector
  imu_data.emplace_back(message);
}

void MapBuilder::callback_monocular(const sensor_msgs::ImageConstPtr &msg0, int cam_id0) {

  // Get the image
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    PRINT_ERROR(RED "[ov_maplab]: cv_bridge exception: %s\n" RESET, e.what());
    return;
  }

  // Create the measurement
  ov_core::CameraData message;
  message.timestamp = cv_ptr->header.stamp.toSec();
  message.sensor_ids.push_back(cam_id0);
  message.images.push_back(cv_ptr->image.clone());

  // Load the mask if we are using it, else it is empty
  // TODO: in the future we should get this from external pixel segmentation
  if (_app->get_params().use_mask) {
    message.masks.push_back(_app->get_params().masks.at(cam_id0));
  } else {
    message.masks.push_back(cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1));
  }

  // send it to our system
  feed_measurement_camera(message);
}

void MapBuilder::callback_stereo(const sensor_msgs::ImageConstPtr &msg0, const sensor_msgs::ImageConstPtr &msg1, int cam_id0, int cam_id1) {

  // Get the image
  cv_bridge::CvImageConstPtr cv_ptr0;
  try {
    cv_ptr0 = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    PRINT_ERROR(RED "[ov_maplab]: cv_bridge exception: %s\n" RESET, e.what());
    return;
  }

  // Get the image
  cv_bridge::CvImageConstPtr cv_ptr1;
  try {
    cv_ptr1 = cv_bridge::toCvShare(msg1, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    PRINT_ERROR(RED "[ov_maplab]: cv_bridge exception: %s\n" RESET, e.what());
    return;
  }

  // Create the measurement
  ov_core::CameraData message;
  message.timestamp = cv_ptr0->header.stamp.toSec();
  message.sensor_ids.push_back(cam_id0);
  message.sensor_ids.push_back(cam_id1);
  message.images.push_back(cv_ptr0->image.clone());
  message.images.push_back(cv_ptr1->image.clone());

  // Load the mask if we are using it, else it is empty
  // TODO: in the future we should get this from external pixel segmentation
  if (_app->get_params().use_mask) {
    message.masks.push_back(_app->get_params().masks.at(cam_id0));
    message.masks.push_back(_app->get_params().masks.at(cam_id1));
  } else {
    // message.masks.push_back(cv::Mat(cv_ptr0->image.rows, cv_ptr0->image.cols, CV_8UC1, cv::Scalar(255)));
    message.masks.push_back(cv::Mat::zeros(cv_ptr0->image.rows, cv_ptr0->image.cols, CV_8UC1));
    message.masks.push_back(cv::Mat::zeros(cv_ptr1->image.rows, cv_ptr1->image.cols, CV_8UC1));
  }

  // send it to our system
  feed_measurement_camera(message);
}

void MapBuilder::feed_measurement_camera(const ov_core::CameraData &message_tmp) {

  // Check that our MSCKF system has initialized yet
  auto rT1 = boost::posix_time::microsec_clock::local_time();
  if (!_app->initialized())
    return;

  // Return if we do not have the raw image recorded for the state time
  camera_data[message_tmp.timestamp] = message_tmp;
  if (camera_data.find(_app->get_state()->_timestamp) == camera_data.end()) {
    PRINT_WARNING(YELLOW "[ov_maplab]: unable to find recorded image for state %.15f\n" RESET, _app->get_state()->_timestamp);
    return;
  }
  ov_core::CameraData message = camera_data.at(_app->get_state()->_timestamp);

  // Erase all older camera timestamps from our history
  auto it0 = camera_data.begin();
  while (it0 != camera_data.end()) {
    if (it0->first <= _app->get_state()->_timestamp) {
      it0 = camera_data.erase(it0);
    } else {
      it0++;
    }
  }

  // Get the current timestamp in the imu clock frame of reference
  double t_ItoC = _app->get_state()->_calib_dt_CAMtoIMU->value()(0);
  double timestamp_inI = _app->get_state()->_timestamp + t_ItoC;

  // Our transformation
  Eigen::Matrix4d T_ItoG = Eigen::Matrix4d::Identity();
  T_ItoG.block(0, 0, 3, 3) = _app->get_state()->_imu->q()->Rot().transpose();
  T_ItoG.block(0, 3, 3, 1) = _app->get_state()->_imu->pos();

  // Create our VIO state object
  vio::MapUpdate update;
  update.timestamp_ns = (int64_t)(1e9 * timestamp_inI);
  update.vinode = vio::ViNodeState(aslam::Transformation::constructAndRenormalizeRotation(T_ItoG), _app->get_state()->_imu->vel(),
                                   _app->get_state()->_imu->bias_a(), _app->get_state()->_imu->bias_g());
  update.vio_state = vio::EstimatorState::kRunning;
  update.map_update_type = vio::UpdateType::kNormalUpdate;

  // Create the camera frame system
  aslam::NCamera::Ptr camera_rig = vi_map::getSelectedNCamera(*sensor_manager);
  aslam::VisualNFrame::Ptr nframe(new aslam::VisualNFrame(camera_rig));
  nframe->setId(aslam::createRandomId<aslam::NFramesId>());
  for (size_t i = 0; i < message.images.size(); i++) {
    std::shared_ptr<aslam::VisualFrame> frame = std::make_shared<aslam::VisualFrame>();
    frame->setCameraGeometry(camera_rig->getCameraShared(i));
    frame->setId(aslam::createRandomId<aslam::FrameId>());
    frame->setTimestampNanoseconds((int64_t)(1e9 * timestamp_inI));
    frame->clearKeypointChannels();
    frame->setRawImage(message.images.at(i).clone());
    nframe->setFrame(message.sensor_ids.at(i), frame);
  }
  assert(nframe->areAllFramesSet());
  assert(nframe->hasRawImagesInAllFrames());

  // Get IMU measurements between last and current vertex nodes
  // Also track features from the last frame to this one!
  // The first ever frame doesn't need its pose just the feature tracks
  if (last_timestamp == -1) {
    trackpipe->initializeFirstNFrame(nframe.get());
  } else {

    // First lets construct an IMU vector of measurements we need
    // TODO: delete old IMU measurements from here
    double time0 = last_timestamp + last_prop_time_offset;
    double time1 = message.timestamp + _app->get_state()->_calib_dt_CAMtoIMU->value()(0);
    std::vector<ov_core::ImuData> prop_data = Propagator::select_imu_readings(imu_data, time0, time1);

    // If empty, then we should skip this camera measurement
    if (prop_data.size() < 2) {
      PRINT_ERROR(RED "[ov_maplab]: not enough IMU between this and previous images!\n" RESET);
      return;
    }

    // Convert into the maplab format (order is acc, gyro??)
    update.imu_timestamps.resize(1, (int)prop_data.size());
    update.imu_measurements.resize(6, (int)prop_data.size());
    for (int i = 0; i < (int)prop_data.size(); i++) {
      update.imu_timestamps(i) = (int64_t)(1e9 * prop_data.at(i).timestamp);
      update.imu_measurements.block(0, i, 3, 1) = prop_data.at(i).am;
      update.imu_measurements.block(3, i, 3, 1) = prop_data.at(i).wm;
    }

    // Perform feature tracking into the newest nframe
    aslam::FrameToFrameMatchesList inlier_matches_kp1_k;
    aslam::FrameToFrameMatchesList outlier_matches_kp1_k;
    aslam::Transformation T_Bk_Bkp1 = update.vinode.get_T_M_I().inverse() * last_vinode.get_T_M_I();
    trackpipe->trackFeaturesNFrame(T_Bk_Bkp1.getRotation().inverse(), nframe.get(), last_nframe.get(), &inlier_matches_kp1_k,
                                   &outlier_matches_kp1_k);
  }

  // Send it to our map builder and store the raw images for later use
  update.keyframe = std::make_shared<const vio::SynchronizedNFrame>(nframe, vio::MotionType::kGeneralMotion);
  builder->apply(update, true);
  for (size_t frame_idx = 0u; frame_idx < nframe->getNumFrames(); frame_idx++) {
    map->storeFrameResource(nframe->getFrame(frame_idx).getRawImage(), frame_idx, backend::ResourceType::kRawImage,
                            map->getVertexPtr(builder->getLastVertexId()));
  }

  // Record the time we last propagated to
  last_timestamp = message.timestamp;
  last_prop_time_offset = _app->get_state()->_calib_dt_CAMtoIMU->value()(0);
  last_nframe = nframe;
  last_vinode = update.vinode;

  // Debug print timing info
  auto rT2 = boost::posix_time::microsec_clock::local_time();
  PRINT_INFO(BLUE "[TIME]: %.4f seconds for maplab vimap\n" RESET, (rT2 - rT1).total_microseconds() * 1e-6);
}

void MapBuilder::save_to_disk() {

  // Go through and extract things
  PRINT_INFO("[ov_maplab]: going to triangulate all tracked landmarks...\n");
  vi_map::MissionIdList mission_id_list;
  map->getAllMissionIds(&mission_id_list);
  assert(mission_id_list.size() == 1);
  vi_map_helpers::VIMapManipulation landmark_manipulation(map);
  landmark_manipulation.initializeLandmarksFromUnusedFeatureTracksOfMission(mission_id_list.at(0));
  landmark_triangulation::retriangulateLandmarksOfMission(mission_id_list.at(0), map);
  landmark_manipulation.removeBadLandmarks();

  // Check for a good map
  builder->checkConsistency();

  // Save to file
  PRINT_INFO("[ov_maplab]: saving to file!!!\n");
  backend::SaveConfig save_config;
  save_config.overwrite_existing_files = true;
  vi_map::serialization::saveMapToFolder(save_folder, save_config, map);
  PRINT_INFO("[ov_maplab]: saved to %s\n", save_folder.c_str());
}
