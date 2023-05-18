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

#ifndef OV_MAPLAB_MAPBUILDER_H
#define OV_MAPLAB_MAPBUILDER_H

#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion-equidistant.h>
#include <aslam/cameras/distortion-radtan.h>
#include <aslam/cameras/distortion.h>
#include <aslam/cameras/ncamera.h>
#include <feature-tracking/vo-feature-tracking-pipeline.h>
#include <landmark-triangulation/landmark-triangulation.h>
#include <maplab-common/file-logger.h>
#include <maplab-common/map-manager-config.h>
#include <maplab-common/progress-bar.h>
#include <online-map-builders/stream-map-builder.h>
#include <vi-map-helpers/vi-map-manipulation.h>
#include <vi-map/sensor-utils.h>
#include <vi-map/vi-map-serialization.h>
#include <vio-common/map-update.h>
#include <vio-common/vio-types.h>
#include <vio-common/vio-update.h>
#include <visualization/viwls-graph-plotter.h>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/Propagator.h"
#include "state/State.h"

#include "utils/print.h"
#include "utils/sensor_data.h"

using namespace ov_msckf;

/**
 * @brief Class that will take the output of ov_msckf and save it to our maplab ViMap.
 *
 * This allows us to load this data into the maplab console and merge it with other maps.
 * The estimator VioManager should be called externally as here we *poll* to get information from it.
 */
class MapBuilder {
public:
  /**
   * @brief Default constructor
   * @param nh ROS node handler
   * @param app Core estimator manager
   * @param params Core estimator parameters
   */
  MapBuilder(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<VioManager> app, const VioManagerOptions &params);

  // Destructors to free maplab pointers
  ~MapBuilder() {
    delete builder;
    delete map;
    delete sensor_manager;
    delete trackpipe;
  }

  /// Callback for inertial information
  void callback_inertial(const sensor_msgs::Imu::ConstPtr &msg);

  /// Callback for monocular cameras information
  void callback_monocular(const sensor_msgs::ImageConstPtr &msg0, int cam_id0);

  /// Callback for synchronized stereo camera information
  void callback_stereo(const sensor_msgs::ImageConstPtr &msg0, const sensor_msgs::ImageConstPtr &msg1, int cam_id0, int cam_id1);

  /**
   * @brief This will save the current map to disk!
   * Should also re-run the extraction so we don't save the raw images...
   */
  void save_to_disk();

protected:
  /**
   * @brief Feed function for camera measurements
   * @param message Contains our timestamp, images, and camera ids
   */
  void feed_measurement_camera(const ov_core::CameraData &message);

  /// Our msckf filter estimator
  std::shared_ptr<VioManager> _app;

  /// Viomanger options and parameters
  VioManagerOptions _params;

  /// Master map object that we want to save data into
  vi_map::VIMap *map;

  // Feature tracker
  feature_tracking::VOFeatureTrackingPipeline *trackpipe;

  /// Master online stream builder
  online_map_builders::StreamMapBuilder *builder;

  /// Sensor manager (cameras and IMU)
  vi_map::SensorManager *sensor_manager;

  /// Our history of IMU messages (time, angular, linear)
  std::vector<ov_core::ImuData> imu_data;

  /// Our history of CAMERA messages
  std::map<double, ov_core::CameraData> camera_data;

  /// Estimate for time offset at last propagation time
  double last_prop_time_offset = 0;

  /// Last timestamp that we updated at
  double last_timestamp = -1;

  /// Last nframe with all images
  aslam::VisualNFrame::Ptr last_nframe = nullptr;

  /// Last VIO state
  vio::ViNodeState last_vinode;

  /// Save folder
  std::string save_folder;
};

#endif // OV_MAPLAB_MAPBUILDER_H
