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

#ifndef OV_MAPLAB_MAPBUILDER_H
#define OV_MAPLAB_MAPBUILDER_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/filesystem.hpp>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion.h>
#include <aslam/cameras/distortion-equidistant.h>
#include <aslam/cameras/distortion-radtan.h>
#include <aslam/cameras/ncamera.h>
#include <vio-common/vio-types.h>
#include <vio-common/vio-update.h>
#include <maplab-common/file-logger.h>
#include <maplab-common/map-manager-config.h>
#include <online-map-builders/stream-map-builder.h>
#include <visualization/viwls-graph-plotter.h>
#include <vi-map/vi-map-serialization.h>
#include <feature-tracking/vo-feature-tracking-pipeline.h>


#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/Propagator.h"


using namespace ov_msckf;


/**
 * @brief Class that will take the output of ov_msckf and save it to our maplab ViMap.
 * This allows us to load this data into the maplab console and merge it with other maps.
 */
class MapBuilder {

public:

    /**
     * @brief Default constructor
     * @param nh ROS node handler
     * @param app Core estimator manager
     * @param params Core estimator parameters
     */
    MapBuilder(ros::NodeHandle &nh, VioManager* app, const VioManagerOptions &params);

    /**
     * @brief Feed function for inertial data
     * @param timestamp Time of the inertial measurement
     * @param wm Angular velocity
     * @param am Linear acceleration
     */
    void feed_measurement_imu(double timestamp, Eigen::Vector3d wm, Eigen::Vector3d am) {
        // Create our imu data object
        Propagator::IMUDATA data;
        data.timestamp = timestamp;
        data.wm = wm;
        data.am = am;
        // Append it to our vector
        imu_data.emplace_back(data);
    }


    /**
     * @brief Feed function for a single camera
     * @param timestamp Time that this image was collected
     * @param images Set of cameras for this timestamp
     * @param cam_ids Set of ids for this camera
     */
    void feed_measurement_camera(double timestamp, std::vector<cv::Mat> &images, std::vector<size_t> &cam_ids);


    /**
     * @brief This will save the current map to disk!
     * Should also re-run the extraction so we don't save the raw images...
     */
    void save_to_disk();



protected:

    /// Our msckf filter estimator
    VioManager* _app;

    /// Viomanger options and parameters
    VioManagerOptions _params;

    /// Master map object that we want to save data into
    vi_map::VIMap* map;

    /// Our camera rig
    std::shared_ptr<aslam::NCamera> camera_rig;

    /// Master online stream builder
    online_map_builders::StreamMapBuilder* builder;

    /// Our history of IMU messages (time, angular, linear)
    std::vector<Propagator::IMUDATA> imu_data;

    /// Estimate for time offset at last propagation time
    double last_prop_time_offset = 0;

    /// Last timestamp that we updated at
    double last_timestamp = -1;

    /// Save folder
    std::string save_folder;

    /**
     * @brief Nice helper function that will linearly interpolate between two imu messages.
     *
     * This should be used instead of just "cutting" imu messages that bound the camera times
     * Give better time offset if we use this function, could try other orders/splines if the imu is slow.
     *
     * @param imu_1 imu at beggining of interpolation interval
     * @param imu_2 imu at end of interpolation interval
     * @param timestamp Timestamp being interpolated to
     */
    Propagator::IMUDATA interpolate_data(const Propagator::IMUDATA imu_1, const Propagator::IMUDATA imu_2, double timestamp) {
        // time-distance lambda
        double lambda = (timestamp - imu_1.timestamp) / (imu_2.timestamp - imu_1.timestamp);
        //cout << "lambda - " << lambda << endl;
        // interpolate between the two times
        Propagator::IMUDATA data;
        data.timestamp = timestamp;
        data.am = (1 - lambda) * imu_1.am + lambda * imu_2.am;
        data.wm = (1 - lambda) * imu_1.wm + lambda * imu_2.wm;
        return data;
    }

};


#endif //OV_MAPLAB_MAPBUILDER_H
