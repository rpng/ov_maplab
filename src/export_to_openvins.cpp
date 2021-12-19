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

#include <console-common/console.h>
#include <map-manager/map-manager.h>
#include <maplab-common/progress-bar.h>
#include <vi-map/vi-map.h>

DEFINE_string(export_path, "", "Path to save the exported text files into.");

/**
 * @brief This plug provides a simple export utilities to save the optimized trajectory to file.
 * This would typically be used if you want to use the optimized trajectory as groundtruth.
 */
class ExportToOpenvVINSPlugin : public common::ConsolePluginBase {
public:
  // Every plugin needs to implement a getPluginId function which returns a
  // string that gives each plugin a unique name.
  std::string getPluginId() const override { return "export_to_openvins"; }

  // The constructor takes a pointer to the Console object which we can forward
  // to the constructor of ConsolePluginBase.
  ExportToOpenvVINSPlugin(common::Console *console) : common::ConsolePluginBase(console) {
    // You can add your commands in here.
    addCommand(
        {"export_to_openvins"},
        [this]() -> int {
          // Get the currently selected map.
          std::string selected_map_key;

          // This function will write the name of the selected map key into
          // selected_map_key. The function will return false and print an error
          // message if no map key is selected.
          if (!getSelectedMapKeyIfSet(&selected_map_key)) {
            return common::kStupidUserError;
          }

          // Get the directory that the user wants to save the file into
          const std::string &save_path = FLAGS_export_path;
          if (save_path.empty()) {
            LOG(ERROR) << "[EXPORT]: No path to export the text files into has been "
                          "specified. Please specify using the --export_path flag.";
            return common::kStupidUserError;
          }

          // Create a map manager instance.
          vi_map::VIMapManager map_manager;

          // Get and lock the map which blocks all other access to the map.
          vi_map::VIMapManager::MapWriteAccess map = map_manager.getMapWriteAccess(selected_map_key);

          // Now run your algorithm on the VI map.
          // E.g., we can get the number of missions and print it.
          const size_t num_missions = map->numMissions();
          std::cout << "[EXPORT]: the VI map " << selected_map_key << " contains " << num_missions << " missions." << std::endl;

          // Get all missions in this map and loop through them
          vi_map::MissionIdList all_mission_ids;
          map->getAllMissionIds(&all_mission_ids);

          for (const vi_map::MissionId &mission_id : all_mission_ids) {

            // Create the base folder and the final exported file
            CHECK(common::createPath(save_path));
            const std::string base_path_for_mission = common::concatenateFolderAndFileName(save_path, mission_id.hexString() + ".txt");

            // Open this file we want to write to
            std::ofstream outfile;
            outfile.open(base_path_for_mission.c_str());
            if (outfile.fail()) {
              LOG(ERROR) << "[EXPORT]: unable to open filepath: " << base_path_for_mission.c_str();
              return common::kStupidUserError;
            }
            outfile << "# timestamp(s) tx ty tz qx qy qz qw" << std::endl;

            // Loop through each vertex in this mission
            LOG(INFO) << "[EXPORT]: starting export for mission " << mission_id << ".";
            pose_graph::VertexIdList vertex_ids_in_mission;
            map->getAllVertexIdsInMissionAlongGraph(mission_id, &vertex_ids_in_mission);
            common::ProgressBar progress_bar(vertex_ids_in_mission.size());
            for (const pose_graph::VertexId &vertex_id : vertex_ids_in_mission) {

              // Get this vertex
              const vi_map::Vertex &vertex = map->getVertex(vertex_id);
              double timestamp = 1e-9 * vertex.getMinTimestampNanoseconds();
              Eigen::Vector4d q_ItoM = vertex.get_q_M_I().coeffs();
              Eigen::Vector3d p_IinM = vertex.get_p_M_I();

              // timestamp
              outfile.precision(5);
              outfile.setf(std::ios::fixed, std::ios::floatfield);
              outfile << timestamp << " ";

              // pose
              outfile.precision(6);
              outfile << p_IinM.x() << " " << p_IinM.y() << " " << p_IinM.z() << " " << q_ItoM(0) << " " << q_ItoM(1) << " " << q_ItoM(2)
                      << " " << q_ItoM(3) << std::endl;

              // Move forward in time
              progress_bar.increment();
            }

            // Finally close the file
            outfile.close();
          }

          return common::kSuccess;
        },

        // This is the description of your command.
        // This will get printed when you run `help` in the console.
        "Export the selected vimap trajectories to space separated text file [time(sec),q_GtoI,p_IinG].",

        // This specifies the execution method of your command. For most
        // commands, it is sufficient to run them in sync with common::Processing::Sync.
        common::Processing::Sync);
  }
};

// Finally, call the MAPLAB_CREATE_CONSOLE_PLUGIN macro to create your console plugin.
MAPLAB_CREATE_CONSOLE_PLUGIN(ExportToOpenvVINSPlugin);
