#!/usr/bin/env bash

# Source our workspace directory to load ENV variables
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source ${SCRIPT_DIR}/../../../devel/setup.bash

#=============================================================
#=============================================================
#=============================================================

# dataset locations
bagnames=(
  "V1_01_easy"
  "V1_02_medium"
  "V1_03_difficult"
  "V2_01_easy"
  "V2_02_medium"
  "V2_03_difficult"
)

#=============================================================
#=============================================================
#=============================================================

big_start_time="$(date -u +%s)"

# Loop through all datasets
for i in "${!bagnames[@]}"; do

# start timing
start_time="$(date -u +%s)"

# run our ROS launch file (note we send console output to terminator)
roslaunch ov_maplab serial.launch \
  max_cameras:="2" \
  use_stereo:="true" \
  config:="euroc_mav" \
  dataset:="${bagnames[i]}" \
  mapfolder:="/datasets/euroc_mav/maplab/raw_maps/${bagnames[i]}/" \
  dolivetraj:="true" &> /dev/null

# print out the time elapsed
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${bagnames[i]} - vio took $elapsed seconds";

done

# start timing
start_time="$(date -u +%s)"

# run our maplab batch processing command
rosrun maplab_console batch_runner --batch_control_file=src/ov_maplab/scripts/commands_eurocmav.yaml

# print out the time elapsed
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: full maplab opt took $elapsed seconds";

# fix our permissions...
sudo chmod -R 777 /datasets/euroc_mav/maplab/

# print out the time elapsed
big_end_time="$(date -u +%s)"
big_elapsed="$(($big_end_time-$big_start_time))"
echo "BASH: script took $big_elapsed seconds in total!!";