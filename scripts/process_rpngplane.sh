#!/usr/bin/env bash

# Source our workspace directory to load ENV variables
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source ${SCRIPT_DIR}/../../../devel/setup.bash

#=============================================================
#=============================================================
#=============================================================

# dataset locations
bagnames=(
  "table_01"
  "table_02"
  "table_03"
  "table_04"
  "table_05"
  "table_06"
  "table_07"
  "table_08"
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
  max_cameras:="1" \
  use_stereo:="true" \
  config:="rpng_plane" \
  dataset:="${bagnames[i]}" \
  dolivetraj:="true" &> /dev/null

# print out the time elapsed
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${bagnames[i]} - vio took $elapsed seconds";

done

# start timing
start_time="$(date -u +%s)"

# run our maplab batch processing command
rosrun maplab_console batch_runner --batch_control_file=src/ov_maplab/scripts/commands_rpngplane.yaml

# print out the time elapsed
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: full maplab opt took $elapsed seconds";

# fix our permissions...
sudo chmod -R 777 /datasets/rpng_plane/maplab/
sudo chown -R patrick /datasets/rpng_plane/maplab/

# print out the time elapsed
big_end_time="$(date -u +%s)"
big_elapsed="$(($big_end_time-$big_start_time))"
echo "BASH: script took $big_elapsed seconds in total!!";