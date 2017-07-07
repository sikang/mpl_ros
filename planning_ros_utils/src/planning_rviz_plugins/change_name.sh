#!/bin/bash

for file in $(find . -name "*.cpp" -o -name "*.h"); do
  sed -i -e 's/mav_rviz_plugins/planning_rviz_plugins/g' $file

  echo "changed $file"
done
