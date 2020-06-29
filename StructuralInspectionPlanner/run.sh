#!/bin/bash
source ../../devel/setup.bash
roslaunch koptplanner a380.launch > spaceSize_test/res_1_80_PI-9_spaceSize_500_230_300.txt &
sleep 5
for i in {10..50}
do
	echo "Nb_iter=$i"
	rosrun request a380 1 80 $i 500 230 500
done
