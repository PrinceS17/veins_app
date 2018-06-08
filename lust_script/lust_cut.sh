#!/bin/bash
# Author: Jinhui Song; Date: 2018.6.6
# This script is used to generate cut route given a reduced map and add file.
# It will generate a new folder for reduced sumo file except the sumocfg.
# Since I don't know how to write to file in shell now...
#
# Last updated:		2018.6.6

#input the file name
read -p "Input the sumo filename: " fname

# set the path for lust
LUST_HOME=~/Downloads/LuST/LuSTScenario

# begin the procedure: netedit, user should load add file, save "test" net & add file for later processing
cd $LUST_HOME/scenario/
echo "Please save the new net.xml and add.xml as test.net.xml and test.add.xml before exiting netedit."
read -p "Press Enter to continue..." a
netedit -s lust.net.xml		# manually select subarea and save file
mv test.net.xml $fname.net.xml
mv test.add.xml $fname.add.xml

# cut routes: generate $fname.rou.xml
$SUMO_HOME/tools/route/cutRoutes.py $fname.net.xml *.rou.xml DUARoutes/*.rou.xml --routes-output $fname.rou.xml --orig-net lust.net.xml 

# sort out all the file
mkdir ../$fname
mv $fname.* ../$fname/
cp vtypes* *poly* dua.actuated.sumocfg ../$fname/
cd ../$fname/
mv dua.actuated.sumocfg $fname.sumocfg

# modify sumocfg
read -p "Please edit sumocfg file to add route, net and additionals (add,poly and vtypes), press Enter to continue..." a
nano $fname.sumocfg

# choose how to run it
read -p "How do you like to run it? (1. sumo-gui; 2. sumo; 3. none): " num
if [ "$num" = "1" ]             # little confusing for a beginner like me
then
  echo "running sumo-gui..."
  sumo-gui -c $fname.sumocfg
elif [ "$num" = "2" ]
then
  echo "running sumo..."
  sumo -c $fname.sumocfg
else
  echo "Then finished."
fi

