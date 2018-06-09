#!/bin/bash
# Author Jinhui Song; Date: 6.9.2018
# This script helps add sumo project to veins and configure it.
#
# Last update:		6.8.2018	first released

# define path variables
VEINS_HOME=~/Documents/veins-veins-4.7.1/
VEINS_EX=$VEINS_HOME/examples/veins/
LUST_HOME=~/Downloads/LuST/LuSTScenario/

# get the name and rm the old sumo files
cd $VEINS_EX
temp=$(ls *launchd*)
prev=$(echo ${temp%.launchd.xml})
read -p "Are you sure to remove $prev.*.xml?(y/n)" ans
if [ "$ans" == "y" ]
then
 rm $prev.*.xml
 echo "$prev.*.xml all removed!"
else
 echo "$prev.*.xml all reserved."
fi

# copy the new sumo files
read -p "Input the sumo file name: " fname
cp $LUST_HOME/$fname/$fname.* .
echo "<?xml version=\"1.0\"?>
<!-- debug config -->
<launch>
        <copy file=\"$fname.net.xml\" />
        <copy file=\"$fname.rou.xml\" />
        <copy file=\"$fname.add.xml\" />
        <copy file=\"vtypes.add.xml\" />
        <copy file=\"lust.poly.xml\" />
        <copy file=\"$fname.sumocfg\" type=\"config\" />
</launch>
" >$fname.launchd.xml

# edit the omnetpp.ini
read -p "Will open omnetpp.ini: find launchd.xml to edit. Press Enter to continue..." a
gedit omnetpp.ini
echo "SUMO project configured."
