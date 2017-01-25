#!/bin/bash


# IMPORTANT, SCRIPT IS SUPPOSED TO RUN FROM A SUBFOLDER OF LAUNCH FOLDER

# Script to set and execute ros launch files.
# We have three parameters to play with
#
# radius              [2,10] step 1m             (5 times)
# weight increment    [0.05-0.55] step 0.1   (5 times)            
# weight decrement    [0.01-0.11] step 0.02  (5 times)            
#
# We have three recordings to play with:
#
#  1) FDG 20 db     4:00 
#  2) UOL 20 db    10:34
#  3) UOL 30 db     8:53
# TOTAL BAG RUN    23:27 = total replay times ~40 hours
#
# Results are pickle files, in subfolders numbered 1,2,3
# Folder code name is r#-i#.###-d#.###
# This is: folder 1/r4-i0.025-d0.010 has radius 4, weight increment 0.025 and weight decrement 0.10

r_min=2.0
r_step=1.0
r_max=6.0

wi_min=0.05
wi_step=0.1
wi_max=0.55

wd_min=0.01
wd_step=0.02
wd_max=0.11
i=0
testCounter=0
totalTests=375

## declare an array variable
declare -a testList=("REPLAY_FDG_1" "REPLAY_UOL_2" "REPLAY_UOL_3")
tLen=${#testList[@]}


for r in `seq $r_min  $r_step $r_max` # radius loop
do	
    for wi in `seq $wi_min  $wi_step $wi_max` # weight increment loop
    do	
        for wd in `seq $wd_min  $wd_step $wd_max` # weight decrement loop
        do	
            for (( i=0; i<${tLen}; i++ )); # tests loop
            do
               if [ -f ./doExit ]; then
                 echo "Early EXIT requested!"
                 exit 1
               fi
                

                counter=$((i + 1))            
                testCounter=$((testCounter + 1))            
                # set parameters in launch files
                # change save route using $folderName 
                saveFolder="$counter/r$r-i$wi-d$wd/"
                mkdir -p "./$saveFolder"
                prevText="/launch/article/$counter/"
                newText="/launch/article/$saveFolder"
                                
                mainLaunchFile="${testList[$i]}.launch"
                cp "$mainLaunchFile.orig" "$mainLaunchFile"
                sed -i -r "s#$prevText#$newText#" "$mainLaunchFile"

                #    "<arg name="saveRoute" value="$(find rfid_grid_map)/save/" />"
                prevText="/save/"
                sed -i -r "s#$prevText#$newText#" "$mainLaunchFile"
                

                # ........................................
                echo "running test # $testCounter of $totalTests"
                echo -e "Params:"
                echo -e "\t Test: \t ${testList[$i]}"                
                echo -e "\t r: \t $r"
                echo -e "\t wi: \t $wi"
                echo -e "\t wd: \t $wd"
                echo -e "...................................\n\n\n\n"
                
                # change grid_deploy params
                launchFile="../grid_deploy.launch"
                cp "$launchFile.orig" "$launchFile"
                #    "  <arg name="weight_inc" default="0.005"/> "                
                prevText="0.005"
                newText="$wi"                
                sed -i -r "s#$prevText#$newText#" "$launchFile"
                
                #    "  <arg name="weight_dec" default="0.001"/> "
                prevText="0.001"
                newText="$wd"                
                sed -i -r "s#$prevText#$newText#" "$launchFile"
                
                #    "  <arg name="detectRadius" default="2.0"/> "
                prevText="2.0"
                newText="$r"                
                sed -i -r "s#$prevText#$newText#" "$launchFile"     


                STARTTIME=$(date +%s)

                # now launch everything!!!           
                roslaunch "$mainLaunchFile"               
             
                ENDTIME=$(date +%s)


                echo -e "...................................\n\n\n\n"                
                echo "Bag took $(($ENDTIME - $STARTTIME)) to complete..."
                date 
                echo -e "...................................\n\n\n\n"
                                
                
            done  # tests loop
        done # weight decrement loop
    done # weight increment loop
done # radius loop

python /home/mfernandezcarmona/catkin_ws/src/ENRICHME/codes/ais/LibMercuryRFID/src/clients/ros/rol_server/scripts/dataPlot.py

