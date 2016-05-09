####################################################################################
############################ 1 MCLHMM WITH PRIME _ SAME LOOP #######################
####################################################################################
EXPERIMENT_PATH="/home/sina/indigo_workspace/src/experiment_launcher"
EXPERIMENT_BRANCH="experiment_big_map_same_loop"

cd $EXPERIMENT_PATH
git checkout $EXPERIMENT_BRANCH

NESTED_PATH="/home/sina/indigo_workspace/src/nested_amcl"
NESTED_BRANCH="NPF_AW_LMCHMM_CONT"

cd $NESTED_PATH
git checkout $NESTED_BRANCH

HMM_PATH="/home/sina/indigo_workspace/src/nested_amcl/MCFHMM"
HMM_BRANCH="Layered-MC-HMM"

cd $HMM_PATH
git checkout $HMM_BRANCH

WORKSPACE_PATH="/home/sina/indigo_workspace"
cd $WORKSPACE_PATH

#catkin_make

DUMP_PATH="/home/sina/indigo_workspace/dump_files"
cd $DUMP_PATH

RUN_IN_LOOP_PATH="/home/sina/indigo_workspace/src/experiment_launcher/shellscripts/run_in_loop.sh"
#sh $RUN_IN_LOOP_PATH >> "OUTPUT".$NESTED_BRANCH.$EXPERIMENT_BRANCH.txt
echo "SKIPPED THE FIRST SETUP"

#mkdir "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH
#mv LMCHMM* "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH
#mv OUTPUT* "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH


####################################################################################
############################ 2 MCLHMM NO PRIME _ SAME LOOP #########################
####################################################################################
EXPERIMENT_PATH="/home/sina/indigo_workspace/src/experiment_launcher"
EXPERIMENT_BRANCH="experiment_big_map_same_loop"

cd $EXPERIMENT_PATH
git checkout $EXPERIMENT_BRANCH

NESTED_PATH="/home/sina/indigo_workspace/src/nested_amcl"
NESTED_BRANCH="NPF_AW_LMCHMM_CONT_NO_PRIME"

cd $NESTED_PATH
git checkout $NESTED_BRANCH

HMM_PATH="/home/sina/indigo_workspace/src/nested_amcl/MCFHMM"
HMM_BRANCH="Layered-MC-HMM"

cd $HMM_PATH
git checkout $HMM_BRANCH

WORKSPACE_PATH="/home/sina/indigo_workspace"
cd $WORKSPACE_PATH

catkin_make

DUMP_PATH="/home/sina/indigo_workspace/dump_files"
cd $DUMP_PATH

RUN_IN_LOOP_PATH="/home/sina/indigo_workspace/src/experiment_launcher/shellscripts/run_in_loop.sh"
sh $RUN_IN_LOOP_PATH >> "OUTPUT".$NESTED_BRANCH.$EXPERIMENT_BRANCH.txt

mkdir "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH
mv LMCHMM_NO* "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH
mv OUTPUT* "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH


####################################################################################
############################ 3 STATIC _ SAME LOOP ##################################
####################################################################################
EXPERIMENT_PATH="/home/sina/indigo_workspace/src/experiment_launcher"
EXPERIMENT_BRANCH="experiment_big_map_same_loop"

cd $EXPERIMENT_PATH
git checkout $EXPERIMENT_BRANCH

NESTED_PATH="/home/sina/indigo_workspace/src/nested_amcl"
NESTED_BRANCH="NPF_AW_Static_Motion_Model"

cd $NESTED_PATH
git checkout $NESTED_BRANCH

HMM_PATH="/home/sina/indigo_workspace/src/nested_amcl/MCFHMM"
HMM_BRANCH="Layered-MC-HMM"

cd $HMM_PATH
git checkout $HMM_BRANCH

WORKSPACE_PATH="/home/sina/indigo_workspace"
cd $WORKSPACE_PATH

catkin_make

DUMP_PATH="/home/sina/indigo_workspace/dump_files"
cd $DUMP_PATH

RUN_IN_LOOP_PATH="/home/sina/indigo_workspace/src/experiment_launcher/shellscripts/run_in_loop.sh"
sh $RUN_IN_LOOP_PATH >> "OUTPUT".$NESTED_BRANCH.$EXPERIMENT_BRANCH.txt

mkdir "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH
mv STATIC* "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH
mv OUTPUT* "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH


####################################################################################
############################ 4 RANDOM _ SAME LOOP ##################################
####################################################################################
EXPERIMENT_PATH="/home/sina/indigo_workspace/src/experiment_launcher"
EXPERIMENT_BRANCH="experiment_big_map_same_loop"

cd $EXPERIMENT_PATH
git checkout $EXPERIMENT_BRANCH

NESTED_PATH="/home/sina/indigo_workspace/src/nested_amcl"
NESTED_BRANCH="NPF_AW_Random_Action"

cd $NESTED_PATH
git checkout $NESTED_BRANCH

HMM_PATH="/home/sina/indigo_workspace/src/nested_amcl/MCFHMM"
HMM_BRANCH="Layered-MC-HMM"

cd $HMM_PATH
git checkout $HMM_BRANCH

WORKSPACE_PATH="/home/sina/indigo_workspace"
cd $WORKSPACE_PATH

catkin_make

DUMP_PATH="/home/sina/indigo_workspace/dump_files"
cd $DUMP_PATH

RUN_IN_LOOP_PATH="/home/sina/indigo_workspace/src/experiment_launcher/shellscripts/run_in_loop.sh"
sh $RUN_IN_LOOP_PATH >> "OUTPUT".$NESTED_BRANCH.$EXPERIMENT_BRANCH.txt

mkdir "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH
mv RANDOM* "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH
mv OUTPUT* "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH


####################################################################################
############################ 5 MCLHMM WITH PRIME _ 3 Robots ########################
####################################################################################
EXPERIMENT_PATH="/home/sina/indigo_workspace/src/experiment_launcher"
EXPERIMENT_BRANCH="experiment_big_map_three_robots"

cd $EXPERIMENT_PATH
git checkout $EXPERIMENT_BRANCH

NESTED_PATH="/home/sina/indigo_workspace/src/nested_amcl"
NESTED_BRANCH="NPF_AW_LMCHMM_CONT"

cd $NESTED_PATH
git checkout $NESTED_BRANCH

HMM_PATH="/home/sina/indigo_workspace/src/nested_amcl/MCFHMM"
HMM_BRANCH="Layered-MC-HMM"

cd $HMM_PATH
git checkout $HMM_BRANCH

WORKSPACE_PATH="/home/sina/indigo_workspace"
cd $WORKSPACE_PATH

catkin_make

DUMP_PATH="/home/sina/indigo_workspace/dump_files"
cd $DUMP_PATH

RUN_IN_LOOP_PATH="/home/sina/indigo_workspace/src/experiment_launcher/shellscripts/run_in_loop.sh"
sh $RUN_IN_LOOP_PATH >> "OUTPUT".$NESTED_BRANCH.$EXPERIMENT_BRANCH.txt

mkdir "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH
mv LMCHMM* "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH
mv OUTPUT* "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH


####################################################################################
############################ 6 MCLHMM NO PRIME _ 3 Robots ##########################
####################################################################################
EXPERIMENT_PATH="/home/sina/indigo_workspace/src/experiment_launcher"
EXPERIMENT_BRANCH="experiment_big_map_three_robots"

cd $EXPERIMENT_PATH
git checkout $EXPERIMENT_BRANCH

NESTED_PATH="/home/sina/indigo_workspace/src/nested_amcl"
NESTED_BRANCH="NPF_AW_LMCHMM_CONT_NO_PRIME"

cd $NESTED_PATH
git checkout $NESTED_BRANCH

HMM_PATH="/home/sina/indigo_workspace/src/nested_amcl/MCFHMM"
HMM_BRANCH="Layered-MC-HMM"

cd $HMM_PATH
git checkout $HMM_BRANCH

WORKSPACE_PATH="/home/sina/indigo_workspace"
cd $WORKSPACE_PATH

catkin_make

DUMP_PATH="/home/sina/indigo_workspace/dump_files"
cd $DUMP_PATH

RUN_IN_LOOP_PATH="/home/sina/indigo_workspace/src/experiment_launcher/shellscripts/run_in_loop.sh"
sh $RUN_IN_LOOP_PATH >> "OUTPUT".$NESTED_BRANCH.$EXPERIMENT_BRANCH.txt

mkdir "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH
mv LMCHMM_NO* "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH
mv OUTPUT* "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH


####################################################################################
############################ 7 STATIC _ 3 Robots ###################################
####################################################################################
EXPERIMENT_PATH="/home/sina/indigo_workspace/src/experiment_launcher"
EXPERIMENT_BRANCH="experiment_big_map_three_robots"

cd $EXPERIMENT_PATH
git checkout $EXPERIMENT_BRANCH

NESTED_PATH="/home/sina/indigo_workspace/src/nested_amcl"
NESTED_BRANCH="NPF_AW_Static_Motion_Model"

cd $NESTED_PATH
git checkout $NESTED_BRANCH

HMM_PATH="/home/sina/indigo_workspace/src/nested_amcl/MCFHMM"
HMM_BRANCH="Layered-MC-HMM"

cd $HMM_PATH
git checkout $HMM_BRANCH

WORKSPACE_PATH="/home/sina/indigo_workspace"
cd $WORKSPACE_PATH

catkin_make

DUMP_PATH="/home/sina/indigo_workspace/dump_files"
cd $DUMP_PATH

RUN_IN_LOOP_PATH="/home/sina/indigo_workspace/src/experiment_launcher/shellscripts/run_in_loop.sh"
sh $RUN_IN_LOOP_PATH >> "OUTPUT".$NESTED_BRANCH.$EXPERIMENT_BRANCH.txt

mkdir "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH
mv STATIC* "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH
mv OUTPUT* "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH


####################################################################################
############################ 8 RANDOM _ 3 Robots ###################################
####################################################################################
EXPERIMENT_PATH="/home/sina/indigo_workspace/src/experiment_launcher"
EXPERIMENT_BRANCH="experiment_big_map_three_robots"

cd $EXPERIMENT_PATH
git checkout $EXPERIMENT_BRANCH

NESTED_PATH="/home/sina/indigo_workspace/src/nested_amcl"
NESTED_BRANCH="NPF_AW_Random_Action"

cd $NESTED_PATH
git checkout $NESTED_BRANCH

HMM_PATH="/home/sina/indigo_workspace/src/nested_amcl/MCFHMM"
HMM_BRANCH="Layered-MC-HMM"

cd $HMM_PATH
git checkout $HMM_BRANCH

WORKSPACE_PATH="/home/sina/indigo_workspace"
cd $WORKSPACE_PATH

catkin_make

DUMP_PATH="/home/sina/indigo_workspace/dump_files"
cd $DUMP_PATH

RUN_IN_LOOP_PATH="/home/sina/indigo_workspace/src/experiment_launcher/shellscripts/run_in_loop.sh"
sh $RUN_IN_LOOP_PATH >> "OUTPUT".$NESTED_BRANCH.$EXPERIMENT_BRANCH.txt

mkdir "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH
mv RANDOM* "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH
mv OUTPUT* "DATA".$NESTED_BRANCH.$EXPERIMENT_BRANCH