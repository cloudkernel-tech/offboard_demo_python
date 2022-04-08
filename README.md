# Intro

The repo is an offboard control example for Kerloud Flying Rover Series via python.
It can serve as the basis for further application development based on python.

Kerloud Flying Rover series: <https://cloudkerneltech.gitbook.io/kerloud-flyingrover/>

**Highlight**: The code is customized from the offboard control demo from [GAAS project](https://gaas.gitbook.io/guide/software-realization-build-your-own-autonomous-drone/build-your-own-autonomous-drone-e01-offboard-control-and-gazebo-simulation>),
and we owe sincere thanks to the original authors.

# Environment Requirements

Recommended environment: Ubuntu 18.04, ROS melodic and python 3.6.9

Dependencies: 
* mavros (dev_flyingrover branch): <https://github.com/cloudkernel-tech/mavros>
* mavlink (dev_flyingrover branch): <https://github.com/cloudkernel-tech/mavlink-gdp-release>
* python module requirements:

    python3 -m pip install rospkg pyquaternion

# How to run 


## Simulation test

        # Users have to start the Kerloud flying rover SITL space first  
        
        # terminal 1
        # source the mavros package in the c++ offboard ros workspace
        cd ~/src/rover_workspace/offboard_demo_python
        source ../catkinws_offboard/devel/setup.bash
        # launch the mavros node for real tests
        roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
        
        # terminal 2
        cd ~/src/rover_workspace/offboard_demo_python
        source ../catkinws_offboard/devel/setup.bash
        python3 px4_mavros_run.py --sim True
        
        # terminal 3
        cd ~/src/rover_workspace/offboard_demo_python
        source ../catkinws_offboard/devel/setup.bash
        python3 commander.py

## Real test

        # terminal 1
        # source the mavros package in the c++ offboard ros workspace
        cd ~/src/flyingrover_workspace/offboard_demo_python
        source ../catkinws_offboard/devel/setup.bash
        # launch the mavros node for real tests
        roslaunch mavros px4.launch fcu_url:="/dev/ttyPixhawk:921600"
        
        # terminal 2
        cd ~/src/flyingrover_workspace/offboard_demo_python
        source ../catkinws_offboard/devel/setup.bash
        python3 px4_mavros_run.py
        
        # terminal 3
        cd ~/src/flyingrover_workspace/offboard_demo_python
        source ../catkinws_offboard/devel/setup.bash
        python3 commander.py

# References:

* Kerloud Flying Rover documentation: <https://cloudkerneltech.gitbook.io/kerloud-flyingrover/>

* Detailed code explanation: <https://cloudkerneltech.gitbook.io/kerloud-flyingrover/user-guide/tutorials/offboard_python>

* An offboard control example with mavros from PX4 community: <https://dev.px4.io/master/en/ros/mavros_offboard.html>

* mavros wiki: <https://wiki.ros.org/mavros>

* GAAS project: <https://gaas.gitbook.io/guide/>
