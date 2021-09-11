# Intro

The repo is an offboard control example for Kerloud Flying Rover Series via python.
It can serve as the basis for further application development based on python.

Kerloud Flying Rover series: <https://cloudkerneltech.gitbook.io/kerloud-flyingrover/>

**Highlight**: The code is customized from the offboard control demo from [GAAS project](https://gaas.gitbook.io/guide/software-realization-build-your-own-autonomous-drone/build-your-own-autonomous-drone-e01-offboard-control-and-gazebo-simulation>),
and we owe sincere thanks to the original authors.

# Environment Requirements

Recommended environment: Ubuntu 18.04, ROS melodic and python 3.6.9

Dependencies: 
* mavros: <https://github.com/cloudkernel-tech/mavros>
* mavlink: <https://github.com/cloudkernel-tech/mavlink-gdp-release>
* python module requirements:

    python3 -m pip install rospkg pyquaternion

# How to run 

        # source the mavros workspace in another ros workspace
        source ~/src/catkinws_offboard/devel/setup.bash
        # launch the mavros node for real tests
        roslaunch mavros px4.launch fcu_url:="/dev/ttyPixhawk:921600"
        # launch another terminal
        python3 px4_mavros_run.py
        # launch another terminal
        python3 commander.py

# References:

Kerloud Flying Rover documentation: <https://cloudkerneltech.gitbook.io/kerloud-flyingrover/>

Detailed code explanation: <https://cloudkerneltech.gitbook.io/kerloud-flyingrover/user-guide/tutorials/offboard_python>

An offboard control example with mavros from PX4 community: <https://dev.px4.io/master/en/ros/mavros_offboard.html>

mavros wiki: <https://wiki.ros.org/mavros>

GAAS project: <https://gaas.gitbook.io/guide/>
