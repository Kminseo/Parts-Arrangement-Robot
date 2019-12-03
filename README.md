# Parts Arrangement Robot
<img src="https://github.com/yehengchen/video_demo/blob/master/video_demo/grasp_detection.gif" width="80%" height="80%">

__Parts Arrangement Robot Demo Video - [[YouTube]](https://www.youtube.com/watch?v=gHX4VQ2hDhI)__
## Environment

* __Ubuntu 18.04__ 
* __ROS Melodic__
* __python 2.7__

***
## Installing

1. **Install ROS**

    Follow these [ROS Melodic installation instructions](http://wiki.ros.org/melodic/Installation).
    You can select any of the default configurations in step 1.4; even the
    ROS-Base (Bare Bones) package (`ros-melodic-ros-base`) is enough.

2. **Download the code**
    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/Kminseo/Parts-Arrangement-Robot.git
    ```

3. **Install python dependencies**
    ```
    $ cd ~/catkin_ws/src/Parts-Arrangement-Robot
    $ pip install -r requirements.txt
    ```

4. **Install ROS dependencies**
    ```
    $ cd ~/catkin_ws
    $ rosdep install --from-paths src -i --rosdistro melodic
    $ sudo apt-get install ros-melodic-rosbash ros-melodic-ros-comm
    ```

5. **Build**
    ```
    $ cd ~/catkin_ws
    $ catkin_make
    ```
## Quick Start

* `$ roslaunch gripper_ur5 gazebo_env_setting.launch`
* `$ roslaunch gripper_ur5_moveit_config moveit_planning_execution.launch`
* `$ roslaunch yolov3_pytorch_ros detector.launch`
* `$ rosrun gripper_ur5 robot_sorting_yolo.py`

## Node info - Launch 파일 설명

* __To simulate the robot environment launch the following:__
   
   ```
    $ roslaunch gripper_ur5 gazebo_env_setting.launch
    ```

    ###### *`gripper_ur5`패키지의 `gazebo_env_setting.launch`는 Gazebo Simulation환경 구성에 필요한 로봇, 테이블, 주변환경 등 불러오기 위한 launch파일.*

* __For setting up the MoveIt! nodes to allow motion planning run:__
    
    ```
    $ roslaunch gripper_ur5_moveit_config moveit_planning_execution.launch`
    ```

    ###### *UR5 제어를 위해  Moveit을 사용하는데, UR5에 맞게 설정된 Moveit 패키지를 불러오기 위해 사용하는 Launch 파일*

* __Start screw grasp detection node:__
 
    ```
     $ roslaunch yolov3_pytorch_ros detector.launch
     ```

    ###### *Gazebo 환경에 설치된 카메라를 사용해 yolov3로  Obejct Detection을 하기위한 구성된 패키지이며, `rqt`,`rvize`를 사용해서 환경에 배치된  `screw`3종을 검출 및 분류 하는 것이 가능하다.*

* __Start screw sorting robot node:__
    
    ```
    $ rosrun gripper_ur5 robot_sorting_yolo.py
    ```

    ###### *Moveit을 사용해로 로봇을 제어, 환경상의 3가지 종류의 스크류를 생성, yolov3를 포함하며 전체적인 Pick & Place에 대한 동작 코드를 포함합니다.*

 
