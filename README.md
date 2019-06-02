# Robotics Cource - Politecnico di Milano

## Team Members
Fabio Canazza, 920125  
Giacomo Bertollino, 919979  
Nikolaos Chairetis, 916092

## Goals
- Two different odometry methods: 
  1. Differential Drive Kinematics
  2. Ackerman model
- Publish them in two ways:
  1. tf
  2. odom topic
- Use dynamic configuration for:
  1. switch between different published odometry (Differential vs Ackerman)
  2. reset odometry to (0,0) or set to a specific (x,y) starting point
- Publish a custom message with odometry value and type of source

## Files
For this project, we created 5 files; one for the dynamic reconfiguration, one launch file, two files related with custom messages and one C++ file, in which our node exists
- **odom_param.cfg**:
We used one enum integer parameter (*type*) for the different types of model, another integer parameter (*reset*) for the reset and two double parameters (*x* and *y*) that jointly define the desired reconfigurable position for the car. The different levels of each parameter let us recognise the actual parameter that will be dynamically reconfigured.
- **car_odom.launch**:
A very simple launch file, which just runs the only implemented node, along with the `roscore` command. Since each bag will be manually started during the test, there isn't anything else needed.
- **floatStamped.msg**:
This custom message is about the type of topics published from the bag. Specifically we use the topics `speedR_stamped`, `speedL_stamped` and `steer_stamped`. The msg file structure is:
```
Header header
float64 data
```
- **odomCustom.msg**:
Here, we implement a custom message to be published containing information about the running node and the odometry of the car. Apart from the header, which contains info about time and frame name, we have included in the custom message the type of model that is used, the x-y position and the yaw angle of the car. The msg file structure is:
```
Header header
string type
float64 x
float64 y
float64 yaw
```
- **bag_odom.cpp**:
This is the main file of the project where the single node is implemented, along with every desirable function. The node *odom_car* is created as a class and initialized accordingly. Inside the class *Node*, all the used variables are defined in the private section and three functions (two of them are callbacks) are used to properly structure the problem. In the public section, apart from the variable initialization, we implement the following straightforward operations.
  1. subscribe to bag topics and synchronize them using the *ApproximateTime* policy. As an extension of this, the callback *SyncCallback* is used, where we immediately compute the velocity and the omega values from the bag contents, based on the used model.
  2. The dynamic configuration is set up and for this purpose we use the *dynamicCallback* callback function. When this callback is called, it means there is change in parameters and using the level value we recognize which operation should occur; reset to (0,0) or change odometry's model or place the car to a desired (x,y) position.
  3. Finally, we set up two subscribers in order to publish our custom message topic and the nav_msgs/Odometry topic and we initialize two time variables that will be used for the odometry computation.
The actual odometry computation occurs inside the function *odometry*, which is called from the callback SyncCallback every time we receive the synchronized messages and compute the velocity and the omega based on the selected model. The pose of the car is calculated using the Euler integration method. Then, we create and publish the tf transformation defining to frames; *world*, as the world origin, and *car*, which is self-explanatory. Finally, based on the pose values we set the messages' contents and publish them.

## How to use it
To integrate the project in your own workspace and run it properly, follow these simple steps:
- by extracting the contents of the zip file, a 'prj_1' folder should be created
- place it inside the `src` folder of your catkin workspace
- next we have to make sure that the python .cfg file is executable. Inside the terminal, type `chmod +x /path_to_file_folder/odom_param.cfg`
- next, cd to catkin workspace. As an intermediate step it's advisable to run as well `source devel/setup.bash`. Then compile the workspace, using `catkin_make`. Normally, there should be no errors.
- and type `roslaunch prj_1 car_odom.launch`<br>or alternatively, run manually `roscore` and then in another window/tab `rosrun prj_1 bag_odom`<br>Attention!! In order to get actual results, the bag should have been also executed.

To visualize the tf transformation:<br>in a separate window/tab: `rviz` and select 'add' to add a TF and change the 'fixed frame' to 'world'<br>
To change dynamically the parameters:
- change odometry model: `rosrun dynamic_reconfigure dynparam set /odom_car type (value)`, where the (value) is set to 0 for Differential Drive Kinematics and 1 for Ackerman Model
- reset (place car in (0,0) position): `rosrun dynamic_reconfigure dynparam set /odom_car reset 1`
- specific (x,y) position: `rosrun dynamic_reconfigure dynparam set /odom_car "{x:(value1), y:(value2)}"`, where (value1) correspongs to the value of X value and (value2) to that of Y

<br> During the demonstration, the tf tree should normally be similar with the following image
![tf-tree](https://raw.githubusercontent.com/nikchrts/prj_1/master/tf_tree.png)

## Further Considerations
* The bag should be manually executed in order the demonstration of the project to be meaningful.
* Our interprentation for the reset and the (x,y) position configuration is to only change accordingly the position of the car and all the other variables to remain exactly the same. That means that the car will be moved to the desired position and it will start moving to a different circular trajectory than the initial one, since the rest of the variables (eg yaw angle) do not experience a change.
* The car is moving initially using the Differential Drive Kinematics. 
