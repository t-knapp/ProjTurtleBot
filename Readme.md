# ROS Turtlebot Soccer Group A

## Description
Plays soccer 1on1.
Searchs red ball, detects enemies goal and kicks the ball.


## Dependencies 

- ROS Indigo
- Python 2.7.6
- OpenCV 2 (with Python binding)

## Launch

_Note:_ Copy Node shellscript to folder in $PATH and adjust PATH_SOCCER 
parameter. Create a copy of Node shellscript and rename to Node-Names.  
e.g.  
~/bin/NodeBallDetection   
~/bin/Node...   
~/bin/NodeStrategy   

_Note:_ Create shellscripts for roslaunch minimal.launch and roslaunch openni.launch    
in folder in $PATH  
e.g. ~/bin/minimal.bringup   

Then run:

```$ roslaunch turtlebot_bringup minimal.launch``` (or minimal.bringup if shellscript in PATH)   
```$ roslaunch openni_launch openni.launch``` (or openni.bringup if shellscript in PATH)   

```$ NodeBallDetection```   
```$ NodeGoalDetection``` (with -c for calibration; in game mode without -c)   
```$ NodeReferee``` (press stop in App, otherwise Bot will start immediately after NodeStrategy starts)   
```$ NodeKick```   
```$ NodeBallJourney```   
```$ NodeCollisionDetection```   
```$ NodeStrategy```    


## Ignore Lid Closing
The following instructions allow the closing of the lid without suspending.

1. Open the following file ```/etc/systemd/logind.conf```:  
        ```$ sudo -H gedit /etc/systemd/logind.conf```
2. Add the following file:  
         ```HandleLidSwitch=ignore```   
3. Restart the systemd daemon with this command:   
         ```$ sudo restart systemd-logind```
