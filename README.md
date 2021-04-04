# warmup_project

## **Behaviors**
## Drive in Square:
### Description: 
This behavior causes the robot, upon initialization, to drive in a quare. I turned the robot after a certain number of seconds had elapsed and then drove in a straight line until I turned again. This is in a while loop that continues until shutdown.
### Code explanation: 
I created a class, DriveInSquare, with two functions; init(), and run(). In my init() function I initialize a rospy node: drive_in_square, a publisher: twist_pub, and a Twist variable: twist, composed of angular and linear velocities. The angular velocity is initialized at 0 and the linear velocity has an inital x value or 0.2. In my run() function I start a while loop that drives the robot in a straight line for 5s at 0.2 linear.x speed, then rotates the robot by setting angular.z velocity to 1 for 1.6 seconds and then sets angluar.z velocity back to zero. Then when the loop runs again, the robot moves in its new direction for 5s and drives in a square after every 4 iterations of the loop.
### Gif: 
![GitHub Logo](/home/josephinep/gifs/driveInSquare2.gif)
Format: ![Alt Text](url)
## **Challenges:** *TBD*
## **Future Work:** *TBD*
## **Takeaways:** *TBD*


