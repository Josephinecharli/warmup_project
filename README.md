# warmup_project

## **Behaviors**
## Drive in Square:
### Description: 
This behavior causes the robot, upon initialization, to drive in a quare. I turned the robot after a certain number of seconds had elapsed and then drove in a straight line until I turned again. This is in a while loop that continues until shutdown.
### Code explanation: 
I created a class, DriveInSquare, with two functions; init(), and run(). In my init() function I initialize a rospy node: drive_in_square, a publisher: twist_pub, and a Twist variable: twist, composed of angular and linear velocities. The angular velocity is initialized at 0 and the linear velocity has an inital x value or 0.2. In my run() function I start a while loop that drives the robot in a straight line for 5s at 0.2 linear.x speed, then rotates the robot by setting angular.z velocity to 1 for 1.6 seconds and then sets angluar.z velocity back to zero. Then when the loop runs again, the robot moves in its new direction for 5s and drives in a square after every 4 iterations of the loop.
### Gif: 
![driveInSquare2](./gifs/driveInSquare2.gif)
## Person Follower:
### Description: 
For this problem I made a robot that searched for an object and moved close to it. I divided up my situation into 3 cases: The robot sensor read infinity, the robot detected an object but was too far away, and the robot was at the target distance from an object. I then, respectively, either spun the robot until an object was sensed, moved towards the object, or stopped the robot.
### Code explanation: 
My code is based off the stop at wall file from class. I subscribe to /scan and publish to /cmd_vel in my init function. In my process scan function I check the sensor of the robot and implement the above functionality by altering the angular and linear velocities. Specifically, if the robot senses nothing, I set angular velocity to .3 to turn it until it finds an object. If it finds an object but is too far away, I set angular velocity to 0 so it doesn't spin anymore and set linear velocity to .1. If the robot is the target distance from the object I set both velocities to 0. In my main function I just keep rospy spinning.
### Gif: 
![person-follower](./gifs/person-follower.gif)
## Wall Follower:
### Description: 
This program has a robot search for a wall and then once it finds one, continuously follows the wall. I pull data from multiple points on the robots sensor to detect how close it was to any walls and corners and keep it a consistent distance from them on all sides. I use the front sensor to detect any upcoming corners and I use a sensor on the front right of the robot to detect how far the robot is from the adjacent wall (because my robot moves counter-clockwise).
### Code explanation: 
This program is also based off of stop at wall, so most of the initialization is the same. My process scan function is composed of 5 main conditionals, which check which state the robot is in and makes decisions based off of this sensor information.
* I first check if the robot is right against a wall, in which case I back up by setting linear velocity to -0.3
* If that case is not triggered I check if the front sensor is within .7 of the wall or if the side sensor is within the set buffer distance from the wall. If either of these are true, I turn the robot away from the wall slightly by setting angular velocity to .3 and linear velocity to .1.
* If neither of these are true, I check if the front sensor is within the set distance *(1)* from the wall. If it is I start turning a lot by setting angular velocity to .8 and linear velcoity to .3.
* If the front sensor is greater than *1* away from any wall, I have two cases:
    * If the robot has turned too far away from the wall I turn slightly back towards it by setting angular velocity to -.1 and linear velocity to .1
    * If not, I drive straight by setting angular velocity to 0 and linear velocity to .2
### Gif: 
![wall-follow](./gifs/wall-follow.gif)
## **Challenges:**
The main challenges I faced were with programming the behaviors of wall follower. Drive in Square and Person follower came relatively intuitively for me, but I struggled with wall follower a lot until I realized I needed to use multiple sensors on the robot. I was able to easily get my robot to follow the walls by driving into them and then rotating slowly and driving forward again, etc., but couldn't get my robot to not touch the walls. Once I started using the 360 degrees of sensors I was able to track where my robot was much better and control its movement with much more precision. I also ran into an issue with resetting the world, which made me think that my code had broken and I spent a while trying to debug this until I saw the very helpful post on slack (Thanks!!!).
## **Future Work:**
For Person Follower I think it would be really cool to implement proportional control to make it look more dynamic and also work faster. I would also like to implement functionality that allows the robot to go searching for objects if it doesnt see any within 3m on any side. For Wall Follower there are a lot of little fixes including making my robot drive smoother, faster, implement turns better, etc, but a major feature I would improve is making it be able to follow walls with doors, holes, etc.
## **Takeaways:**
* Taking a step back and thinking about the problem theoretically helps a lot. Doing this allows me to form a more targeted approach and list the steps out so I have more direction when coding. It also allows me to refine my ideas before coding them and cut out time lost from implementing solutions with fundamental errors.
* Looking at example code is very helpful so I dont waste a ton of time debugging modules and packages I dont understand. I was very stuck on accessing other degrees of my sensor untill I looked up documentation for the package and example code for how it was used. It ended up being very simple and in hindsight I really should not have struggled as much as I did. In the future I plan to read the documentation before trying to use packages I am unfamiliar with.



