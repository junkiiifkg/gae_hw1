# 1-TURTLESIM AND KEYBOARD CONTROLLER
In this homework i made a keyboard controller for turtlesim and i was face to face an extra challange because i am using WSL

# Publish and controller
– Publishes geometry msgs/msg/Twist messages to the topic /turtle1/cmd vel
– Allows manual control using the following keys:
w - move forward
s - move backward
a - turn left
d - turn right
x - mode changes between "autonomous" and "manual"

# Path Following with Stanley Controller
Main mission is: Implement a Stanley Controller that follows a predefined path
and evaluates total tracking error.

# Extra challange for wsl
You cant take do this mission with reading from terminal because its cant take two data at the same time and some libraries like "evdev" cant usable for wsl
but with "SDL2" you can make this perfectly

# Main Tasks
• Create your own path publisher node instead of using a provided one.
The node should publish a simple path (as a list of positions) that the
turtle will follow.
• Define the path as a sequence of waypoints in 2D space. You may
create the path in any shape (e.g., straight line, curve, square) but it
should be continuous. The turtle starts at approximately the center of
the Turtlesim window, around coordinates (5.5, 5.5).
• The path can be published using either:
– nav msgs/msg/Path message (recommended), or
– a custom message containing an array of (x, y, θ) coordinates,
where θ represents the orientation (yaw angle) at each waypoint.

# Stanley Method
The Stanley method is the path tracking approach used by Stanford University’s autonomous vehicle entry in
the DARPA Grand Challenge, Stanley. The Stanley method is a nonlinear feedback function of the cross track error
efa, measured from the center of the front axle to the nearest path point (cx, cy), for which exponential convergence
can be shown. Co-locating the point of control with the steered front wheels allows for an intuitive control law,
where the first term simply keeps the wheels aligned with the given path by setting the steering angle δ equal to the
heading error
θe = θ − θp,
where θ is the heading of the vehicle and θp is the heading of the path at (cx, cy). When efa is non-zero, the second
term adjusts δ such that the intended trajectory intersects the path tangent from (cx, cy) at kv(t) units from the front
axle. Figure 14 illustrates the geometric relationship of the control parameters. The resulting steering control law is
given as
<img width="273" height="58" alt="image" src="https://github.com/user-attachments/assets/7370c960-491a-4368-b0d6-0c20fb976705" />
where k is a gain parameter. It is clear that the desired effect is achieved with this control law: As efa increases, the
wheels are steered further towards the path.
<img width="490" height="261" alt="image" src="https://github.com/user-attachments/assets/ef96550e-fc16-4c0f-a228-bf7978ce3d0d" />



# publishers and subscribers
I create a path with and publish it to "path_topic" and for stanley_controller subscribe for two topics: "path_topic" and "turtle/pose"
and a publisher to "/turtle1/cmd_vel" topic. In the last node publish turtle_pose_to_rviz subscribe to one topics: "/turtle1/pose" 
create two publisher: "/turtle1/pose_stamped" and "/turtle1/odom"

# Example video
https://youtu.be/Uz0VTQnGJyw
