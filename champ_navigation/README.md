## Go to Goa Goal Controller

## 1. Node Diagram

This diagram shows how the node fits into the ROS 2 ecosystem: it subscribes to `odom`, processes the data internally, and publishes velocity commands to `cmd_vel`.

```
        +---------+         (1) Robot publishes odometry
        | /odom   |         (nav_msgs/msg/Odometry)
        |(Odometry)|
        +----+----+
             |
             v
+--------------------------+
|  GoToGoalController Node | (2) The node receives odom,
| (C++ ROS 2 Node)         |     calculates control,
|                          |     and publishes cmd_vel.
|  Subscribes: /odom       |
|  Publishes:  /cmd_vel    |
+-----------+--------------+
            |
            v
     +------+-------+      (3) Node sends velocity commands
     |  /cmd_vel    |      (geometry_msgs/msg/Twist)
     |   (Twist)    |
     +--------------+
             |
             v
        +----------+
        |  Robot   | (4) Robot actuates wheels/motors
        +----------+
```

1. The robot or simulator produces **odometry** on the `/odom` topic.
2. **GoToGoalController** node subscribes to `/odom`, uses it to compute a velocity command, and publishes to `/cmd_vel`.
3. The robot (or simulation) subscribes to **/cmd_vel**, which steers the robot’s motion.

---

## 2. Logic Flow Diagram

This flowchart shows how the **controlLoop()** method handles the go-to-goal logic.

```
   +---------------------------------------------------+
   | Start controlLoop(): Retrieve current pose (x,y,θ)|
   +---------------------------------------------------+
                     |
                     v
   +---------------------------------------------------+
   | Compute dx = goal_x - current_x                   |
   |         dy = goal_y - current_y                   |
   | distance_error = sqrt(dx^2 + dy^2)                |
   | yaw_error = heading_to_goal - current_yaw         |
   |   where heading_to_goal = atan2(dy, dx)           |
   +---------------------------------------------------+
                     |
             Is distance_error > distance_tolerance?
                     |
            +--------+---------+
            | yes              | no
            v                  v
   +-------------------+    +---------------------------------+
   | cmd_vel.linear.x  |    |  // Already close to goal;      |
   |  = linear_gain *  |    |  // Check orientation:          |
   |    distance_error |    |  yaw_goal_error = (goal_yaw -   |
   +-------------------+    |                   current_yaw)  |
   | cmd_vel.angular.z |    +---------------------------------+
   |  = angular_gain * |
   |    yaw_error      |
   +-------------------+
                     |
                     v
   +---------------------------------------------------+
   |          If std::fabs(yaw_goal_error) > yaw_tolerance?  |
   |               (Are we within orientation tolerance?)     |
   +---------------------------------------------------+
                   | yes                no
                   v                    v
        +-----------------------------------+   +------------------------+
        | cmd_vel.linear.x = 0.0           |   |   cmd_vel.linear.x = 0 |
        | cmd_vel.angular.z = angular_gain * yaw_goal_error             |
        +-----------------------------------+
                   |
                   v
          +-------------------------------+
          | Publish cmd_vel over /cmd_vel|
          +-------------------------------+
```

1. **Distance Error**: The node first calculates the difference between the current position \((x, y)\) and the goal position \((goal_x, goal_y)\). If this distance exceeds `distance_tolerance`, it will move forward (proportional to `distance_error`) and turn (proportional to `yaw_error`).
2. **Orientation Correction**: Once within the distance tolerance, it checks orientation. If the current yaw is not within `yaw_tolerance` of `goal_yaw`, it rotates in place.
3. **Stop Condition**: When both distance and yaw are within their tolerances, the node sets linear and angular speeds to zero (the robot stops).

