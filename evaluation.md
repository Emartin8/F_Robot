# Evaluation

**The day of the evaluation you'll be asked to demonstrate the behaviors your robot can execute. Each group will have ~10 minutes to do so and answer a few questions. A mark will be given to the group, but keep in mind that the final mark is individual as it takes into account the continuous evaluation.**

**Warning:** Not using the inverse kinematics for the validation of a behavior is a crime (unless prior discussion). Your mark will be 0, and you might even go to jail.

**Note:** Originality is absolutely encouraged, showing interesting behaviors or approaches during the evaluation can only be good for you.

**Mandatory behaviors:**

   - Moving an arbitrary leg to an arbitrary (x, y, z) position
   - Moving the center of the robot to an arbitrary (x, y, z) position (the 6 legs staying on the floor)
   - Walking in a straight line (arbitrary direction)
   - Rotating without moving the center of the robot

**Advanced behaviors:**

- Any combination of rotation and straight line walking is possible. This is called an holonomic walk

- Controlling the robot with a keyboard/mouse/whatever

- Walk parameters can be changed on the go (frequency of the steps, step amplitude, step heigh, default robot position, etc)

- Odometry. Being able to track the (x, y, theta) position of the robot while it's walking (of course the starting position is considered known)

- Go to (x, y, theta). The robot can reach any (x, y, theta) goal position by walking there.

- Legs move smoothly (do better then the constant spee interpolation). Keywords: acceleration, bang-bang, min-jerk, feed-forward.

- The robot can change its direction on the go

- Untethered robot. You can use the battery and the low level control board

  

**A few fun ideas:**
- Attaching a pen to a leg and draw something with it
- Making the spider dance to the rythm of an arbitrary (i.e. not predetermined) song
