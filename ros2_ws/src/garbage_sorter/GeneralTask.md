## 4. Smart Robotic Sorter

Concept: A 6-DOF robot arm with an overhead or wrist camera identifies objects by color and size on a table, then picks them up and places them into matching bins.

Core challenge: Tight integration of perception, planning, and control: translating a pixel location into a real-world grasp pose the arm can actually reach.

Key ROS 2 concepts: MoveIt2 for motion planning and kinematics; camera calibration (mapping image points to the world frame); image segmentation with OpenCV; ros2_control for the simulated joints.


## My added ideas

The baseline above is the C. These are the extras on top, in the order we build them.

### 1. Garbage sorting instead of colour sorting
- Four waste classes instead of plain colours: plastic, paper, metal and glass.
- Objects built from simple shapes (cylinders and boxes), not meshes.
- Classified by colour, size **and shape** (aspect ratio, solidity, area), not only colour and size.
- One pair is made hard to tell apart on purpose, for example a clear plastic bottle and a glass bottle of the same size.
- Result: a confusion matrix, and an explanation of where it gets confused and why.

### 2. Depth camera to find the height of objects (idea from the TA)
- A depth camera (Gazebo `rgbd_camera`) instead of a plain colour camera: a colour image and a depth image from the same view.
- The camera looks straight down, so: object height = distance to the table − distance to the pixel.
- What the height is used for:
  - Telling real objects from flat marks on the table or belt (stains, labels, shadows). Anything about 0 cm high is not an object.
  - Knowing how far down the suction nozzle has to go, since a can is 11 cm tall and paper is 1 cm.
  - An extra feature for classification, next to colour, size and shape.
- Result: put flat decoy stains on the table, coloured like the real waste. Compare false picks with colour only against colour + depth.

### 3. Suction gripper
- A suction nozzle on the arm instead of fingers, like real waste-sorting robots.
- Simulated with a detachable joint in Gazebo: attach when the nozzle touches the object, detach over the bin.
- Makes the grasp simpler (straight down onto the centre of the object), so more time goes into 4 and 5.

### 4. Two arms on one torso (main contribution)
- Two arms on a shared base, like the ABB YuMi at CIIE.
- One URDF and one `move_group` with two planning groups, so the arms see each other and don't collide.
- Our own **allocation algorithm** that decides which arm picks which object.
  - Baseline: the nearest arm takes it.
  - Ours: balances the load, respects each arm's reach, and avoids one arm waiting on the other.
- Result: throughput with 1 arm, 2 arms naive, and 2 arms with ours. Two arms will not give 2×, and the gap is the finding.

### 5. Moving conveyor belt (bonus)
- Objects move past on a belt instead of lying still.
- The arm has to predict where the object will be when it gets there.
- Result: pick success rate against belt speed, and the speed where it breaks.

### World
- Obstacles: box, cylinder.