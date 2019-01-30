# Specifying pose in SDFormat

A fundamental tool for robot modeling is the ability to concisely and
intuitively express relative position and orientation of model components
in 3-D.

The SDFormat specification has the `<pose/>` element which accepts 3 numbers
to represent a position vector `[x y z]`, followed by 3 numbers to express the
orientation as roll-pitch-yaw angles in radians.

    <pose>x y z roll pitch yaw</pose>

The roll angle corresponds to rotations about the X axis, the pitch angle
to rotations about the Y axis, and the yaw angle to rotations about the Z axis.
These angles are a form of Euler angles and are more concise than
quaternions and rotation matrices, which makes them preferable for a
human-readable text format like SDFormat.
For a command-line utility to convert between these different representations
of orientation, please see the `quaternion_from_euler` and `quaternion_to_euler`
example programs in
[ignition math pull request 297](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/297/examples-converting-between-euler-angles/diff).

## Details of roll-pitch-yaw formulation

There are many different conventions for expressing orientation with Euler
angles as a sequence of 3 angular rotations about specified axes.
One must specify the axes in a specific order and clarify whether the rotations
are relative to the world frame axes (extrinsic) or the body frame axes
(intrinsic).
The convention used by the SDFormat specification and implemented in the
[ignition::math::Quaternion](https://bitbucket.org/ignitionrobotics/ign-math/src/ignition-math4_4.0.0/include/ignition/math/Quaternion.hh#Quaternion.hh-308:398)
class is equivalent to the convention used by the
[drake::math::RollPitchYaw](https://github.com/RobotLocomotion/drake/blob/246b2c038/math/roll_pitch_yaw.h#L19-L31)
class.
This convention is an extrinsic X-Y-Z rotation by roll, pitch, and yaw angles
(r, p, y), which is equivalent to the rotation sequence expressed by the
multiplication of the following rotation matrices (duplicated from
[drake::math::RollPitchYaw](https://github.com/RobotLocomotion/drake/blob/246b2c038/math/roll_pitch_yaw.h#L19-L31)).

           ⎡cos(y) -sin(y)  0⎤   ⎡ cos(p)  0  sin(p)⎤   ⎡1      0        0 ⎤
    R_AD = ⎢sin(y)  cos(y)  0⎥ * ⎢     0   1      0 ⎥ * ⎢0  cos(r)  -sin(r)⎥
           ⎣    0       0   1⎦   ⎣-sin(p)  0  cos(p)⎦   ⎣0  sin(r)   cos(r)⎦
         =       R_AB          *        R_BC          *        R_CD
