# Specifying pose in SDFormat

A fundamental tool for robot modeling is the ability to concisely and
intuitively express relative position and orientation of model components
in 3-D.

The SDFormat specification has the `<pose/>` element which accepts 6 numbers
in total:

    <pose>x y z roll pitch yaw</pose>

The elements `x y z` define a position vector (in meters), and the elements
`roll, pitch, yaw` are Euler angles (in radians) that specify the orientation,
as shown in the following figure:

[[file:roll-pitch-yaw.svg|200px]]

These angles are a form of Euler angles and are more concise than
quaternions and rotation matrices, which makes them preferable for a
human-readable text format like SDFormat.
See Footnote [1] for a software utitlity for converting between these
different representations of orientation.

There are many different conventions for expressing orientation with Euler
angles as a sequence of 3 angular rotations about specified axes.
One must specify the axes in a specific order and clarify whether the rotations
are relative to the world frame axes (extrinsic) or the body frame axes
(intrinsic).
The convention used by the SDFormat specification is an extrinsic X-Y-Z rotation
by roll, pitch, and yaw angles (r, p, y), which is equivalent to the rotation
sequence expressed by the multiplication of the following rotation matrices
(duplicated from
[drake::math::RollPitchYaw](https://github.com/RobotLocomotion/drake/blob/246b2c038/math/roll_pitch_yaw.h#L19-L31),
see Footnote [2] for more information about software implementations).

<script src='https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.5/MathJax.js?config=TeX-MML-AM_CHTML' async></script>

$$
    R_{AD}
    =
    \begin{bmatrix}
      \cos(y) & -\sin(y) & 0 \\\
      \sin(y) &  \cos(y) & 0 \\\
           0  &       0  & 1
    \end{bmatrix}
    *
    \begin{bmatrix}
       \cos(p) & 0 & \sin(p) \\\
            0  & 1 &      0  \\\
      -\sin(p) & 0 & \cos(p)
    \end{bmatrix}
    *
    \begin{bmatrix}
      1 &      0  &       0  \\\
      0 & \cos(r) & -\sin(r) \\\
      0 & \sin(r) &  \cos(r)
    \end{bmatrix}
$$

$$ R\_{AD} = R\_{AB} * R\_{BC} * R\_{CD} $$

## Footnotes

[1] For a command-line utility to convert between roll/pitch/yaw angles,
    quaternions, and rotation matrices , please see the `quaternion_from_euler`
    and `quaternion_to_euler` example programs in
    [ignition math pull request 297](https://bitbucket.org/ignitionrobotics/ign-math/pull-requests/297/examples-converting-between-euler-angles/diff).

[2] Software implementations for converting between this Euler angle convention
    and quaternions can be found in the
    [drake::math::RollPitchYaw](https://github.com/RobotLocomotion/drake/blob/246b2c038/math/roll_pitch_yaw.h#L19-L31)
    C++ class and the
    [ignition::math::Quaternion](https://bitbucket.org/ignitionrobotics/ign-math/src/ignition-math4_4.0.0/include/ignition/math/Quaternion.hh#Quaternion.hh-308:398)
    C++ class.
