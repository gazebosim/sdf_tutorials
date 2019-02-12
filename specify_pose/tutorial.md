# Specifying pose in SDFormat

A fundamental tool for robot modeling is the ability to concisely and
intuitively express the relative position and orientation of model components
in 3-D.

The SDFormat specification has the `<pose/>` element which accepts 6 numbers
in total:

    <pose>x y z roll pitch yaw</pose>

The elements `x y z` specify the position vector (in meters), and the elements
`roll pitch yaw` are Euler angles (in radians) that specify the orientation, which can be
computed by an extrinsic X-Y-Z rotation as shown by the following equation and figure:

<script src='https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.5/MathJax.js?config=TeX-MML-AM_CHTML' async></script>

$$
    R_{rpy}
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

[[file:roll-pitch-yaw.svg|200px]]

## See Also

For a command-line utility to convert between roll-pitch-yaw angles,
quaternions, and rotation matrices, please see the
[quaternion_from_euler](https://bitbucket.org/ignitionrobotics/ign-math/src/ign-math4/examples/quaternion_from_euler.cc)
and [quaternion_to_euler](https://bitbucket.org/ignitionrobotics/ign-math/src/ign-math4/examples/quaternion_to_euler.cc)
example programs in ignition math.

Software implementations for converting between this Euler angle convention and
quaternions can be found in
[ignition::math::Quaternion](https://bitbucket.org/ignitionrobotics/ign-math/src/ignition-math4_4.0.0/include/ignition/math/Quaternion.hh#Quaternion.hh-308:398)
C++ class, and the [urdf::Rotation](https://github.com/ros/urdfdom_headers/blob/1.0.3/urdf_model/include/urdf_model/pose.h#L103-L155) C++ class.

Some other implementations:

*   [Drake](https://drake.mit.edu/): `drake::math::RollPitchYaw` ([C++](https://drake.mit.edu/doxygen_cxx/classdrake_1_1math_1_1_roll_pitch_yaw.html#details), [Python](https://drake.mit.edu/pydrake/pydrake.math.html#pydrake.math.RollPitchYaw))
*   [MuJoCo](http://www.mujoco.org/): See `eulerseq` in the
[`compiler` XML reference](http://www.mujoco.org/book/XMLreference.html#compiler).
