# Specifying pose in SDFormat

Prerequisites: [Pose frame semantics tutorial]()

A fundamental tool for robot modeling is the ability to concisely and
intuitively express the relative position and orientation of model components
in 3-D.

Throughout the documentation, the suffix `_A` is used to indicate
a kinematic quantity that is expressed w.r.t (with respect to) frame `A`. In addition,
the suffix `_BA` is used to indicate a kinematic quantity that changes from being
expressed w.r.t frame `A` to being expressed w.r.t frame `B`. For example, the
pose of frame `A` expressed w.r.t frame `B` is given by the transform `X_BA` &in; SE(3).
An alternative description of `X_BA` alludes to `X` being a linear operator
that transforms a frame that is coincident to frame `B` to being coincident to
frame`A`. In this sense,`X_BA` is said to be "the transform from B to A". Note
that both descriptions are valid interpretations of the same mathematical
object.

The SDFormat specification has the `<pose>` element which accepts 6 numbers
in total to represent a coordinate transform `X_PC` from parent frame `P`
to child frame `C`:

    <pose>x y z roll pitch yaw</pose>

The elements `x y z` specify the position vector (in meters), and the elements
`roll pitch yaw` are Euler angles (in radians) that specify the orientation, which can be
computed by an extrinsic X-Y-Z rotation as shown by the following equation
for the rotation matrix `R_PC` and figure:

<script src='https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.5/MathJax.js?config=TeX-MML-AM_CHTML' async></script>

$$
    R_{PC}
    =
    \begin{bmatrix}
      \cos(yaw) & -\sin(yaw) & 0 \\\
      \sin(yaw) &  \cos(yaw) & 0 \\\
             0  &         0  & 1
    \end{bmatrix}
    *
    \begin{bmatrix}
       \cos(pitch) & 0 & \sin(pitch) \\\
                0  & 1 &          0  \\\
      -\sin(pitch) & 0 & \cos(pitch)
    \end{bmatrix}
    *
    \begin{bmatrix}
      1 &         0  &          0  \\\
      0 & \cos(roll) & -\sin(roll) \\\
      0 & \sin(roll) &  \cos(roll)
    \end{bmatrix}
$$

[[file:roll-pitch-yaw.svg|200px]]

The SDFormat `<pose/>` element is similar to the
[`<origin/>` element from URDF](http://wiki.ros.org/urdf/XML/joint#Elements),
which both share the same definition of roll, pitch, and yaw angles:

    <!-- URDF -->
    <origin xyz="x y z" rpy="roll pitch yaw" />


## See Also

For a command-line utility to convert between roll-pitch-yaw angles,
quaternions, and rotation matrices, please see the
[quaternion\_from\_euler](https://github.com/ignitionrobotics/ign-math/blob/ign-math4/examples/quaternion_from_euler.cc)
and [quaternion\_to\_euler](https://github.com/ignitionrobotics/ign-math/blob/ign-math4/examples/quaternion_to_euler.cc)
example programs in ignition math.

Software implementations for converting between this Euler angle convention and
quaternions can be found in
[ignition::math::Quaternion](https://github.com/ignitionrobotics/ign-math/blob/ignition-math4_4.0.0/include/ignition/math/Quaternion.hh#L308-L398)
C++ class, and the [urdf::Rotation](https://github.com/ros/urdfdom_headers/blob/1.0.3/urdf_model/include/urdf_model/pose.h#L103-L155) C++ class.

Some other implementations:

*   [Drake](https://drake.mit.edu/): `drake::math::RollPitchYaw` ([C++](https://drake.mit.edu/doxygen_cxx/classdrake_1_1math_1_1_roll_pitch_yaw.html#details), [Python](https://drake.mit.edu/pydrake/pydrake.math.html#pydrake.math.RollPitchYaw))
*   [MuJoCo](http://www.mujoco.org/): See `eulerseq` in the
[`compiler` XML reference](http://www.mujoco.org/book/XMLreference.html#compiler).
