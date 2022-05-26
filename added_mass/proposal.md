<script src='https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.5/MathJax.js?config=TeX-MML-AM_CHTML' async></script>

# Added Mass Proposal

* **Authors**:
Louise Poubel `<louise@openrobotics.org>`,
Andrew Hamilton `<hamilton@mbari.org>`,
Michael Anderson `<anderson@mbari.org>`,
* **Status**: Draft
* **SDFormat Version**: 1.9
* **`libsdformat` Version**: 12.X

## Introduction

This proposal suggests the addition of elements under the `//link/inertial` element
to support the simulation of [added mass](https://en.wikipedia.org/wiki/Added_mass)
in SDFormat 1.9.

Current implementations of added mass, such as that of
[gz::sim::systems::Hydrodynamics](https://gazebosim.org/api/gazebo/6.9/classignition_1_1gazebo_1_1systems_1_1Hydrodynamics.html#details)
apply forces as separate external forces after the physics step, adding to simulation instability.

Native support of added mass through the SDF specification makes it possible for physics
engines to handle these terms together with other inertial terms during the physics update
step.

## Document summary

The proposal includes the following sections:

* [Motivation](#motivation): An explanation of the background and rationale behind the proposal
* [Proposed changes](#proposed-changes): Terms to be added to the SDFormat specification and C++ API

## Motivation

Newton's second law describes how forces affect a body's motion. It can be written as:

$$ M * \ddot{X} = \sum{F} $$

where \\(\sum{F}\\) is the sum of all forces applied to the body, \\(M\\) is the body's
spatial mass matrix, and \\(\ddot{X}\\) is the resulting acceleration.

The mass matrix can be broken down into [1]:

$$
    M
    =
    m
    =
    \begin{bmatrix}
      m           & 0           & 0           & 0           &  m z\_{CoM} & -m y\_{CoM} \\\
      0           & m           & 0           & -m z\_{CoM} & 0           &  m x\_{CoM} \\\
      0           & 0           & m           &  m y\_{CoM} & -m x\_{CoM} & 0           \\\
      0           & -m z\_{CoM} &  m y\_{CoM} & i\_{xx}     & i\_{xy}     & i\_{xz}     \\\
       m z\_{CoM} & 0           & -m x\_{CoM} & i\_{xy}     & i\_{yy}     & i\_{yz}     \\\
      -m y\_{CoM} &  mx\_{CoM}  & 0           & i\_{xz}     & i\_{yz}     & i\_{zz}
    \end{bmatrix}
$$

where

* \\(m\\): Body's mass
* \\(i\_{xx}\\): Principal moment of inertia about the X axis
* \\(i\_{yy}\\): Principal moment of inertia about the Y axis
* \\(i\_{zz}\\): Principal moment of inertia about the Z axis
* \\(i\_{xy}\\): Product moment of inertia about the X and Y axes
* \\(i\_{xz}\\): Product moment of inertia about the X and Z axes
* \\(i\_{yz}\\): Product moment of inertia about the Y and Z axes
* \\(x\_{CoM}\\) Center of mass's X coordinate
* \\(y\_{CoM}\\) Center of mass's Y coordinate
* \\(z\_{CoM}\\) Center of mass's Z coordinate

and

$$
    X
    =
    \begin{bmatrix}
      x \\\
      y \\\
      z \\\
      p \\\
      q \\\
      r
    \end{bmatrix}
$$

where

* \\(x\\): Position in X axis
* \\(y\\): Position in Y axis
* \\(z\\): Position in Z axis
* \\(p\\): Rotation about X axis
* \\(q\\): Rotation about Y axis
* \\(q\\): Rotation about Z axis

> Note that the p-q-r notation is used for rotation, for this is common in maritime literature.
  They're equivalent to roll-pitch-yaw.

When the density of the fluid surrounding a moving body isn't negligible compared to the density of
the body, the forces required to dislocate that fluid are relevant. That's called "added mass"
and can be included by adding to \\(m\\), e.g. \\(M = m + n\\), where \\(n\\) is a symmetric matrix
defined in the link frame:

$$
    n
    =
    \begin{bmatrix}
      n\_{11} & n\_{12} & n\_{13} & n\_{14} & n\_{15} & n\_{16} \\\
      n\_{12} & n\_{22} & n\_{23} & n\_{24} & n\_{25} & n\_{26} \\\
      n\_{13} & n\_{23} & n\_{33} & n\_{34} & n\_{35} & n\_{36} \\\
      n\_{14} & n\_{24} & n\_{34} & n\_{44} & n\_{45} & n\_{46} \\\
      n\_{15} & n\_{25} & n\_{35} & n\_{45} & n\_{55} & n\_{56} \\\
      n\_{16} & n\_{26} & n\_{36} & n\_{46} & n\_{56} & n\_{66}
    \end{bmatrix}
$$

where

* \\(n\_{11}\\): added mass force along the X axis due to linear acceleration in the X direction
* \\(n\_{12}\\): added mass force along the X axis due to linear acceleration in the Y direction
* \\(n\_{13}\\): added mass force along the X axis due to linear acceleration in the Z direction
* \\(n\_{14}\\): added mass force along the X axis due to angular acceleration about the X direction
* \\(n\_{15}\\): added mass force along the X axis due to angular acceleration about the X direction
* \\(n\_{16}\\): added mass force along the X axis due to angular acceleration about the X direction
* \\(n\_{21} == n\_{12}\\): added mass force along the Y axis due to linear acceleration in the X direction
* \\(n\_{22}\\): added mass force along the Y axis due to linear acceleration in the Y direction
* \\(n\_{23}\\): added mass force along the Y axis due to linear acceleration in the Z direction
* \\(n\_{24}\\): added mass force along the Y axis due to angular acceleration about the X direction
* \\(n\_{25}\\): added mass force along the Y axis due to angular acceleration about the X direction
* \\(n\_{26}\\): added mass force along the Y axis due to angular acceleration about the X direction
* \\(n\_{31} == n\_{13}\\): added mass force along the Z axis due to linear acceleration in the X direction
* \\(n\_{32} == n\_{23}\\): added mass force along the Z axis due to linear acceleration in the Y direction
* \\(n\_{33}\\): added mass force along the Z axis due to linear acceleration in the Z direction
* \\(n\_{34}\\): added mass force along the Z axis due to angular acceleration about the X direction
* \\(n\_{35}\\): added mass force along the Z axis due to angular acceleration about the X direction
* \\(n\_{36}\\): added mass force along the Z axis due to angular acceleration about the X direction
* \\(n\_{41} == n\_{14}\\): added mass torque about the X axis due to linear acceleration in the X direction
* \\(n\_{42} == n\_{24}\\): added mass torque about the X axis due to linear acceleration in the Y direction
* \\(n\_{43} == n\_{34}\\): added mass torque about the X axis due to linear acceleration in the Z direction
* \\(n\_{44}\\): added mass torque about the X axis due to angular acceleration about the X direction
* \\(n\_{45}\\): added mass torque about the X axis due to angular acceleration about the X direction
* \\(n\_{46}\\): added mass torque about the X axis due to angular acceleration about the X direction
* \\(n\_{51} == n\_{15}\\): added mass torque about the Y axis due to linear acceleration in the X direction
* \\(n\_{52} == n\_{25}\\): added mass torque about the Y axis due to linear acceleration in the Y direction
* \\(n\_{53} == n\_{35}\\): added mass torque about the Y axis due to linear acceleration in the Z direction
* \\(n\_{54} == n\_{45}\\): added mass torque about the Y axis due to angular acceleration about the X direction
* \\(n\_{55}\\): added mass torque about the Y axis due to angular acceleration about the X direction
* \\(n\_{56}\\): added mass torque about the Y axis due to angular acceleration about the X direction
* \\(n\_{61} == n\_{16}\\): added mass torque about the Z axis due to linear acceleration in the X direction
* \\(n\_{62} == n\_{26}\\): added mass torque about the Z axis due to linear acceleration in the Y direction
* \\(n\_{63} == n\_{36}\\): added mass torque about the Z axis due to linear acceleration in the Z direction
* \\(n\_{64} == n\_{46}\\): added mass torque about the Z axis due to angular acceleration about the X direction
* \\(n\_{65} ==n\_{56}\\): added mass torque about the Z axis due to angular acceleration about the X direction
* \\(n\_{66}\\): added mass torque about the Z axis due to angular acceleration about the X direction

This matrix is composed of 21 unique values, which are usually determined using boundary
element methods to evaluate the flow due to body acceleration. It can have off-diagonal
elements (\\(n\_{ij}, i \neq j \\)) that result when a body's motion accelerates fluid in
a different direction.

## Proposed changes

The [`//link/inertial`](http://sdformat.org/spec?ver=1.9&elem=link#link_inertial) term
in SDF currenly accepts elements to describe the m matrix (mass and moments of inertia),
as well as a center of mass pose. This proposal suggests exposing each of the 21 elements
for the mass matrix in addition to that. All added mass elements are optional and default
to zero if unset. This preserves the behaviour for links that don't have those terms.

### XML spec

A new `<added_mass>` element will be added under `//link/inertial/`. It will contain each of the
21 matrix elements started by `n` and followed by its index.

```xml
<inertial>
  <mass>10</mass>
  <pose>1 1 1 0 0 0</pose>
  <inertia>
    <ixx>0.16666</ixx>
    <ixy>0</ixy>
    <ixz>0</ixz>
    <iyy>0.16666</iyy>
    <iyz>0</iyz>
    <izz>0.16666</izz>
  </inertia>
  <added_mass>
    <n11>1</n11>
    <n12>0</n12>
    <n13>0</n13>
    <n14>0</n14>
    <n15>0</n15>
    <n16>0</n16>
    <n22>1</n22>
    <n23>0</n23>
    <n24>0</n24>
    <n25>0</n25>
    <n26>0</n26>
    <n33>1</n33>
    <n34>0</n34>
    <n35>0</n35>
    <n36>0</n36>
    <n44>0.1</n44>
    <n45>0</n45>
    <n46>0</n46>
    <n55>0.1</n55>
    <n56>0</n56>
    <n66>0.1</n66>
  </added_mass>
</inertial>
```

### C++ API

The `//link/inertial` element maps to a
[gz::math::Inertiald](https://gazebosim.org/api/math/6.10/classignition_1_1math_1_1Inertial.html)
object. To support added mass, the following member functions will be added to the
`Inertial` class:

```cpp
/// \brief Constructs an inertial object from the mass matrix for a body
/// B, about its center of mass Bcm, and expressed in a frame that we’ll
/// call the "inertial" frame Bi, i.e. the frame in which the components
/// of the mass matrix are specified (see this class’s documentation for
/// details). The pose object specifies the pose X_FBi of the inertial
/// frame Bi in the frame F of this inertial object
/// (see class’s documentation). The added mass matrix is also expressed
/// in frame Bi.
/// \param[in] _massMatrix Mass and inertia matrix.
/// \param[in] _pose Pose of center of mass reference frame.
public: Inertial(const MassMatrix3<T> &_massMatrix,
                 const Pose3<T> &_pose,
                 const Matrix6<T> &_addedMass)

/// \brief Set the added mass matrix.
///
/// \param[in] _m New Matrix6 object, which must be a symmetric matrix.
/// \return True if the Matrix6 is valid.
public: bool SetAddedMass(const Matrix6<T> &_m)

/// \brief Get the added mass matrix.
/// \return The added mass matrix.
public: const Matrix6<T> &AddedMass() const

/// \brief Spatial mass matrix, which is the resulting 6x6 matrix including
/// added mass.
/// \return The spatial mass matrix.
public: const Matrix6<T> &SpatialMatrix() const
```

In order to support this API, a new `gz::math::Matrix6<T>` class will be created.
It will mirror functionality from the existing
[Matrix3](https://gazebosim.org/api/math/6.10/classignition_1_1math_1_1Matrix3.html)
and
[Matrix4](https://gazebosim.org/api/math/6.10/classignition_1_1math_1_1Matrix4.html)
classes,
and contain the minimal API needed by `Inertial`.


* [1] Fossen, T. I., Guidance and Control of Ocean Vehicles, John Wiley &
Sons Ltd (1994).
