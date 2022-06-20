<script src='https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.5/MathJax.js?config=TeX-MML-AM_CHTML' async></script>

# Hydrodynamic Added Mass Proposal

* **Authors**:
Louise Poubel `<louise@openrobotics.org>`,
Andrew Hamilton `<hamilton@mbari.org>`,
Michael Anderson `<anderson@mbari.org>`
* **Status**: Draft
* **SDFormat Version**: 1.9
* **`libsdformat` Version**: 12.X

## Introduction

This proposal suggests the addition of elements under the `//link/inertial` element
to support the inclusion of [hydrodynamic added mass] (https://en.wikipedia.org/wiki/Added_mass)
effects in SDFormat 1.9.

Current implementations, such as
[gz::sim::systems::Hydrodynamics](https://gazebosim.org/api/gazebo/6.9/classignition_1_1gazebo_1_1systems_1_1Hydrodynamics.html#details),
include forces due to added mass effects by estimating these forces from the last time-steps
acceleration value and applying these estimates as separate external forces, reducing simulation accuracy and stability.

Native support of added mass through the SDF specification makes it possible for physics
engines to handle these terms together with other inertial terms during the physics update
step.

## Document summary

The proposal includes the following sections:

* [Motivation](#motivation): An explanation of the background and rationale behind the proposal
* [Proposed changes](#proposed-changes): Terms to be added to the SDFormat specification and C++ API

## Motivation

Newton's second law describes how forces affect a body's motion in an inertial frame. It can be written as:

$$ (\bf{M} + \bf{\mu})   \ddot{\bf x} = \sum{F({\bf x}, t)} $$

where \\(M\\) is the body mass inertia matrix, \\(\mu\\) is the hydrodynamic added mass matrix,
\\(\sum{F}\\) is the sum of all forces applied to the body, and \\(\ddot{\bf x}\\) is the resulting acceleration, with:

$$
    {\bf x}^T
    =
    \begin{bmatrix}
      x           & y           & z           & p           &  q         & r
    \end{bmatrix}
$$

where,

* \\(x\\) : Position in X axis
* \\(y\\) : Position in Y axis
* \\(z\\) : Position in Z axis
* \\(p\\) : Rotation about X axis
* \\(q\\) : Rotation about Y axis
* \\(r\\) : Rotation about Z axis
> Note that the p-q-r notation is used for rotation, for this is common in maritime literature and cooresponds to roll-pitch-yaw.

> Note also that DART stores the state vector with the rotational modes first ${\bf x}^T = [p & q & r & x & y & z]$, and re-orients the interia matrix accordingly, for this document the more usual ordering with translational modes first in the state vector is used.

&nbsp;

&nbsp;

The body mass matrix is a result of the mass and mass-distribution of the body and is defined as [1]:

$$
    \bf M
    =
    \begin{bmatrix}
      m          & 0          & 0          & 0          &  m z_{CoM} & -m y_{CoM} \\\
      0          & m          & 0          & -m z_{CoM} & 0          &  m x_{CoM} \\\
      0          & 0          & m          &  m y_{CoM} & -m x_{CoM} & 0          \\\
      0          & -m z_{CoM} &  m y_{CoM} & I_{xx}     & I_{xy}     & I_{xz}     \\\
       m z_{CoM} & 0          & -m x_{CoM} & I_{yx}     & I_{yy}     & I_{yz}     \\\
      -m y_{CoM} &  mx_{CoM}  & 0          & I_{zx}     & I_{zy}     & I_{zz}
    \end{bmatrix}
$$

where
* \\(m\\) : Body's mass
* \\(I_{xx}\\) : Principal mass moment of inertia about the X axis
* \\(I_{yy}\\) : Principal mass moment of inertia about the Y axis
* \\(I_{zz}\\) : Principal mass moment of inertia about the Z axis
* \\(I_{xy} = I_{yx}\\) : Product mass moment of inertia about the X and Y axes, and vice-versa
* \\(I_{xz} = I_{zx}\\) : Product mass moment of inertia about the X and Z axes, and vice-versa
* \\(I_{yz} = I_{zy}\\) : Product mass moment of inertia about the Y and Z axes, and vice-versa
* \\(x_{CoM}\\) : Center of mass X coordinate
* \\(y_{CoM}\\) : Center of mass Y coordinate
* \\(z_{CoM}\\) : Center of mass Z coordinate

\\(M\\) is sufficient to represent the body's inertia when the density of the surrounding fluid is much less than the density of the body, and this matrix is formed by sdf descriptions of \\(m\\), \\((x_{CoM},y_{CoM},z_{CoM})\\), and the moment of inertial matrix \\(\bf I\\), and so no access is given to the individiual components of \\(\bf M\\).

When the density of the surrounding fluid is not negligible compared to the density of the body, the added mass matrix \\(\bf{\mu}\\) must be included for accurate simulations.  This situation commonly arises for submerged and floating bodies which have average densities comparable to the density of water, or very lightweight objects such as ballons in air.  In this case, \\(\bf{\mu}\\) is symmetric and contains 21 unique values in the most general case. The off-diagonal terms result physically from the situation in which acceleration of the body in one direction results in an acceleration of the surrounding fluid in a different direction.  [2]

$$
    \bf{\mu}
    =
    \begin{bmatrix}
      xx & xy & xz & xp & xq & xr \\\
      xy & yy & yz & yp & yq & yr \\\
      xz & yz & zz & zp & zq & zr \\\
      xp & yp & zp & pp & pq & pr \\\
      xq & yq & zq & pq & qq & qr \\\
      xr & yr & zr & pr & qr & rr
    \end{bmatrix}
$$

where

* \\(xx\\) : added mass in the X direction due to linear acceleration in the X direction
* \\(xy\\) : added mass in the X direction due to linear acceleration in the Y direction, and vice-versa
* \\(xz\\) : added mass in the X direction due to linear acceleration in the Z direction, and vice-versa
* \\(xp\\) : added mass in the X axis due to angular acceleration about the X direction, and vice-versa
* \\(xq\\) : added mass in the X axis due to angular acceleration about the X direction, and vice-versa
* \\(xr\\) : added mass orce along the X axis due to angular acceleration about the X direction, and vice-versa
* \\(yy\\) : added mass in the Y axis due to linear acceleration in the Y direction
* \\(yz\\) : added mass in the Y axis due to linear acceleration in the Z direction, and vice-versa
* \\(yp\\) : added mass in the Y axis due to angular acceleration about the X direction, and vice-versa
* \\(yq\\) : added mass in the Y axis due to angular acceleration about the X direction, and vice-versa
* \\(yr\\) : added mass in the Y axis due to angular acceleration about the X direction, and vice-versa
* \\(zz\\) : added mass in the Z axis due to linear acceleration in the Z direction
* \\(zp\\) : added mass in the Z axis due to angular acceleration about the X direction, and vice-versa
* \\(zq\\) : added mass in the Z axis due to angular acceleration about the X direction, and vice-versa
* \\(zr\\) : added mass in the Z axis due to angular acceleration about the X direction, and vice-versa
* \\(pp\\) : added mass moment about the X axis due to angular acceleration about the X direction
* \\(pq\\) : added mass moment about the X axis due to angular acceleration about the X direction, and vice-versa
* \\(pr\\) : added mass moment about the X axis due to angular acceleration about the X direction, and vice-versa
* \\(qq\\) : added mass moment about the Y axis due to angular acceleration about the X direction
* \\(qr\\) : added mass moment about the Y axis due to angular acceleration about the X direction, and vice-versa
* \\(rr\\) : added mass moment about the Z axis due to angular acceleration about the Z direction

&nbsp;

&nbsp;

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
    <xx>1.0</xx>
    <xy>0.0</xy>
    <xz>0.0</xz>
    <xp>0.0</xp>
    <xq>0.0</xq>
    <xr>0.0</xr>
    <yy>1.0</yy>
    <yz>0.0</yz>
    <yp>0.0</yp>
    <yq>0.0</yq>
    <yr>0.0</yr>
    <zz>1.0</zz>
    <zp>0.0</zp>
    <zq>0.0</zq>
    <zr>0.0</zr>
    <pp>0.1</pp>
    <pq>0.0</pq>
    <pr>0.0</pr>
    <qq>0.1</qq>
    <qr>0.0</qr>
    <rr>0.1</rr>
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
* [2] Newman, J. N., Marine Hydrodynamics, MIT Press (1977)
