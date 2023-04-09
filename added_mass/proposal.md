<script src='https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.5/MathJax.js?config=TeX-MML-AM_CHTML' async></script>

# Fluid Added Mass Proposal

* **Authors**:
Louise Poubel `<louise@openrobotics.org>`,
Andrew Hamilton `<hamilton@mbari.org>`,
Michael Anderson `<anderson@mbari.org>`
* **Status**: Draft
* **SDFormat Version**: 1.10
* **`libSDFormat` Version**: 13.X (Gazebo Garden)

## Introduction

This proposal suggests the addition of elements under the `//link/inertial` element
to support the inclusion of [fluid added mass] (https://en.wikipedia.org/wiki/Added_mass)
effects in SDFormat 1.10 and support for parsing it in libSDFormat 13.

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
* [Proposed implementation](#proposed-implementation): Terms to be added to the SDFormat specification and C++ API

## Motivation

Newton's second law describes how forces affect a body's motion in an inertial frame. It can be written as:

$$ (\bf{M} + \bf{\mu})   \ddot{\bf x} = \sum{F({\bf x}, t)} $$

where \\(\bf{M}\\) is the body mass inertia matrix, \\(\mu\\) is the fluid added mass matrix,
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

&nbsp;

The body mass matrix is a result of the mass and mass-distribution of the body and is defined as [1]:

$$
    M
    =
    \begin{bmatrix}
      m           & 0           & 0           & 0           &  m z\_{CoM} & -m y\_{CoM} \\\
      0           & m           & 0           & -m z\_{CoM} & 0           &  m x\_{CoM} \\\
      0           & 0           & m           &  m y\_{CoM} & -m x\_{CoM} & 0           \\\
      0           & -m z\_{CoM} &  m y\_{CoM} & I\_{xx}     & I\_{xy}     & I\_{xz}     \\\
       m z\_{CoM} & 0           & -m x\_{CoM} & I\_{xy}     & I\_{yy}     & I\_{yz}     \\\
      -m y\_{CoM} &  mx\_{CoM}  & 0           & I\_{xz}     & I\_{yz}     & I\_{zz}
    \end{bmatrix}
$$

where

* \\(m\\) : Body's mass
* \\(I\_{xx}\\) : Principal mass moment of inertia about the X axis
* \\(I\_{xy}\\) : Product mass moment of inertia about the X and Y axes, and vice-versa
* \\(I\_{xz}\\) : Product mass moment of inertia about the X and Z axes, and vice-versa
* \\(I\_{yy}\\) : Principal mass moment of inertia about the Y axis
* \\(I\_{yz}\\) : Product mass moment of inertia about the Y and Z axes, and vice-versa
* \\(I\_{zz}\\) : Principal mass moment of inertia about the Z axis
* \\(x\_{CoM}\\) : Center of mass X coordinate
* \\(y\_{CoM}\\) : Center of mass Y coordinate
* \\(z\_{CoM}\\) : Center of mass Z coordinate

\\(\bf{M}\\) is sufficient to represent the body's inertia when the density of the
surrounding fluid is much less than the density of the body, and this matrix is
formed by sdf descriptions of \\(m\\), \\((x\_{CoM},y\_{CoM},z\_{CoM})\\), and the
moment of inertial matrix \\(\bf I\\), and so no access is given to the individiual
components of \\(\bf M\\).

When the density of the surrounding fluid is not negligible compared to the
density of the body, the added mass matrix \\(\bf{\mu}\\) must be included for
accurate simulations.  This situation commonly arises for submerged and floating
bodies which have average densities comparable to the density of water, or very
lightweight objects such as ballons in air.  In this case, \\(\bf{\mu}\\) is
symmetric and contains 21 unique values in the most general case. The
off-diagonal terms result physically from the situation in which acceleration of
the body in one direction results in an acceleration of the surrounding fluid in
a different direction.  [2]

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

* \\(xx\\) : added mass in the X axis due to linear acceleration in the X axis
* \\(xy\\) : added mass in the X axis due to linear acceleration in the Y axis, and vice-versa
* \\(xz\\) : added mass in the X axis due to linear acceleration in the Z axis, and vice-versa
* \\(xp\\) : added mass in the X axis due to angular acceleration about the X axis, and vice-versa
* \\(xq\\) : added mass in the X axis due to angular acceleration about the Y axis, and vice-versa
* \\(xr\\) : added mass in the X axis due to angular acceleration about the Z axis, and vice-versa
* \\(yy\\) : added mass in the Y axis due to linear acceleration in the Y axis
* \\(yz\\) : added mass in the Y axis due to linear acceleration in the Z axis, and vice-versa
* \\(yp\\) : added mass in the Y axis due to angular acceleration about the X axis, and vice-versa
* \\(yq\\) : added mass in the Y axis due to angular acceleration about the Y axis, and vice-versa
* \\(yr\\) : added mass in the Y axis due to angular acceleration about the Z axis, and vice-versa
* \\(zz\\) : added mass in the Z axis due to linear acceleration in the Z axis
* \\(zp\\) : added mass in the Z axis due to angular acceleration about the X axis, and vice-versa
* \\(zq\\) : added mass in the Z axis due to angular acceleration about the Y axis, and vice-versa
* \\(zr\\) : added mass in the Z axis due to angular acceleration about the Z axis, and vice-versa
* \\(pp\\) : added mass moment about the X axis due to angular acceleration about the X axis
* \\(pq\\) : added mass moment about the X axis due to angular acceleration about the Y axis, and vice-versa
* \\(pr\\) : added mass moment about the X axis due to angular acceleration about the Z axis, and vice-versa
* \\(qq\\) : added mass moment about the Y axis due to angular acceleration about the Y axis
* \\(qr\\) : added mass moment about the Y axis due to angular acceleration about the Z axis, and vice-versa
* \\(rr\\) : added mass moment about the Z axis due to angular acceleration about the Z axis

The unit of each matrix element matches its corresponding element on the mass matrix \\(\bf{M}\\).
That is, elements on the top-left 3x3 corner are in \\(kg\\), the bottom-right ones are in
\\(kg\cdotm^2\\), and the rest are in \\(kg\cdotm\\).

## Proposed implementation

While many physics engines may not have direct support for added mass, they often
support directly setting each element of the resulting \\(\bf{M} + \bf{\mu}\\)
matrix that's multiplied by acceleration. That's the case for
[DART](https://dartsim.github.io/dart/main/d6/d91/classdart_1_1dynamics_1_1Inertia.html#aa4661f9950d5958efa4c8609d42aedf7),
for example.

Although the matrices may be combined by the time they're fed into the physics
engine, this proposal suggests storing all the terms of the fluid added mass
matrix as they are in \\(\bf{\mu}\\) before being added to the mass matrix
\\(\bf{M}\\). This includes the representation of the matrix within SDF files,
as well as their storage in memory in C++ objects. Keeping them separate makes
sure they hold values with clear meaning that can be obtained from external
software. That also makes them easier to introspect and change independently of
other inertial terms.

### XML spec

The [`//link/inertial`](http://sdformat.org/spec?ver=1.9&elem=link#link_inertial) term
in SDF currenly accepts elements to describe the m matrix (mass and moments of inertia),
as well as a center of mass pose. This proposal suggests exposing each of the 21 elements
for the mass matrix in addition to that. All added mass elements are optional and default
to zero if unset. This preserves the behaviour for links that don't have those terms.

A new `<fluid_added_mass>` element will be added under `//link/inertial/`.
It will contain each of the 21 matrix elements, and the fluid density.

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
  <fluid_added_mass>
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
  </fluid_added_mass>
</inertial>
```

### C++ API

The `//link/inertial` element maps to a
[gz::math::Inertiald](https://gazebosim.org/api/math/6.10/classignition_1_1math_1_1Inertial.html)
object. To support added mass, the following member functions will be added to the
`Inertial` class:

```cpp
      /// \brief Constructs an inertial object from the mass matrix for a body
      /// B, about its center of mass Bcm, and expressed in a frame that we'll
      /// call the "inertial" frame Bi, i.e. the frame in which the components
      /// of the mass matrix are specified (see this class's documentation for
      /// details). The pose object specifies the pose X_FBi of the inertial
      /// frame Bi in the frame F of this inertial object
      /// (see class's documentation). The added mass matrix is expressed
      /// in the link origin frame F.
      /// \param[in] _massMatrix Mass and inertia matrix.
      /// \param[in] _pose Pose of center of mass reference frame.
      /// \param[in] _addedMass Coefficients for fluid added mass.
      public: Inertial(const MassMatrix3<T> &_massMatrix,
                       const Pose3<T> &_pose,
                       const Matrix6<T> &_addedMass)
      : massMatrix(_massMatrix), pose(_pose), addedMass(_addedMass)
      {}


      /// \brief Set the matrix representing the inertia of the fluid that is
      /// dislocated when the body moves. Added mass should be zero if the
      /// density of the surrounding fluid is negligible with respect to the
      /// body's density.
      ///
      /// \param[in] _m New Matrix6 object, which must be a symmetric matrix.
      /// \return True if the Matrix6 is symmetric.
      /// \sa SpatialMatrix
      public: bool SetFluidAddedMass(const Matrix6<T> &_m);

      /// \brief Get the fluid added mass matrix.
      /// \return The added mass matrix. It will be nullopt if the added mass
      /// was never set.
      public: const std::optional< Matrix6<T> > &FluidAddedMass() const;

      /// \brief Spatial mass matrix for body B. It does not include fluid
      /// added mass. The matrix is expressed in the object's frame F, not to
      /// be confused with the center of mass frame Bi.
      ///
      /// The matrix is arranged as follows:
      ///
      /// | m          0          0          0           m * Pz    -m * Py |
      /// | 0          m          0          -m * Pz    0           m * Px |
      /// | 0          0          m           m * Py    -m * Px    0       |
      /// | 0          -m * Pz     m * Py    Ixx        Ixy        Ixz     |
      /// |  m * Pz    0          -m * Px    Ixy        Iyy        Iyz     |
      /// | -m * Py     m* Px     0          Ixz        Iyz        Izz     |
      ///
      /// \return The body's 6x6 inertial matrix.
      /// \sa SpatialMatrix
      public: Matrix6<T> BodyMatrix() const;

      /// \brief Spatial mass matrix, which includes the body's inertia, as well
      /// as the inertia of the fluid that is dislocated when the body moves.
      /// The matrix is expressed in the object's frame F, not to be confused
      /// with the center of mass frame Bi.
      /// \return The spatial mass matrix.
      /// \sa BodyMatrix
      /// \sa FluidAddedMass
      public: Matrix6<T> SpatialMatrix() const
      {
        return this->addedMass.has_value() ?
            this->BodyMatrix() + this->addedMass.value() : this->BodyMatrix();
      }

```

These changes are ABI-breaking, because they will require adding new private members
to the `Inertial` class. Therefore, this proposal targets the upcoming Gazebo release,
Garden, which will use libSDFormat 13 and Gazebo Math 7.

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
