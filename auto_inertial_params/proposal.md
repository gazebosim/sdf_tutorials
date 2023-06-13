<script src='https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.5/MathJax.js?config=TeX-MML-AM_CHTML' async></script>

# Automatically Compute Inertia Tensor Proposal

* **Authors**:
Jasmeet Singh `<jasmeet0915@gmail.com>`, Addisu Taddese `<addisu@openrobotics.org>`, Dharini Dutia `<dharinidutia@intrinsic.ai>`
* **Status**: Draft
* **SDFormat Version**: 1.11
* **`libSDFormat` Version**: 14.X 

## Introduction

This proposal suggests adding new attributes and elements to support the automatic calculation of Moments of Inertia and Products of Inertia for a link in SDFormat 1.11. It also proposes adding support for parsing these elements and attributes in libsdformat14.

Setting physically plausible values for inertial parameters is crucial for an accurate simulation. However, these parameters are often complex to comprehend and visualize. Therefore, incorporating native support for the automatic calculation of inertial parameters of a link through the SDF specification would enable simulators that utilize SDFormat to provide an enhanced and smoother workflow.

## Document summary

The proposal includes the following sections:

* [Motivation](#motivation): A short explanation to provide context reagrding the problem statement and the need for this feature
* [User Perspective](#user-perspective): Describes the current usage and the proposed usage by describing the terms to be added in the SDFormat specification
* [Proposed Implmentation](#proposed-implementation): Detailed explanation of the proposed implmentation and modifications to be done in the C++ API of requried libraries like libsdformat. 

## Motivation

Physically plausible values for inertial parameters like mass, center of mass, moments of inertia, etc. are required for an accurate simulation. Such parameters are often difficult to visualize and a user may tend to enter wrong values for parameters This often leads to an incorrect simulation which is hard to debug.

Currently, there are 2 major workflows used by the users to obtain the correct inertial parameters of their models:

 * Using CAD softwares like [Fusion360](https://www.autodesk.in/products/fusion-360/overview?term=1-YEAR&tab=subscription) or [Solidworks](https://www.solidworks.com/). Many users design their robot models using such CAD software which usually provide plugins that automatically generates the URDF/SDF for their model. These plugins handle the calculation of the inertial parameters. For eg, Fusion360 provides the [Fusion2URDF](https://github.com/syuntoku14/fusion2urdf) plugin which automatically generates a URDF with all the inertial parameters.

 * Another way is to use 3rd-party Mesh Processing Software like [Meshlab](https://www.meshlab.net/). Such softwares take the mesh file as an input and provide the inertial parameters as an output which can then be copied and pasted into the URDF/SDF file. This is also the method that was suggested in official [Classic Gazebo docs](https://classic.gazebosim.org/tutorials?cat=build_robot&tut=inertia).

Both of these ways create a dependency on external software and might be complicated for beginners. Integrating this directly in Gz Sim would provide a smoother and user-friendly workflow.

## User Perspective

Currently to specify the `<inertial>` element of a `<link>` in SDFormat, the user needs to add the `<mass>`, `<pose>`, and `<inertia>` tags. Here `<mass>` is the mass of the link and `<pose>` is the pose of the Centre of Mass with respect to the link frame. The `<inertia>` tag, on the other hand, needs to enclose the following 6 tags:

 * `<ixx>` (Moment of Inertia)

 * `<iyy>` (Moment of Inertia)

 * `<izz>` (Moment of Inertia)

 * `<ixy>` (Product of Inertia)
 
 * `<ixz>` (Product of Inertia)
 
 * `<izy>` (Product of Inertia)

This proposal suggests the addition of an `auto` parameter for the `<inertia>` tag that would tell `libsdformat` to calculate Inertia matrix values automatically for the respective link. 

Usage example:    
```
<inertia auto=”true” />
```

Addition of a `//link/collision/material_density` tag is also suggested. This density value would be used to calculate the inertial parameters of the respective collision geometry. Adding this as part of the `<collision>` tag would allow a user to simulate links with different material types for different collisions. By default, the value of density would be set equal to that of water which is 1 kg/m^3.  

## Proposed Implementation
Below are some key architectural considerations for the implementation of this feature:

 *  The parsing of the proposed SDFormat elements and the Moment of Inertia calculations for primitive geometries(Box, Cylinder, Sphere, Ellipsoid and Capsule) can be developed as an integral part of libsdformat. This would help enable all simulators that rely on SDFormat to utilize this feature and not limit it to just Gazebo.

 * In case of 3D meshes being used as geometries, a modular architecture can be followed where the user is free to develop and use their own Moments of Inertia Calculators. The default approach for handling MOI calculations of 3D meshes for Gazebo is proposed below and would rely on [Voxelization of Meshes](#inertia-matrix-calculation-with-voxelization-for-3d-mesh).

 * For links where `<inertial>` tag is not set, the inertial calculations would be omitted if `<static>` is set to true. Currently a [default value](https://github.com/gazebosim/sdformat/blob/4530dba5e83b5ee7868156d3040e7554f93b19a6/src/Link.cc#L164) is set with \\(I\_{xx}=I\_{yy}=I\_{zz}=1\\) and \\(I\_{xy}=I\_{yz}=I\_{xz}=0\\).

 * The collision geometry of the link would used for all the inertial calculations.

Existing [`MassMatrix()`](https://github.com/gazebosim/gz-math/blob/2dd5ab6f9e0b7b3220723c5fa5f4f763746c0851/include/gz/math/detail/Capsule.hh#L100) function from the `gz::math` class of each primitive geometry would be used for their inertial calculations. 
An additional data member of type `gz::math::Material` would be added to `sdf::Collision` to set the density value of each collision. This `gz::math::Material` object would be passed to the `MassMatrix()` call for the respective geometry as specified above.
The flow of the [`sdf::Link::Load()`](https://github.com/gazebosim/sdformat/blob/4530dba5e83b5ee7868156d3040e7554f93b19a6/src/Link.cc#L170) function would be updated to check for the value of the `auto` parameter if the `<inertial>` tag is found and then accordingly call the required functions.

## Inertia Matrix Calculation with Voxelization for 3D Mesh

Voxels are the 3D equivalent of a pixel in 2D. Voxels can be arranged in ‘Voxel Grids’ which are the 3D equivalent of a structured image using pixels. 

During the voxelization of a point cloud or mesh, all the points in the 3D data are intersected with a Voxel Grid. The voxels which contain a point of the mesh are kept while others are zeroed out(discarded). This way, we are left with a voxelized mesh that closely resembles the original mesh.

Voxelization of meshes/point cloud data is widely used for mesh processing. It can be used for feature extraction, occupancy analysis, asset generation, surface simplification, etc. 

### Moment of Inertia Matrix Calculation

The Moment of Inertia Matrix of an object is a 3x3 symmetric matrix. This means for all elements in a Moment of Inertia Matrix, I:

$$ I\_{ij} = I\_{ji} $$

The diagonal elements of the matrix are denoted as \\(I\_{xx}\\), \\(I\_{yy}\\) and \\(I\_{zz}\\) and are the Moments of Inertia of the object. The remaining 6 off-diagonal elements are called the Products of Inertia and their value depends on the symmetry of the object about the axes about which the MOI Tensor is being calculated. Only 3 values out of the 6 are needed since the matrix is symmetric. 

>**Note:** If the axis about which the MOI Tensor is calculated, is taken to be the principal axis of inertia, then the products of inertia become 0 and the matrix becomes a diagonal matrix.

All these values of MOI Tensor can be calculated as follows:

$$\begin{eqnarray} 
 I\_{11} = I\_{xx} = \int (y^2 + z^2)dm \\\
 I\_{22} = I\_{yy} = \int (x^2 + z^2)dm \\\
 I\_{33} = I\_{zz} = \int (x^2 + y^2)dm \\\
\end{eqnarray}$$

Here \\(dm\\) is the mass of an infinitesimal unit of the object and \\(x, y, z\\) are the distances of that unit from axes. 

Similarly, the Products of Inertia can be calculated as:

$$\begin{eqnarray}
 I\_{12}  = I\_{xy} = \int -xydm = I\_{yx} = I\_{21} \\\
 I\_{13}  = I\_{xz} = \int -xzdm = I\_{zx} = I\_{31} \\\
 I\_{23}  = I\_{yz} = \int -yzdm = I\_{zy} = I\_{32} \\\
\end{eqnarray}$$

In this solution, the **infinitesimal element** of the object can be **represented by the each Voxel.** Instead of calculating the mass \\(dm\\) for each voxel, we will calculate the volume \\(dv\\) of each voxel using the voxel size (the simple volume formula of a cube can be used because voxels are cubes). Then mass for each element would be mass density, \\(\rho\\) multiplied by the volume, \\(dv\\). Considering the mass density to be constant and substituting in the above equations we get:

$$\begin{eqnarray}
I\_{11} = I\_{xx} = \rho\int (y^2 + z^2)dv \\\
I\_{22} = I\_{yy} = \rho\int (x^2 + z^2)dv \\\
I\_{33} = I\_{zz} = \rho\int (x^2 + y^2)dv \\\
I\_{12}  = I\_{xy} = \rho\int -xydv = I\_{yx} = I\_{21} \\\
I\_{13}  = I\_{xz} = \rho\int -xzdv = I\_{zx} = I\_{31} \\\
I\_{23}  = I\_{yz} = \rho\int -yzdv = I\_{zy} = I\_{32} \\\
\end{eqnarray}$$

### Advantages of the Voxelization Approach
 * Voxelization is a generalized approach that would work for all kinds of meshes (convex and non-convex meshes).
 * It can be made configurable by allowing the user to set an appropriate `voxel_size` for the process. A smaller `voxel_size` would be comparitively computationally heavy but would provide closer to real world value for moment of inertia.
 * Voxelization is more intuitive way of moment of inertia calculations as compared to other integral methods. 
