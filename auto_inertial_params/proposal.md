<script src='https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.5/MathJax.js?config=TeX-MML-AM_CHTML' async></script>

# Automatically Compute Inertia Tensor Proposal

* **Authors**:
Jasmeet Singh `<jasmeet0915@gmail.com>`, Addisu Taddese `<addisu@openrobotics.org>`, Dharini Dutia `<dharinidutia@openrobotics.org>`
* **Status**: Draft
* **SDFormat Version**: 1.11
* **`libSDFormat` Version**: 14.X 

## Introduction

This proposal suggests adding new attributes and elements to support the automatic calculation of [Moments of Inertia and Products of Inertia](https://en.wikipedia.org/wiki/List_of_moments_of_inertia) for a link in SDFormat 1.11. It also proposes adding support for parsing these elements and attributes in libsdformat14.

Setting physically plausible values for inertial parameters is crucial for an accurate simulation. However, these parameters are often complex to comprehend and visualize, and users may tend to enter wrong values leading to incorrect simulation. Therefore, native support for calculating inertial parameters through SDFormat specification would enable accurate simulations in simulators that use SDFormat.

## Document summary

The proposal includes the following sections:

* [Motivation](#motivation): A short explanation to provide context regarding the problem statement and the need for this feature
* [User Perspective](#user-perspective): Describes the current usage and the proposed usage by describing the terms to be added in the SDFormat specification
* [Proposed Implementation](#proposed-implementation): Detailed explanation of the proposed implementation and modifications to be done in the C++ API of requried libraries like libsdformat. 

## Syntax

The proposal uses [XPath syntax](https://www.w3schools.com/xml/xpath_syntax.asp) to describe elements and attributes concisely. For example, `<model>` tags are referred to as `//model` using XPath. XPath is even more concise for referring to nested tags and attributes. In the following example, `<link>` elements inside `<model>` tags are referenced as `//model/link` and  model `name` attributes as `//model/@name`:

```xml
<model name="model_name">
  <link/>
</model>
```

## Motivation

Currently, there are 2 major workflows used by the users to obtain the correct inertial parameters of their models:

 * Using CAD software like [Fusion360](https://www.autodesk.in/products/fusion-360/overview?term=1-YEAR&tab=subscription) or [Solidworks](https://www.solidworks.com/). Many users design their robot models using such CAD software which provide plugins that automatically generate the URDF/SDF for their model. Such plugins handle the calculation of the inertial parameters. For example, Fusion360 provides the [Fusion2URDF](https://github.com/syuntoku14/fusion2urdf) plugin which automatically generates a URDF with all the inertial parameters.

 * Another way is to use 3rd-party Mesh Processing Software like [Meshlab](https://www.meshlab.net/). Such softwares take the mesh file as an input and provide the inertial parameters as an output which can then be copied and pasted into the URDF/SDF file. This is also the method that was suggested in official [Classic Gazebo docs](https://classic.gazebosim.org/tutorials?cat=build_robot&tut=inertia).

Both of these ways create a dependency on external software and might be complicated for beginners. In case the user doesn't provide any inertial values, a default Mass Matrix is used with `mass = 1.0` and `Diagonal Elements = (1, 1, 1)` which might not be best suited for all kinds of models. Native support for automatic inertia calculations directly into `libsdformat` would work as a better alternative to using the default values and facilitate the effortless generation of accurate simulations.

## User Perspective

To specify the `<inertial>` element of a `<link>` in SDFormat, the user needs to add the `<mass>`, `<pose>`, and `<inertia>` tags. Here `<mass>` is the mass of the link and `<pose>` is the pose of the Centre of Mass with respect to the link frame. The `<inertia>` tag, on the other hand, needs to enclose the following 6 tags:

 * `<ixx>` (Moment of Inertia)

 * `<iyy>` (Moment of Inertia)

 * `<izz>` (Moment of Inertia)

 * `<ixy>` (Product of Inertia)
 
 * `<ixz>` (Product of Inertia)
 
 * `<izy>` (Product of Inertia)

This proposal suggests the addition of the following elements and attributes in `SDFormat Spec`: 

 1.  `//inertial/@auto` attribute that would tell `libsdformat` to calculate the Inertial values (Mass, Mass Matrix & Inertial Pose) automatically for the respective link. 

 2. `//collision/density` element for specifying the density of the collision geometry. This density value would be used to calculate the inertial parameters of the respective collision geometry. Adding this as part of the `<collision>` tag would allow a user to simulate links with different material types for different collisions. By default, the value of density would be set equal to that of water which is 1000 kg/m^3. A `//link/inertial/density` element would also be added in the spec to allow users to specify the density values on a link level instead of specifying the same values for each collision.

 3. `//collision/auto_inertia_params` element would be added that can be used to provide some parameters or options for a custom inertia calculator. Similar to the `density` element above, a link-level `//link/inertial/auto_inertia_params` element would also be added. This would allow the user to provide inertia calculator params for the whole link while retainging the flexibility to specify different parameters for each collision. Custom elements and attributes using a namespace prefix can be used to provide the user-defined parameters for a custom calculator. More about this can be found in [this](http://sdformat.org/tutorials?tut=custom_elements_attributes_proposal&cat=pose_semantics_docs&#specifying-custom-elements-and-attributes) proposal.

The example snippet below shows how the above proposed elements would be used in a SDFormat `<link>`:

```xml
<link name="robot_link">
  <inertial auto="true">
    <auto_inertia_params>
      <gz:voxel_size>0.01</gz:voxel_size>
    </auto_inertia_params>
  <collision name="collision">
    <density>*some_float_value*</density>
    <geometry>
      .
      .
    </geometry>
  </collision>
  <visual name="visual">
    <geometry>
      .
      .
    </geometry>
    <material>
      .
      .
    </material>
  </visual>
</link>
```

## Proposed Implementation
> Note: In the section below, the term `primitive geometries` is used to collectively describe Box, Capusle, Cylinder, Ellipsoid and Sphere geometries.

### Some Architectural Considerations

Below are some key architectural considerations for the implementation of this feature:

 *  The parsing of the proposed SDFormat elements and the Moment of Inertia calculations for primitive geometries (Box, Cylinder, Sphere, Ellipsoid and Capsule) can be implemented as an integral part of `libsdformat`. This would help enable all simulators that rely on SDFormat to utilize this feature.

 * In case of 3D meshes being used as geometries, a modular callback-based architecture can be followed where the user can integrate their custom Moments of Inertia calculator.  A [Voxelization-based](#inertia-matrix-calculation-with-voxelization-for-3d-mesh) and an integration-based numerical method were explored for computing the inertial properties (mass, mass matrix and center of mass) of 3D meshes.

 * For links where `<inertial>` tag is not set, the inertial calculations would be omitted if `<static>` is set to true. [By default](https://github.com/gazebosim/sdformat/blob/4530dba5e83b5ee7868156d3040e7554f93b19a6/src/Link.cc#L164) it is set as \\(I\_{xx}=I\_{yy}=I\_{zz}=1\\) and \\(I\_{xy}=I\_{yz}=I\_{xz}=0\\).

 * The collision geometry of the link would used for all the inertial calculations. In case of multiple collisions, the inertial pose of each collision would be set in the link frame (if not already done) and then inertials would be aggregated using the `+` operator from the `gz::math::Inertial` class. If, however, no collisions are provided, an error would be thrown. 

### Implemetation for Primitive Geometries
A `std::optional<gz::math::Inertiald> CalculateInertial(double density)` function would be added to the classes of all the Geometry Types which would be supported by this feature (Box, Capsule, Cylinder, Ellipsoid, Sphere and Mesh). For all the types except Mesh, existing [`MassMatrix()`](https://github.com/gazebosim/gz-math/blob/2dd5ab6f9e0b7b3220723c5fa5f4f763746c0851/include/gz/math/detail/Capsule.hh#L100) functions from the `gz::math` class of the respective geometry would be used for their inertial calculations.
An additional data member `double density` would be added to `sdf::Collision` along with getter and setter functinons to get/set the `density` value of each collision. The density value would be sent to `CalculateInertial()` functions as a param. 
In case of the primitive geometries, the density value would be used to create a `gz::math::Material` object before calling the `MassMatrix()` function of the respective geometry. Below is the implmentation of the `Box::CalculateInertial()` function as an example:

```C++
/// \brief Calculate and return the Mass Matrix values for the Box
/// \param[in] _density Density of the box in kg/m^3
/// \return A std::optional with gz::math::Inertiald object or std::nullopt
public: std::optional<gz::math::Inertiald>
        CalculateInertial(double _density);

/////////////////////////////////////////////////
std::optional<gz::math::Inertiald> Box::CalculateInertial(double _density)
{
  gz::math::Material material = gz::math::Material(_density);
  this->dataPtr->box.SetMaterial(material);

  auto boxMassMatrix = this->dataPtr->box.MassMatrix();

  if (!boxMassMatrix)
  {
    return std::nullopt;
  }
  else
  {
    gz::math::Inertiald boxInertial;
    boxInertial.SetMassMatrix(boxMassMatrix.value());
    return std::make_optional(boxInertial);
  }
}
```

### Callback-based API for Mesh/Custom Inertia Calculator

A `CustomInertiaCalcProperties` class with the following members would be added:

```C++
class CustomInertiaCalcProperties::Implementation
{
  /// \brief Density of the mesh. 1000 kg/m^3 by default
  public: double density{1000.0};

  /// \brief Optional SDF mesh object. Default is std::nullopt
  public: std::optional<sdf::Mesh> mesh{std::nullopt};

  /// \brief SDF element pointer to <auto_inertia_params> tag.
  /// This can be used to access custom params for the
  /// Inertia Caluclator
  public: sdf::ElementPtr inertiaCalculatorParams{nullptr};
};
```

This class would act as an interface between `libsdformat` and the custom calculator to bridge the data like density, properties of the mesh and user-defined calculator params given through the SDF. A signature for the Custom Calculator function is provided through an alias:

```C++
using CustomInertiaCalculator = std::function<std::optional<gz::math::Inertiald>(sdf::Errors &, const sdf::CustomInertiaCalcProperties &)>;
```

Functions to get and register a custom inertia calculator is provided through the `sdf::ParserConfig`. Using all these additions, the `Mesh::CalculateInertial()` function would be implmented as follows:

```C++
  /// \brief Calculate and return the Mass Matrix values for the Mesh
  /// \param[in] density Density of the mesh in kg/m^3
  /// \param[in] _autoInertiaParams ElementPtr to
  /// <auto_inertia_params> element
  /// \param[in] _config Parser Configuration
  /// \param[out] _errors A vector of Errors object. Each object
  /// would contain an error code and an error message.
  /// \return A std::optional with gz::math::Inertiald object or std::nullopt
  public: std::optional<gz::math::Inertiald> CalculateInertial(double _density, const sdf::ElementPtr _autoInertiaParams, const ParserConfig &_config, sdf::Errors &_errors);

  //////////////////////////////////////////////////
  std::optional<gz::math::Inertiald> Mesh::CalculateInertial(double _density,
      const sdf::ElementPtr _autoInertiaParams, const ParserConfig &_config,
      sdf::Errors &_errors)
  {
    if (this->dataPtr->filePath.empty())
    {
      _errors.push_back({
        sdf::ErrorCode::WARNING,
        "File Path for the mesh was empty. Could not calculate inertia"});
      return std::nullopt;
    }

    const auto &customCalculator = _config.CustomInertiaCalc();

    sdf::CustomInertiaCalcProperties calcInterface = CustomInertiaCalcProperties(
      _density, *this, _autoInertiaParams);

    return customCalculator(_errors, calcInterface);
  }
```

### Configuring the `CalculateInertial()` function behavior

`CalculateInertial()` functions are also added to the `sdf::Root`, `sdf::World`, `sdf::Model`, `sdf::Link`, `sdf::Collision` and then `sdf::Geometry` classes. Calling the `CalculateInertial()` function of any class, in turn recursively calls the `CalculateInertial()` for each element below it down the chain. For eg, generally a user might call the `Root::CalculateInertial()` which would in turn call `World::CalculateInertials()` for all the worlds which in turn calls the `Model::CaluclateInertials()` for all the models in the world and so on until the final `CalculateInertial()` for the respective geometry type is called where the chain ends.

The `sdf::ParserConfig` object is sent down the `CalculateInertial()` chain as a function parameter and is used to get the configuration information like the registered Custom Inertia Calculator. An `enum class` would be added to the `sdf::ParserConfig` with values that would allow the user to configure the behavior of the `CalculateInertial()` function:

```C++
/// \enum ConfigureCalculateInertial
/// \brief Configuration options of how CalculateInertial() function
/// would be used
enum class ConfigureCalculateInertial
{
  /// \brief If this value is used, CalculateInertial() won't be 
  /// called from inside the Root::Load() function
  SKIP_CALCULATION_IN_LOAD,

  /// \brief If this values is used, CalculateInertial() would be 
  /// called and the computed inertial values would be saved 
  SAVE_CALCULATION
};
```

Setting values from the above enum for the `sdf::ParserConfig` object, the user can configure the `CalculateInertial()` functions. For eg: if the configuration is set to `SKIP_CALCULATION_IN_LOAD` (which would be the default configuration), then the `Root::CalculateInertial()` won't be called from within the `Root::Load()`. The user would need to call the `Root:CalculateInertial()` separately after the load is complete. This is also recommended as the inertia calculation uses the `PoseGraph` to resolve the inertial poses for different collisions if they are not in the link frame. 

## Proposed Mesh Inertia Calculation Methods

## Voxelization-based method

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

#### Advantages of the Voxelization Approach
 * Voxelization is a generalized approach that would work for all kinds of meshes (convex and non-convex meshes).
 * It can be made configurable by allowing the user to set an appropriate `voxel_size` for the process. A smaller `voxel_size` would be comparitively computationally heavy but would provide closer to real world value for moment of inertia.
 * Voxelization is more intuitive way of moment of inertia calculations as compared to other integral methods.

### Integration-based numerical method
It uses Gauss’s Theorem and Greene’s Theorem of integration to convert volume integrals to surface integrals (Gauss’s Theorem) and then surface integrals to line integrals(Greene’s Theorem).[1](https://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf) 
This method works for triangle meshes which are simple water-tight polyhedrons. Currently, the origin of the mesh being used needs to be set at the geometric centre to obtain the correct value.
Since this method uses the vertex data for calculations, a high vertex count would be required for near-ideal values. For eg, in case of a cylinder, it was observed that with a vertex count of 4096, the inertial values obtained were withtin a 0.005 tolerance of the ideal values.