# Specifying inertial properties in SDFormat

This documentation describes how SDFormat models the inertial properties of
articulated multibody systems. Each rigid body is represented as a `<link>`
with an `<inertial>` specifying the mass, position of its center of mass,
and its central inertia properties.

A link may have zero or one `<inertial>` element. If any inertial properties
are not specified, default values will be used instead.
The full specification for `<inertial>` can be found
[here](http://sdformat.org/spec?ver=1.4&elem=link#link_inertial).

## The `<mass>` tag

The mass of a link in units of kilograms can be specified with a
floating-point scalar value in the `<mass>` element.
The default value is `1.0` if unspecified.

## The `<pose>` tag

Just as SDFormat links, joints, and models have their own coordinate frames that
can be specified using the `<pose>` tag (see documentation for
[specifying model kinematics](/tutorials?tut=spec_model_kinematics) and
[specifying pose](/tutorials?tut=specify_pose)), SDFormat inertials have their own
coordinate frame as well. This pose specifies the relative position and orientation
of the inertial frame relative to the link frame.
The link's center of mass is located at the origin of the inertial frame.
The orientation of the inertial frame is used to interpret the moment of inertia
values described in the `<inertia>` element.

If unspecified, the inertial pose is an identity pose and the inertial frame
is identical to the link frame.

## The `<inertia>` tags

The link's moments of inertia `<ixx>`, `<iyy>`, `<izz>` and products of inertia
`<ixy>`, `<ixz>`, `<iyz>` are expressedabout the link's center of mass for the unit vectors
fixed to the X-Y-Z axes of the inertial frame is specified by the diagonal values
ixx, iyy, izz and 

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

  <element name="inertia" required="0">
    <description>
      This link's moments of inertia ixx, iyy, izz and products of inertia
      ixy, ixz, iyz about Co (the link's center of mass) for the unit vectors
      Ĉx, Ĉy, Ĉᴢ fixed in the center-of-mass-frame C.
      Note: the orientation of Ĉx, Ĉy, Ĉᴢ relative to L̂x, L̂y, L̂ᴢ is specified
      by the `pose` tag.
      To avoid compatibility issues associated with the negative sign
      convention for product of inertia, align Ĉx, Ĉy, Ĉᴢ with principal
      inertia directions so that all the products of inertia are zero.
      For more information about this sign convention, see the following
      MathWorks documentation for working with CAD tools:
      https://www.mathworks.com/help/releases/R2021b/physmod/sm/ug/specify-custom-inertia.html#mw_b043ec69-835b-4ca9-8769-af2e6f1b190c
    </description>
    <element name="ixx" type="double" default="1.0" required="1">
      <description>
        The link's moment of inertia about Co (the link's center of mass) for Ĉx.
      </description>
    </element>
    <element name="ixy" type="double" default="0.0" required="1">
      <description>
        The link's product of inertia about Co (the link's center of mass) for
        Ĉx and Ĉy, where the product of inertia convention -m x y  (not +m x y)
        is used. If Ĉx or Ĉy is a principal inertia direction, ixy = 0.
      </description>
    </element>
    <element name="ixz" type="double" default="0.0" required="1">
      <description>
        The link's product of inertia about Co (the link's center of mass) for
        Ĉx and Ĉz, where the product of inertia convention -m x z  (not +m x z)
        is used. If Ĉx or Ĉz is a principal inertia direction, ixz = 0.
      </description>
    </element>
    <element name="iyy" type="double" default="1.0" required="1">
      <description>
        The link's moment of inertia about Co (the link's center of mass) for Ĉy.
      </description>
    </element>
    <element name="iyz" type="double" default="0.0" required="1">
      <description>
        The link's product of inertia about Co (the link's center of mass) for
        Ĉy and Ĉz, where the product of inertia convention -m y z  (not +m y z)
        is used. If Ĉy or Ĉz is a principal inertia direction, iyz = 0.
      </description>
    </element>
    <element name="izz" type="double" default="1.0" required="1">
      <description>
        The link's moment of inertia about Co (the link's center of mass) for Ĉz.
      </description>
    </element>
  </element> <!-- End Inertia -->
</element> <!-- End Inertial -->

This documentation explains how to add geometric shapes to a model that describe the
physical and visual characteristics of each link in a model.
These characteristics are specified
using `<visual>` and `<collision>` tags in SDFormat.
Each `<visual>` and `<collision>` must contain one `<geometry>` tag, which
specifies the shape of the object.
A `<visual>` element may contain a `<material>` tag that can specify the
visual appearance of the shape, such as color and texture,
while a `<collision>` element may contain a `<surface>` tag that can specify
physical properties of the surface related to friction and contact.

A link can have zero or more `<visual>` or `<collision>` elements
placed in the parent link frame using a `<pose>` element.
This allows a link to define complex geometries by composition of
individual shapes with distinct visual and physical properties.
The full specifications can be found
[here](http://sdformat.org/spec?ver=1.4&elem=visual) for `<visual>`
and [here](http://sdformat.org/spec?ver=1.4&elem=collision) for `<collision>`.

## The `<geometry>` tag

The following shapes are supported by the `<geometry>` tag in SDFormat:

* `<box>`
* `<cylinder>` (aligned with Z-axis)
* `<sphere>`
* `<plane>`
* `<mesh>`
* `<heightmap>`
* `<image>`

The full specification for `<geometry>` can be found
[here](http://sdformat.org/spec?ver=1.4&elem=geometry).

For `box`, `cylinder`, and `sphere` shapes, the geometric center is used as
the attachment point to its parent.
Examples of each shape type are given below for which
the extents are `-0.5 -0.5 -0.5` and `0.5 0.5 0.5` in the local frame,
ie. that have the unit cube as a bounding box.

```xml
<geometry>
  <sphere>
    <radius>0.5</radius>
  </sphere>
</geometry>
```

```xml
<geometry>
  <cylinder>
    <length>1</length>
    <radius>0.5</radius>
  </cylinder>
</geometry>
```

```xml
<geometry>
  <box>
    <size>1 1 1</size>
  </box>
</geometry>
```

A link with matching sphere shapes in a collision and visual can be expressed as:

```xml
<link name="link">
  <collision name="collision">
    <geometry>
      <sphere>
        <radius>0.5</radius>
      </sphere>
    </geometry>
  </collision>
  <visual name="visual">
    <geometry>
      <sphere>
        <radius>0.5</radius>
      </sphere>
    </geometry>
  </visual>
</link>
```

The geometry for collisions and visuals in a link is not required to match.
This can be used, for example, to reduce computation time of collision detection
algorithms by setting simpler shapes in the `<collision>` element.

## Placing visuals and collisions using the `<pose>` tag

Similar to models, links, and joints, visuals and collisions have their own
coordinate frames that can be offset using the `<pose>` tag. By default, the
parent frame for this pose element is the parent link. However, a different
frame of reference can be set using the `frame` attribute of the `<pose>` tag.
For more information about the `<pose>` tag, see the [Specifying pose in
SDFormat](/tutorials?tut=specify_pose&ver=1.4) and [Pose frame semantics
](/tutorials?tut=pose_frame_semantics&cat=specification&) documentation.

## Composition of Shapes

Multiple visual elements can be used in a single link.
In the following example, a cylinder and a sphere are combined to create a more
complex shape.


```xml
<link name="link1">
  <pose>0 0 0.5 0 0 0</pose>
  <visual name="vis1">
    <pose>0 0 0 0 0 0</pose>
    <geometry>
      <cylinder>
        <radius>1</radius>
        <length>1</length>
      </cylinder>
    </geometry>
  </visual>
  <visual name="vis2">
    <pose>0 0 0.5 0 0 0</pose>
    <geometry>
      <sphere>
        <radius>1</radius>
      </sphere>
    </geometry>
  </visual>
</link>
```
The resulting shape is shown in the following image

[[file:files/cylinder_sphere_combo.png|256px]]

Note that when multiple `<visual>` tags are present inside a link, the `name`
attribute must be unique. The same holds for the `name` attribute of
`<collision>` tags.

## The `<material>` tag

A `<visual>` tag may also contain a `<material>` tag which is used to set the
color and texture properties of a visual.
The full specification for `<material>` can be found
[here](http://sdformat.org/spec?ver=1.4&elem=material).
Documentation on specifying the color of a visual can be found [here](/tutorials?tut=spec_materials).
