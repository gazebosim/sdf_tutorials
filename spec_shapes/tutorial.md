# Adding shapes to a model with collisions and visuals in SDFormat

This tutorial explains how to add geometric shapes to a model that describe the
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
](/tutorials?tut=pose_frame_semantics&ver=1.4) tutorials.

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
A tutorial that covers how to specify the color of a visual can be found [here](/tutorials?tut=spec_materials).
