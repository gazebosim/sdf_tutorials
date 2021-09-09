# Specifying pose: Proposal for a better pose

* **Authors**:
Eric Cousineau `<eric.cousineau@tri.global>`,
* **Status**: Draft
* **SDFormat Version**: 1.9
* **`libsdformat` Version**: 12

## Introduction

**Purpose statement**

This proposal suggests that the `//pose` should have an option to specify the
rotation representation.

Currently, the text within `//pose` consists of a 6-tuple representing
`{xyz} {rpy}`, where `{xyz}` is a 3-tuple representing translation (in meters)
and `{rpy}` is a 3-tuple representing rotation (in radians).

When writing models, there are two drawbacks to this representation:
(1) specifying rotation in radians adds overhead when hand-crafting models
because the author must specify common degree values (e.g. 30, 45, 60, 90
degrees) in radians, and authors may use different precisions in different
circumstances, and, at a lower priority, (2) it is
sometimes hard to visually separate translation from rotation.

This proposal intends to resolve on point (1) by adding `//pose/@degrees`
where degrees can be selected, and *could* address point (2) by structuring
the element differently (see below).

## Document summary

The proposal includes the following sections:

* Syntax: before and after
* Motivation: background and rationale
* Proposed changes
* Examples: using proposed syntax
* Survey: other specifications

## Syntax

This proposal suggests that the following modifications be made to the `//pose` element:

### A. Add "degrees" as a Boolean Attribute

```xml
<pose>{xyz}  {rpy_radians}</pose>
<pose degrees="true">{xyz}  {rpy_degrees}</pose>
```

*and*:

### B. Add "rotation_format" Attribute

```xml
<pose>{xyz}  {rpy_radians}</pose>
<pose rotation_format="rpy_radians">{xyz}  {rpy_radians}</pose>
<pose rotation_format="rpy_degrees">{xyz}  {rpy_degrees}</pose>

<!-- Not yet confirmed -->
<pose rotation_format="q_wxyz">{xyz}  {q_wxyz}</pose>
```

## Motivation

In models, one may come across values that look like this in `//pose`:

```xml
<pose>0.25 1.0 2.1 -2 0 0</pose>
<pose>0 0 0 0 1.5708 0</pose>
<pose relative_to='base'>1 2 3  0 0 0</pose>
<pose>0 0 0 0 0 1.5707963267948966</pose>
<pose>0 0 0 1.57079632679 0 0</pose>
<pose>0.29175 0 0.0335 0 0.261799364 0</pose>
<pose>9.56106065 0.917 -0.0365 0 0 3.14159</pose>
<pose>1.549414224 0.387353556 0.5109999999999999 0.0 -0.17453292519943295 1.570796</pose>
<pose>0.0049 0.092 0.027 1.57 -0.40 0.0</pose>

<!-- xacro -->
<pose>0 0 0.084 0 0 ${pi / 2}</pose>
<pose>0 ${-body_width/4 - body_space_width/4} ${body_bottom_box_height + body_space_height/2} 0 0</pose>  <!-- Note: Missing zero -->
```

This shows both of the aforementioned issues:

1. With several of the poses, it's a tad hard to see the translation vs. rotation. In fact, with one of the longer expressions, a zero was accidentally
excluded due to how long the expression is overall.

2. Notice the varying degrees of precision used to repesent 90 degrees
(1.57-ish radians). Also, note how that for Xacro uses, `${pi / 2}` is used
*solely* to convert from degrees to radians, rather than more relevant things
like computing incremental changes in orientation.

For units of radians, comments *could* be used to help (e.g.
`<!-- This means X in degrees -->`), but ideally, the specification handles
this in an active and self-documenting way.

For both options, `libsdformat` and SDFormat tutorials should
encourage additional whitespace, e.g. to separate translation and rotation, use
3 spaces (instead of 1) as a delimiter between values if they fit on one line,
or use a newline (possibly with hanging indents) if they do not fit on one line.

To help inform this proposal, the authors conducted a brief survey. See the
[Survey](#survey) section below for more information.

## Proposed changes

#### 1.A. `//pose/@degrees` 

This boolean attribute will determine whether the specified angles are in
degrees (when `true`) or radians (when `false`). It will not be used if the
quaternions are used to specify the rotation.

#### 1.B. `//pose/@rotation_format`

This attribute will specify the rotation representation. It is less verbose
than the alternatives considered; however, it will still make the visual
separation between translation and rotation harder to distinguish.

This would be a bit "more" backwards-compatible in terms of looking more
similar than the alternatives cosidered, and general "backwards-compatibility"
will be much easier to implement (in `libsdformat` and other implementations).

For separating the tuples, it may be possible to achieve this by making a
suggested style to insert more whitespace (newlines or additional spaces), and
reflect this style when outputting XML (as mentioned above).

**Other Alternatives Considered**

*Use `//pose/translation` and `//pose/rotation`*

The value of `//pose` could now be specified as `//pose/translation` and
`//pose/rotation`, and the representation for the rotation will be specified
using `//pose/rotation/@format`. While this makes it easier to distinguish
between translation and rotation visually, it would add too much complexity to
the parser especially if backward compatibility is desired.

*Use `@orientation_format` instead of `@rotation_format`*

More verbosity, a bit harder to type.

*As Attributes*

While SDFormat could use attributes for these values like URDF does, it would
go against the convention used for other elements (e.g.
`//joint/axis/xyz`, `//inertia/ixx,...`).

Additionally, allowing the rotation format to be represented implicitly by
mutally exclusive attributes (e.g. `rpy`, `rpy_degrees`, `q_wxyz`) may
complicate parsing to an extent.

*Use `//pose/rot` instead of `//pose/rotation`*

While `rot` is shorter, it would be nicer to be explicit. (This can be
reconsidered.)

*Use `//pose/orientation` instead of `//pose/rotation`*

It's unclear which one may be better. In ROS, `rotation` is used for a
transform, while `orientation` is used for a pose. However, they both appear
equivalent.

#### 1.1  Values for `//pose/@rotation_format` and `//pose/@degrees` 

The permutations of `@rotation_format` and `@degrees` that are
permitted:

* `@rotation_format="rpy"`, `@degrees="true"` - A 3-tuple representing
  Roll-Pitch-Yaw in degrees, which maps to a rotation as specified here.
    * This should be used when the rotation should generally be human-readable.
* `@rotation_format="rpy"`, `@degrees="false"` - Same as above, but with radians as the units for each
angle. This is provided for legacy purposes and ease of conversion.
    * It is not suggested to use this for a text-storage format.
    * Same precision as suggested below for quaternions: Use 17 digits of
    precision, and consider separating each value on a new line.
* `@rotation_format="q_wxyz"` - Quaternion as a 4-tuple, represented as  `(w, x, y, z)`, where `w`
is the real component. This should generally be used when the rotation should be
machine-generated (e.g. calibration artifacts).
    * It is encouraged to use 17 digits of precision when possible (C++'s
    default from `std::numeric_limits<double>::max_digits10`).
        * In Python, this can be done with using the format specifier
        `{value:.17g}` (for a 64-bit float stored in `value`).
    * Consider separating long values on new lines.
    * It is encouraged to prefer upper half-sphere quaternions (`w >= 0`).

Examples:

```xml
<pose degrees="true">{xyz}   90 45 180</pose>

<pose rotation_format="q_wxyz">
    {xyz}

    0.27059805007309851
    -0.27059805007309845
    0.65328148243818818
    0.65328148243818829
</pose>

<pose rotation_format="q_wxyz">{xyz}   0.27059805007309851 -0.27059805007309845 0.65328148243818818 0.65328148243818829</pose> <!-- Same as above, but on one line -->

<pose rotation_format="rpy" degrees="false"> <!-- This is not recommended. -->
    {xyz}

    1.5707963267948966
    0.78539816339744828
    3.1415926535897931
</pose>

<pose>{xyz}   1.5707963267948966 0.78539816339744828 3.1415926535897931</pose> <!-- Same as above, but with attributes removed since they are the default.-->
```

<!--
Python code:

$ python3 -c 'import numpy as m; print(m.__version__)'
1.13.3
$ cat ./share/doc/drake/VERSION.TXT
20200915064519 6b419530fa5d0e25e37610c8642b4852f60e434d

    import numpy as np
    from pydrake.math import RollPitchYaw
    from pydrake.common.eigen_geometry import Quaternion

    rpy_degrees = [90, 45, 180]

    rpy_radians = np.deg2rad(rpy_degrees)
    for x in rpy_radians: print(f"{x:.17g}")

    # q_wxyz
    q_wxyz = RollPitchYaw(rpy_radians).ToQuaternion().wxyz()
    for x in q_wxyz: print(f"{x:.17g}")
-->

**Alternatives Considered**

*Use `@rotation_type` instead of `@rotation_format`*

In higher dimentions, the term rotation type is used to distinguish between
simple, double, and other types of rotation. Even though is only one type of
rotation in 3 dimensions, using `format` would be less confusing without adding
too much verbosity.

*Use `@rotation_representation` instead of `@rotation_format`*

While "representation" may be a better word than "format", it would be nice to be
less verbose while still being concise (e.g. avoiding abbreviations).

*Use `//pose/{rotation_format}` instead of
`//pose/@rotation_format="rotation_format"]`*

Specifying something like `//pose/rpy` or `//pose/q_wxyz` may
encounter some of the parsing complication for mutually exclusive tags, as
mentioned above.

*Use `@rotation_format="quaternion"` instead of `@rotation_format="q_wxyz"`*

In general, it can be confusing when interfacing different libraries that use
different orderings for quaternions and those ordering are not readily stated
in the API (or even the documentation). Instead, the author recommends
explicitly enumerating this order in a relatively unambiguous way that is shown
directly in the specification.

*Add `@rotation_format="q_xyzw`, `@rotation_format="euler_intrinsic_rpy"`, `@rotation_format="matrix"`,
`@rotation_format="axis_angle"`, `@rotation_format="axang3`, etc.*

The author feels that too many representations and permutations may make it
really hard (and annoying) to support an already complex specification.

#### 1.1.1 Re-describe API Implications, potential sources of numerical error

The `ignition::math::Pose3d` stores its rotation as
`ignition::math::Quaternion`.

Therefore, when storing quaternions, users should be aware of what numeric
changes happen to their data (e.g. normalization), so they should generally
know where changes in precision may happen.

When converting to roll-pitch-yaw coordiantes, we should try to specify the
*exact* math being done. (e.g. a cross-reference to `Quaternion::Euler()`
accessor and mutator, but with the algorithm actually described in
documentation).

When converting between radians and degrees, we should try to specify *exactly*
what math is done, and how much precision should be expected to be lost by
`libsdformat` during the conversion (e.g. the exact representation of `pi` used
in code, the order of operations, etc.).

#### 1.2 Conversion to SDFormat 1.9

When SDFormat files are converted from SDFormat <=1.8 to 1.9, the `//pose` tags
will be adjusted to use `//pose/@rotation_format="rpy"` and `//pose/@degrees="false"`.

The conversion command-line tool should also provide an option to use
`rpy_degrees` (`//pose/@rotation_format="rpy"` and `//pose/@degrees="true"`),
with a precision amount for round-off to degrees by values of 5
(e.g. 0, 5, ..., 45, ..., 90 degrees).

#### 1.3 Emitting SDFormat Models

The following changes are necessary when emitting SDFormat files:

- The user should be able to control the output rotation type. For backwards
  compatibility, it will be `//pose/@rotation_format="rpy"` and `//pose/@degrees="false"` by default.
- There should be an admission for "snapping to" well known values in either
  representation, within a given angular tolerance (degrees). This can help
  convert exisiting models to more readable units, and possibly with better
  intended accuracy.

## Examples

Here are some additional simple examples of different poses with equivalent
representations (all printed up to 17 degrees of precision):

```xml
<!-- No translation, identity orientation -->
<pose rotation_format="rpy" degrees="true">0 0 0   0 0 0</pose>
<pose>0 0 0   0 0 0</pose>
<pose rotation_format="q_wxyz">0 0 0   1 0 0 0</pose>

<!-- No translation, rotate 90 degrees about the x-axis -->
<pose rotation_format="rpy" degrees="true">0 0 0   90 0 0</pose>
<pose>0 0 0   1.5707963267948966 0 0</pose>
<pose rotation_format="rpy" degrees="false">0 0 0  1.5707963267948966 0 0</pose>
<pose rotation_format="q_wxyz">
    0 0 0
    0.7071067811865475 0.7071067811865475 0 0
</pose>

<!-- No translation, a semi-arbitrary rotation -->
<pose rotation_format="rpy" degrees="true">0 0 0   10 20 30</pose>
<pose>0 0 0   0.17453292519943295 0.3490658503988659 0.52359877559829882</pose>
<pose rotation_format="q_wxyz">
    0 0 0
    0.95154852464378847 0.038134576474850149 0.18930785741200001
        0.23929833774473031
</pose>
```

## Survey

The following is a brief survey on how a few other formats specify poses /
transforms.

### ROS

ROS provided suggestions for representing rotations / orientations: <br/>
[REP 0103](https://www.ros.org/reps/rep-0103.html)

<!--
TODO(eric.cousineau): It's unclear how this did or didn't influence URDF and
SDFormat development. We should rfind out and document here.
-->

### URDF

URDF provides the attributes `//origin/@xyz` and `//origin/@rpy`, as mentioned
here:
<http://wiki.ros.org/urdf/XML/link#Elements>

Example:
```xml
<origin xyz="{xyz}" rpy="{rpy_radians}"/>
```

### MuJoCo XML

Elements tend to have their poses defined by attributes, a combination of
`@pos` for translation and then one of `@quat` (in `wxyz` order),
`@axisangle`, `@euler`, `@xyaxes`, `@zaxis`:

* <http://www.mujoco.org/book/XMLreference.html#body>
* <http://www.mujoco.org/book/XMLreference.html#compiler>
* <http://www.mujoco.org/book/modeling.html#COrientation>

Some examples for `//body`, with default `//compiler` settings
(`@angle="degree"`, `@eulerseq="xyz"`):

```xml
<body pos="{xyz}" quat="{q_wxyz}" .../>
<!-- or -->
<body pos="{xyz}" euler="{rpy_degrees}" .../>
```

### Collada

Transforms for `//node` can be dictated by any combination of
`//translate`,

See the available specification for children of `//node` in the specification
PDF for Collada 1.5:

* <https://www.khronos.org/collada/>

Some examples:

```xml
<translate>{xyz}</translate>
<rotate>{axis_xyz} {angle_deg}</rotate>
<!-- or -->
<matrix>
    {Rxx} {Rxy} {Rxz} {x}
    {Ryx} {Ryy} {Ryz} {y}
    {Rzx} {Rzy} {Rzz} {z}
    0 0 0 1
</matrix>
```

### glTF

Defined as a node under `//nodes`, and can either be composed of just `@matrix`
or `@translation` and `@rotation`:

* <https://github.com/KhronosGroup/glTF/blob/0f74b714/specification/2.0/README.md#transformations>

Some examples:

```json
"rotation": [{qx}, {qy}, {qz}, {qw}],
"translation": [{x}, {y}, {z}],
... or ...
"matrix": [...]
```

### VRML

Defined using `//Transform` elements with optional `@translation` and
`@rotation` attributes:

* <https://www.web3d.org/documents/specifications/19776-1/V3.3/Part01/EncodingOfNodes.html#Transform>
* <https://www.web3d.org/documents/specifications/19776-1/V3.3/Part01/EncodingOfFields.html#SFRotation>

An example:

```xml
<Transform translation="{xyz}" rotation="{axis_xyz} {angle_radians}">
  ...
</Transform>
```

### SKEL

Similar to SDFormat but stored as `//transformation`:

* <https://dartsim.github.io/skel_file_format.html>

Example:

```xml
<transformation>{xyz} {rpy_radians}</transformation>
```

*Note*: I (Eric) am assuming radians for Euler angles for the SKEL format.
