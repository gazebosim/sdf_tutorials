# Specifying pose: Proposal for a better pose

* **Authors**:
Eric Cousineau `<eric.cousineau@tri.global>`,
* **Status**: Draft
* **SDFormat Version**: 1.8
* **`libsdformat` Version**: 11

## Introduction

**Purpose statement**

This proposal suggests that the `//pose` should have an option to specify the
rotation representation.

Currently, the text within `//pose` consists of a 6-tuple representing
`{xyz} {rpy}`, where `{xyz}` is a 3-tuple representing translation (in meters)
and `{rpy}` is a 3-tuple representing translation (in radians).

When writing models, there are two drawbacks to this representation: (1) it is
sometimes hard to visually separate translation from rotation and (2)
specifying rotation in radians adds overhead when hand-crafting models because
the author must specify common degree values (e.g. 30, 45, 60, 90 degrees) i
radians, and authors may use different precisions in different circumstances.

This proposal intends to resolve on point (1) by structuring the pose as
`//pose/xyz` and `//pose/rotation`, and point (2) by adding
`//pose/rotation/@type` where degrees can be specified.

## Document summary

Make a bullet list of all major sections:

TBD

## Syntax

This proposal suggests that the following fixed pose representation:

```xml
<pose>{xyz} {rpy_radians}</pose>
```

should now allow the following values:

```xml
<pose>{xyz} {rpy_radians}</pose>  <!-- Old format; deprecated. -->

<pose>
    <xyz>{xyz}</xyz>
    <rotation type="rpy_degrees">{rpy_degrees}</rotation>
</pose>

<pose>
    <xyz>{xyz}</xyz>
    <rotation type="q_wxyz">{wxyz}</rotation>
</pose>

<pose>
    <xyz>{xyz}</xyz>
    <rotation type="rpy_radians">{rpy_radians}</rotation>  <!-- This is not recommended. -->
</pose>
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

While some of these items could be changed with linting (e.g. a `//pose` should
either be all on one line, with 2 or 3 spaces between translation and rotation)
or comments (e.g. `<!-- This means X in degrees -->`), a specification should
try to handle this naturally.

## Proposed changes

### 1. `//pose/xyz` and `//pose/rotation`

The value of `//pose` could now be specified as `//pose/xyz` and
`//pose/rotation`, and the representation for the rotation will be specified
using `//pose/rotation/@type`.

**Details**

* `//pose/xyz` will remain a 3-tuple of strings representing floating-point
values. If unspecified (either the tag is not specified or is empty), then
those values will default to `0 0 0`.

* `//pose/rotation` will have more structure. See the section below.

* There will be backwards compatibility for the old form of expressing
`//pose`. See section below for more details.

*As Attributes*

URDF provides the attributes `//origin/@xyz` and `//origin/@rpy`, as mentioned here:
<http://wiki.ros.org/urdf/XML/link#Elements>

While SDFormat could use attributes for these values as well, it would go against the convention used for other elements (e.g. `//joint/axis/xyz`,
`//inertia/ixx,...`).

Additionally, allowing the rotation type to be represented implicitly by
mutally exclusive attributes (e.g. `rpy`, `rpy_degrees`, `q_wxyz`) may
complicate parsing to an extent.

*Use `//pose/rot` instead of `//pose/rotation`*

While `rot` is shorter, it would be nicer to be explicit. (This can be
reconsidered.)

#### 1.1 `//pose/rotation/@type`

The values of `@type` that are permitted:

* `rpy_degrees` - A 3-tuple representing Roll-Pitch-Yaw in degrees, which maps
to a rotation as specified here.
    * This should be used when the rotation should generally be human-readable.
* `q_wxyz` - Quaternion as a 4-tuple, represented as  `(w, x, y, z)`, where `w`
is the real component. This should generally be used when the rotation should be
machine-generated (e.g. calibration artifacts).
    * It is encouraged to use 16 digits of precision when possible.
        * In Python, this can be done with f-strings using `{value:.16g}`.
    * Consider separating long values on new lines.
    * It is encouraged to prefer upper half-sphere quaternions (`w >= 0`). \
    **TODO(eric)**: Is this right? Should confirm.
* `rpy_radians` - Same as `rpy_degrees`, but with radians as the units for each
angel. This is provided merely for legacy purposes and ease of conversion.
    * It is not suggested to use this for a text-storage format.
    * Same as for quaternions: Use 16 digits of precision, consider separating
    each value on a new line.

Examples:

```xml
<pose>
    <xyz>{xyz}</xyz>
    <rotation type="rpy_degrees">90 45 180</rotation>
</pose>

<pose>
    <xyz>{xyz}</xyz>
    <rotation type="q_wxyz">
        0.2705980500730985
        -0.2705980500730985
        0.6532814824381882
        0.6532814824381883
    </rotation>
</pose>

<pose>
    <xyz>{xyz}</xyz>
    <rotation type="rpy_radians">  <!-- This is not recommended. -->
        1.570796326794897
        0.7853981633974483
        3.141592653589793
    </rotation>
</pose>
```

<!--
Python code:

$ python3 -c 'import numpy as m; print(m.__version__)'
1.13.3
$ cat ./share/doc/drake/VERSION.TXT
20200729064618 2daabfac1b81f9165d8fece7891df03bb67e8e72

    import numpy as np
    from pydrake.math import RollPitchYaw
    from pydrake.common.eigen_geometry import Quaternion

    rpy_degrees = [90, 45, 180]

    rpy_radians = np.deg2rad(rpy_degrees)
    for x in rpy_radians: print(f"{x:.16g}")

    # q_wxyz
    q_wxyz = RollPitchYaw(rpy_radians).ToQuaternion().wxyz()
    for x in q_wxyz: print(f"{x:.16g}")
-->

**Alternatives Considred**

*Use `@representation` instead of `@type`*

While "representation" may be a better word than "type", it would be nice to be
less verbose while still being concise (e.g. avoiding abbreviations).

*Use `//pose/{rotation_type}` instead of
`//pose/rotation/@type="rotation_type"]`*

Specifying something like `//pose/rpy_radians` or `//pose/rpy_degrees` may
encounter some of the parsing complication for mutually exclusive tags, as
mentioned above.

*Let `@type` have a default value (e.g. `"rpy_radians"` or `"rpy_degrees"`)*

While this would be ideal in terms of brevity, it is a bit too implicit and may
prove for confusion, especially when mixing degrees and radians (which may then
yield "dumb" scaling factors that have to be debugged).

It is true that "rpy" itself is still a bit ambiguous (e.g. which version of
Euler angles used), but the author feels that we shouldn't support too many
versions, and it may be hard to converge on succinct representations at that
(e.g. are the versions defined in the popular `transformations.py` package
really that easy to understand?).

*Use `@type="quaternion"` instead of `@type="q_wxyz"`*

In general, it can be confusing when interfacing different libraries that use
different orderings for quaternions and those ordering are not readily stated
in the API (or even the documentation). Instead, the author recommends
explicitly enumerating this order in a relatively unambiguous way that is shown
directly in the specification.

*Add `@type="q_xyzw`, `@type="euler_intrinsic_rpy"`, `@type="matrix"`,
`@type="axis_angle"`, `@type="axang3`, etc.*

The author feels that too many representations and permutations may make it
really hard (and annoying) to support an already complex specification.

#### 1.2 Deprecate old `//pose` representation

The existing usage of `//pose` will remain for SDFormat 1.8, but will be
deprecated and removed in SDFormat 1.9.

#### 1.2.1 Conversion to SDFormat 1.8

When SDFormat files are converted from SDFormat <=1.7 to 1.8, the `//pose` tags
will be adjusted to use `//pose/xyz` and `//pose/rotation[@type="rpy_radians"]`.

The conversion command-line tool should also provide an option to use
`rpy_degrees`, with a precision amount for round-off to degrees by values of 5
(e.g. 0, 5, ..., 45, ..., 90 degrees).

#### 1.2.2 Deprecate `@type="rpy_radians"` in SDFormat 1.9

This should be deprecated in SDFormat 1.9, and removed in SDFormat 1.10.

It'd be nice to constrain the rotation types to just two. Radians don't seem
that useful if you have degrees for RPY, and quaternions for machine-generated
data? There should be "one right way to do things"?

## Examples

TBD
