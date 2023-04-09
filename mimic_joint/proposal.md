# Proposal for Mimic Joint Actuation Constraint

* **Authors**:
Steve Peters`<scpeters@openrobotics.org>`,
Aditya Pande `<aditya.pande@osrfoundation.org>`
* **Status**: *Draft*
* **SDFormat Version**: *1.10+*
* **`libsdformat` Version**: *13*

## Introduction

This proposal suggests adding a new joint actuation contraint called the Mimic
Constraint that adds a linear equality constraint between the output position
of two joint axes.

Currently, the Gearbox joint type provides equivalent functionality for
three specified links along specified axis directions, if the three links are
also connected by two revolute joints with matching axis directions.
This is an issue because it requires diplication of axis definitions, and it is
nonintuitive to specify three links instead of two joints.

The Mimic constraint will simplify the definition of this constraint by
specifying the joint axes to which it applies instead of links so
that the joint axis information does not need to be duplicated.
It will be more flexible than the Gearbox joint by
allowing constraints on the output of prismatic joints and other
joints with translational outputs.

## Document summary

* *Syntax*: description of XPath syntax used in this proposal.
* *Motivation*:
* *Proposed changes*:
* *Examples*:

## Syntax

The proposal uses [XPath syntax](https://www.w3schools.com/xml/xpath_syntax.asp)
to describe elements and attributes concisely.
For example, `<model>` tags are referred to as `//model` using XPath.
XPath is even more concise for referring to nested tags and attributes.
In the following example, `<link>` elements inside `<model>` tags are
referenced as `//model/link` and  model `name` attributes as `//model/@name`:

    <model name="model_name">
      <link/>
    </model>

## Motivation

Currently, a Gearbox joint is specified by the following SDFormat parameters:

* 3 links
  * `//joint/gearbox_reference_body`
  * `//joint/parent`
  * `//joint/child`
* 2 axis directions
  * `//joint/axis/xyz`
  * `//joint/axis2/xyz`
* 1 scalar
  * `//joint/gearbox_ratio`

The relative rotation of the `child` link with respect to the
`gearbox_reference_body` about the `//joint/axis/xyz` direction is defined as
`angle_C`, and similarly for the `parent` link with respect to the
`gearbox_reference_body` about the `//joint/axis2/xyz` direction as `angle_P`.
The Gearbox joint creates a proportional equality constraint:

`angle_C = -gearbox_ration * angle_P`

The Gearbox joint is typically used in conjunction with a pair of
revolute joints with identical axes, so the axis definitions must be duplicated
between the revolute joints and the gearbox joint.
The Gearbox joint only constrains the rotational motion of revolute joints,
so it cannot model constraints involving translational motion,
such as a rack and pinion mechanism.

The proposed Mimic constraint will simplify the specification of joint output
constraints by eliminating the need for redundant information.
This simplification is achieved by specifying the pair of joint axes to which
a Mimic constraint applies instead of specifying a group of three links,
a pair of axis directions, and a separate pair of revolute joints with
redundant axis information.
The Mimic constraint will also be more flexible than the gearbox joints
expanding the number of joint types that are supported, such as prismatic
joints and other joints with translational outputs.

## Proposed changes

This section defines the Mimic constraint mathematically and then explains
how to specify it in an SDFormat `//joint` element.

### Definition of Mimic Constraint

A Mimic Constraint encodes a linear equality constraint on the position of
two joint axes. One joint axis is labelled as the *leader* and the other as the
*follower*. The `multiplier`, `offset`, and `reference` parameters determine
the linear relationship according to the equation below.

`follower_position = multiplier * (leader_position - reference) + offset`

The `multiplier` parameter represents the ratio between changes in the
*follower* joint position relative to changes in the *leader* joint position.

`multiplier = (follower_position - offset) / (leader_position - reference)`

Note that the `multiplier` and `offset` parameters match the parameters of the
[URDF mimic](https://wiki.ros.org/urdf/XML/joint#Elements)
tag if the `reference` parameter is `0`.

### New SDFormat tags: `//axis/mimic` and `//axis2/mimic`

To specify the Mimic constraint between two joint axes, an optional `//mimic`
tag is added to the `//joint/axis` and `//joint/axis2` elements.
The `//mimic` tag should be added to the *follower* joint axis, and
the *leader* joint axis is specified using the `//mimic/@joint` and
`//mimic/@axis` attributes. The `multiplier`, `offset`, and `reference`
parameters are specified as child elements of `//mimic`.

An alternative was to add a new joint type called a Mimic joint,
but since URDF already supports the `//joint/mimic` tag (see
[URDF documentation](https://wiki.ros.org/urdf/XML/joint) and
[ros/robot\_state\_publisher#1](https://github.com/ros/robot_state_publisher/issues/1))
it would be more consistent to add a new sdf tag called ``<mimic>`` inside
the ``//joint/axis/`` tag.

### Details of `//mimic`

When added to a `//joint/axis` or `//joint/axis2` element, the `//mimic` tag
causes that joint axis to be treated as the *follower* in a Mimic constraint.
The `//mimic` tag must have a `@joint` attribute that specifies the name of a
joint accessible from the current scope and may optionally specify an `@axis`
attribute as well. The `@axis` attribute has a default value of `axis`, and
its only valid values are `axis` and `axis2`. Together, the `@joint` and
`@axis` attributes specify the *leader* joint axis for the mimic constraint.
Note that the `@axis` attribute may only take a value of `axis2` if `@joint`
refers to a multi-axis joint.
The `multiplier`, `offset`, and `reference` parameters are specified in the
`//mimic/multiplier`, `/mimic/offset`, and `/mimic/reference` child elements,
respectively.

~~~
<mimic joint="leader_joint_name" axis="axis">
  <multiplier>1.0</multiplier>
  <offset>0.0</offset>
  <reference>0.0</reference>
</mimic>
~~~

### Deprecation of `gearbox` joint type

Since the `mimic` joint provides equivalent functionality to the gearbox joint
but with more flexibility and less repeated information, the `gearbox` joint
type and the associated `//joint/gearbox_reference_body` and
`//joint/gearbox_ratio` elements are deprecated.

## Examples

### Alternative to gearbox joint type

There is an example `gearbox` joint in the
[demo\_joint\_types](https://github.com/osrf/gazebo_models/blob/master/demo_joint_types/model.sdf#L156-L328)
model, consisting of three links,

~~~
        <link name="gearbox_base">
            <pose >-.49 0 0.35 0 0 0</pose>
            <!-- ... -->
        </link>
        <link name="gearbox_input">
            <pose >-.38 -0.075 0.55 0 0 0</pose>
            <!-- ... -->
            <visual name="gearbox_input_visual">
                <geometry>
                    <box>
                        <size>0.1 0.25 0.1</size>
                    </box>
                </geometry>
            </visual>
        </link>
        <link name="gearbox_output">
            <pose >-.3 0.0 0.55 0 1.5708 0</pose>
            <visual name="gearbox_output_visual">
                <geometry>
                    <cylinder>
                        <radius>0.1</radius>
                        <length>0.05</length>
                    </cylinder>
                </geometry>
            </visual>
        </link>
~~~

two `revolute` joints,

~~~
        <!-- Gearbox links revolute joints, so create a couple revolute joints -->
        <joint name="gearbox_input_joint" type="revolute">
            <parent>gearbox_base</parent>
            <child>gearbox_input</child>
            <axis>
                <xyz>1 0 0</xyz>
            </axis>
            <pose>0 0.075 0 0 0 0</pose>
        </joint>
        <joint name="gearbox_output_joint" type="revolute">
            <parent>gearbox_base</parent>
            <child>gearbox_output</child>
            <axis>
                <xyz>0 0 1</xyz>
            </axis>
            <pose>0 0 0 0 0 0</pose>
        </joint>
~~~

and one `gearbox` joint.

~~~
        <joint name="gearbox_demo" type="gearbox">
            <parent>gearbox_input</parent>
            <child>gearbox_output</child>
            <gearbox_reference_body>gearbox_base</gearbox_reference_body>
            <gearbox_ratio>5</gearbox_ratio>
            <!-- input axis (relative to child) -->
            <axis>
                <xyz>0 0 1</xyz>
            </axis>
            <!-- output axis (relative to child) -->
            <axis2>
                <xyz>0 0 1</xyz>
            </axis2>
        </joint>
~~~

The `gearbox` joint could replaced equivalently by adding the ``<mimic>``
tag to joint axes :

~~~
        <!-- Gearbox links revolute joints, so create a couple revolute joints -->
        <joint name="gearbox_input_joint" type="revolute">
            <parent>gearbox_base</parent>
            <child>gearbox_input</child>
            <axis>
                <xyz>1 0 0</xyz>
            </axis>
            <pose>0 0.075 0 0 0 0</pose>
        </joint>
        <joint name="gearbox_output_joint" type="revolute">
            <parent>gearbox_base</parent>
            <child>gearbox_output</child>
            <axis>
                <xyz>1 0 0</xyz>
                <mimic joint="gearbox_input_joint">
                  <multiplier>5</multiplier>
                  <offset>0</offset>
                  <reference>0</reference>
                </mimic>
            </axis>
        </joint>
~~~

### Example of rack and pinion constraint

~~~
<sdf version="1.10">
  <model name="mimic_rack_and_pinion">
    <link name="rack">
        <pose >0 0 0 0 0 0</pose>
        <visual name="box_visual">
            <geometry>
                <box>
                    <size>0.4 0.03 0.03</size>
                </box>
            </geometry>
        </visual>
    </link>
    <link name="pinion">
        <pose degrees='true'>0 0 0.02 90 0 0</pose>
        <visual name="cylinder_visual">
            <geometry>
                <cylinder>
                  <length>0.03</length>
                  <radius>0.02</radius>
                </cylinder>
            </geometry>
        </visual>
    </link>
    <joint name="pinion_joint" type="revolute">
      <parent>chassis</parent>
      <child>pinion</child>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
    </joint>
    <joint name="rack_joint" type="prismatic">
      <parent>chassis</parent>
      <child>rack</child>
      <axis>
        <xyz>1 0 0</xyz>
        <mimic joint="pinion_joint">
          <multiplier>0.02</multiplier>
          <offset>0.0</offset>
          <reference>0.0</reference>
        </mimic>
      </axis>
    </joint>
  </model>
</sdf>
~~~


## Appendix
