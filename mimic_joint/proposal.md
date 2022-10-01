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

Currently, the Gearbox joint type adds a similar constraint on the rotation
of `//parent` and `//child` links relative to a reference link
`//gearbox_reference_body`, with the rotation axes specified by
`//axis/xyz` for `//parent` and `//axis2/xyz` for `//child`.
The Gearbox joint is typically used in conjunction with a pair of
revolute joints with identical axes but requires duplication of the
axis definitions.
Another drawback of the Gearbox joint is that it only constrains rotational
motion, so it cannot model constraints involving translational motion,
such as a rack and pinion mechanism.

The Mimic constraint will simplify the definition of this constraint by
specifying the joint axes to which it applies instead of links so
that the joint axis information does not need to be duplicated.
It will be more flexible than the Gearbox joint by
allowing constraints on the output of prismatic joints and other
joints with translational outputs.

An alternative was to add a new joint type called a Mimic joint,
but since URDF already supports the `//joint/mimic` tag (see
[URDF documentation](https://wiki.ros.org/urdf/XML/joint) and
[ros/robot\_state\_publisher#1](https://github.com/ros/robot_state_publisher/issues/1))
it would be more consistent to add a new sdf tag called ``<mimic>`` inside
the ``//joint/axis/`` tag.

## Document summary

Make a bullet list of all major sections:

* "*{section}*: *{single-sentence summary of content in that section}*".

## Syntax (optional)

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

* Elaborate on "Background" statement from "Introduction".
* Elaborate on "Conclusion" statement from "Introduction".
  * "*{changes covered in proposal}* will improve *{concept being iterated on}* by *{key improvements}*".
* Explain why *{changes}* are beneficial/chosen to solve the issues of *{concept}*.
* If you have discussions or material that can easily be cross-referenced and
  accessed, be sure to include them here.

## Proposed changes

Introduce the section before listing changes

### Definition of Mimic Constraint

A Mimic Constraint encodes a linear equality constraint on the position of
two joint axes with `multiplier` and `offset` parameters according to the
equation below. These parameter names match the
[URDF mimic](https://wiki.ros.org/urdf/XML/joint#Elements)
tag parameter names.

`mimic_angle = multiplier * other_joint_angle + offset`

### Addition of //axis/mimic and //axis2/mimic tags

A new `//mimic` tag is defined and added to the `//axis` and `//axis2` elements
to encode a mimic constraint between two joint axes.
The `//mimic` tag encodes the `multiplier` and `offset` parameters in the
`//mimic/multiplier` and `//mimic/offset` elements, respectively.
The name and axis of the joint to be mimicked are specified in the
`//mimic/@joint` and `//mimic/@axis` attributes, respectively.
The `@joint` attribute is required and must resolve to a joint name from
the current scope.
The `@axis` attribute is optional with a default value of `axis`. A value of
`axis2` may be specified in `@axis` if `@joint` refers to a multi-axis joint.
The only valid values of `//mimic/@axis` are `axis` and `axis2`.

~~~
<mimic joint="joint_name" axis="axis">
  <multiplier>1.0</multiplier>
  <offset>0.0</offset>
</mimic>
~~~

**Change**

"*{concept}* must *{behavior}*".

**Details**

* "This is achieved by..." (or simply list details without standard language).
* Use bullet lists for long lists of like-details.
* Code snippets/examples belong here.

**Previous behavior**

* "Previously..." or "In *{previous version}*..."
* Referring to current behavior, but written in past tense (POV of the
proposal).

**Justification**

"*{change}* is necessary because..."

For proposals with many "Proposed changes":

* Number each group of like changes under a descriptive `###` subheading.
* Individual changes under `####` subheadings.
* "Alternatives considered" for each individual change under `#####` subheading.
* If "Previous behavior" and/or "Justification" sections are shared by all the
individual changes under one `###` heading, those parts can go directly under
the `###` heading instead of under each `####` heading.

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
        </mimic>
      </axis>
    </joint>
  </model>
</sdf>
~~~


## Appendix
