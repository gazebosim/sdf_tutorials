# Proposal Procedure and Format

* **Authors**:
Steve Peters`<scpeters@openrobotics.org>`,
Aditya Pande `<aditya.pande@osrfoundation.org>`
* **Status**: *Draft*
* **SDFormat Version**: *1.8+*
* **`libsdformat` Version**: *11*

## Introduction

This proposal suggests adding a new joint type called the Mimic joint
that adds a linear equality constraint between the output position of
two joints.

Currently, the Gearbox joint adds a similar constraint on the rotation
of `//parent` and `//child` links relative to a reference link
`//gearbox_reference_body`, with the rotation axes specified by
`//axis/xyz` for `//parent` and `//axis2/xyz` for `//child`.
The Gearbox joint is typically used in conjunction with a pair of
revolute joints with identical axes but requires duplication of the
axis definitions.
Another drawback of the Gearbox joint is that  it only constrains rotational
motion, so it cannot model constraints involving translational motion,
such as a rack and pinion mechanism.

The Mimic joint will simplify the definition of this constraint by
specifying joints instead of links in `//parent` and `//child` so
that the joint axis information does not need to be duplicated.
The Mimic joint will be more flexible than the Gearbox joint by
allowing constraints on the output of prismatic joints and other
joints with translational outputs.

## Document summary

Make a bullet list of all major sections:

* "*{section}*: *{single-sentence summary of content in that section}*".

## Syntax (optional)

* Define any possibly-unclear syntax used in the document.

## Motivation

* Elaborate on "Background" statement from "Introduction".
* Elaborate on "Conclusion" statement from "Introduction".
  * "*{changes covered in proposal}* will improve *{concept being iterated on}* by *{key improvements}*".
* Explain why *{changes}* are beneficial/chosen to solve the issues of *{concept}*.
* If you have discussions or material that can easily be cross-referenced and
  accessed, be sure to include them here.

## Proposed changes

Introduce the section before listing changes

### *{number}* *{Proposed change}*

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

There is an example `gearbox` joint in the
[demo\_joint\_types](https://github.com/osrf/gazebo_models/blob/master/demo_joint_types/model.sdf#L156-L328)
model, consisting of three links, two `revolute` joints, and one `gearbox` joint.

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

The `gearbox_demo` joint could be equivalently and more concisely expressed as
the following `mimic_demo` joint:

~~~
        <joint name="mimic_demo" type="mimic">
            <parent>gearbox_input_joint</parent>
            <child>gearbox_output_joint</child>
            <gearbox_ratio>5</gearbox_ratio>
        </joint>
~~~

## Appendix
