# Pose Frame Semantics Proposal

As described in the
[tutorial on existing behavior for pose frame semantics](/tutorials?tut=pose_frame_semantics),
`<frame>` elements were added to several elements, and
the `frame` attribute string was added to `<pose>` elements in SDF version 1.5.
Semantics for the frame element and attribute were not fully defined, however,
so they have not yet been used.
This document proposes a series of changes for SDF version 2.0 to
support semantics for more expressivity of kinematics and coordinate frames
in SDFormat.
This includes the ability to describe the kinematics of a URDF model
with an SDF 2.0 file.

**NOTE**: When describing elements or attributes,
[XPath syntax](https://www.w3schools.com/xml/xpath_syntax.asp) is used provide
concise context.
For example, a single `<model>` tag is referred to as `//model` using XPath.
XPath is even more concise for referring to nested tags and attributes.
In the following example, the `<link>` inside the `<model>` tag is referenced
as `//model/link` and the `name` attribute as `//model[@name]`:

    <model name="model_name">
      <link/>
    </model>

## Element naming rule: reserved names and escaped characters

* Since `world` has a special interpretation when specified as a parent
or child link of a joint, it should not be used as a name for any entities
in the simulation.

    ~~~
    <model name="world"/><!-- INVALID: world is a reserved name. -->
    <model name="world_model"/><!-- VALID -->
    ~~~

    ~~~
    <model name="model">
      <link name="world"/><!-- INVALID: world is a reserved name. -->
      <link name="world_link"/><!-- VALID -->
    </model>
    ~~~

* Names that start and end with double underscores (eg. `__wheel__`) are reserved
for use by library implementors. For example, such names might be useful during
parsing for setting sentinel or default names for elements with missing names.

    ~~~
    <model name="__model__"/><!-- INVALID: name starts and ends with __. -->
    ~~~

    ~~~
    <model name="model">
      <link name="__link__"/><!-- INVALID: name starts and ends with __. -->
    </model>
    ~~~

* The forward slash `/` will be replacing `::` as a delimiter between
scoped element names.
Since there are many existing urdf models that use the `/` character in link and
joint names, an accomodation is made to allow for these characters to be escaped
as `\/` when used in name attributes.
Likewise any uses of `\` in a name should be escaped as `\\`.
Some references to `/` use in link and joint names in existing models are given
below.

    * [irobot\_hand\_description in osrf/drcsim](https://bitbucket.org/osrf/drcsim/src/drcsim7_7.0.0/irobot_hand_description/urdf/irobot_hand.urdf.xacro#irobot_hand.urdf.xacro-625:630)
    * [nao\_description in ros-naoqi/nao\_robot](https://github.com/ros-naoqi/nao_robot/blob/0.5.15/nao_description/urdf/naoV50_generated_urdf/nao_sensors.xacro#L7)
    * [r2\_description in nasa robonaut](https://bitbucket.org/nasa_ros_pkg/deprecated_nasa_r2_common/src/15ec0b187aba8ca15e6906c2f9325e4b0d4b45ae/r2_description/urdf/models/r2c_upperbody/r2c.head_sensors.xacro#r2c.head_sensors.xacro-4)
    * [rotors\_description in ethz-asl/rotors\_simulator](https://github.com/ethz-asl/rotors_simulator/blob/3.0.0/rotors_description/urdf/multirotor_base.xacro#L42-L50)

    ```
    <model name="slash_escaping">
      <link name="unescaped/link\name"/> <!-- INVALID: / and \ should be escaped. -->
      <joint name="unescaped/joint\name"/> <!-- INVALID: / and \ should be escaped. -->

      <link name="escaped\/link\\name"/> <!-- VALID. -->

      <link name="parent"/>
      <joint name="escaped\/joint\\name" type="fixed"> <!-- VALID. -->
        <parent>parent</parent>
        <child>escaped\/link\\name</child> <!-- VALID: reference link with escaped name. -->
      </joint>
    </model>
    ```

## Definition of explicit and implicit frames

A frame is defined by a name, an affixed link (mobilized body) to which the
frame is rigidly affixed, and a pose with respect to another frame.
The pose determines the initial configuration of the frame on the affixed link
and the frame relative to which it is expressed may be on a different link.

Frames can be used to compose information and minimize redundancy
(e.g. specify the bottom center of table, add it to the world with that frame,
then add an object to the top-left-back corner), and can be used to abstract
physical attachments (e.g. specify a camera frame in a model to be used for
inverse kinematics or visual servoing, without the need to also know the
attached link).

A frame can be defined explicitly using the `<frame>` element with
`//frame[@name]` and `//frame[@affixed_to]` attributes along with a `<pose>`
element and `//pose[@frame]` attribute.
For example, the following snippet defines an explicit frame `F` affixed to
link `L` with pose `X_AF` relative to frame `A`.

    <frame name="F" affixed_to="L">
      <pose frame="A">{X_AF}</pose>
    </frame>

Since it can be convenient to refer to the link frame specified by
`//link/pose`, each link is given an implicit frame named after itself and
affixed to itself.
Likewise, each joint is given an implicit frame named after itself and affixed
to the child link based on the pose in `//joint/pose`.
The `//pose[@frame]` attribute can be used for both link and joint poses to
specify the frame relative to which the pose is expressed.

As frames are referenced in several attributes by name, it is necessary to
avoid naming conflicts between frames defined in `//model/frame`,
`//model/link`, and `//model/joint`.
This motivates the naming rule proposed in the following section.

## Element naming rule: unique names for all sibling elements

<!-- TODO(eric): These naming rules should stay in this proposal, but should
then transition to a nesting / scoping proposal once they land. -->

While it was not explicitly disallowed in previous versions of the spec, it
can be very confusing when sibling elements of any type have identical names.
In practice, many models include the element type in the name, whether numbered
as `link1`/`link2` or used as a suffix `front_right_wheel_joint`
/ `front_right_steering_joint`, which helps to further ensure name uniqueness
across element types.
Furthermore, the frame semantics proposed in this document use the names of
sibling `//model/frame`, `//model/link` and `//model/joint` to refer to frames.
Thus for the sake of consistency, all named sibling elements must have unique
names.

~~~
<sdf version="1.4">
  <model name="model">
    <link name="base"/>
    <link name="attachment"/>
    <joint name="attachment" type="fixed"> <!-- VALID, but RECOMMEND AGAINST -->
      <parent>base</parent>
      <child>attachment</child>
    </joint>
  </model>
</sdf>
~~~

~~~
<sdf version="2.0">
  <model name="model">
    <link name="base"/>
    <link name="attachment"/>
    <joint name="attachment" type="fixed"> <!-- INVALID, sibling link has same name. -->
      <parent>base</parent>
      <child>attachment</child>
    </joint>
  </model>
</sdf>
~~~

There are some existing SDFormat models that may not comply with this new
requirement. To handle this, a validation tool will be created to identify
models that violate this stricter naming requirement. Furthermore, the
specification version will be incremented so that checks can be added when
converting from older, more permissive versions to the newer, stricter version.

## Details of `//model/frame`

While a `<frame>` element is permitted in many places in sdf 1.5, this proposal
only permits a `<frame>` element to appear in a `<model>` (`//model/frame`).
Further details of the attributes of this element are given below.

### The `//model/frame[@name]` attribute

The `//model/frame[@name]` attribute specifies the name of a `<frame>`.
It is a required attribute, and can be used in the `affixed_to` and
`//pose[@frame]` attributes to refer to this frame.
As stated in a previous section, all sibling elements must have unique names to
avoid ambiguity when referring to frames by name.

~~~
<model name="frame_naming">
  <frame/>          <!-- INVALID: name attribute is missing. -->
  <frame name=''/>  <!-- INVALID: name attribute is empty. -->
  <frame name='A'/> <!-- VALID. -->
  <frame name='B'/> <!-- VALID. -->
</model>
~~~

~~~
<model name="nonunique_sibling_frame_names">
  <frame name='F'/>
  <frame name='F'/> <!-- INVALID: sibling names are not unique. -->
</model>
~~~

~~~
<model name="nonunique_sibling_names">
  <link name='L'/>
  <frame name='L'/> <!-- INVALID: sibling names are not unique. -->
</model>
~~~

### The `//model/frame[@affixed_to]` attribute

The `//model/frame[@affixed_to]` attribute specifies the link to which the
`<frame>` is affixed.
It is a required attribute and must contain the name of a sibling link or frame.
If a frame is specified, recursively following the `affixed_to` attributes
of the specified frames must lead to the name of a link.

~~~
<model name="frame_affixing">
  <link name="L"/>
  <frame name="F1" affixed_to="L"/>   <!-- VALID: Directly affixed_to link L. -->
  <frame name="F2" affixed_to="F1"/>  <!-- VALID: Indirectly affixed_to link L via frame F1. -->
  <frame name="F3" affixed_to="A"/>   <!-- INVALID: no frame or link named A. -->
</model>
~~~

~~~
<model name="joint_affixing">
  <link name="P"/>
  <link name="C"/>
  <joint name="J" type="fixed">
    <parent>P</parent>
    <child>P</child>
  </joint>
  <frame name="F1" affixed_to="P"/>   <!-- VALID: Directly affixed_to link P. -->
  <frame name="F2" affixed_to="C"/>   <!-- VALID: Directly affixed_to link C. -->
  <frame name="F3" affixed_to="J"/>   <!-- VALID: Indirectly affixed_to link C via joint J. -->
  <frame name="F4" affixed_to="F3"/>  <!-- VALID: Indirectly affixed_to link C via frame F3. -->
</model>
~~~

~~~
<model name="frame_affixing_cycle">
  <link name="L"/>
  <frame name="F1" affixed_to="F2"/>
  <frame name="F2" affixed_to="F1"/>  <!-- INVALID: cycle in affixed_to graph does not lead to link. -->
</model>
~~~

## Empty `//pose` and `//frame` elements implies identity pose

With the use of the `//pose[@frame]` and `//frame[@affixed_to]` attributes,
there are many expected cases when a frame is defined relative to another frame
with no additional pose offset.
To reduce verbosity, empty pose elements are interpreted as equivalent to the
identity pose, as illustrated by the following pairs of equivalent poses:

~~~
<pose />
<pose>0 0 0 0 0 0</pose>
~~~

~~~
<pose frame='frame_name' />
<pose frame='frame_name'>0 0 0 0 0 0</pose>
~~~

Likewise, empty `//frame` elements are interpreted as identity pose relative
to `//frame[@affixed_to]`, as illustrated by the following equivalent group
of frames:

~~~
<frame name="F" affixed_to="A" />
<frame name="F" affixed_to="A">
  <pose frame="A" />
</frame>
<frame name="F" affixed_to="A">
  <pose frame="A">0 0 0 0 0 0</pose>
</frame>
~~~

## The `//pose[@frame]` attribute

The `//pose[@frame]` attribute indicates the frame relative to which the initial
pose of the frame is expressed.
This applies equally to `//frame/pose`, `//link/pose`, and `//joint/pose`
elements.
If the `//link/pose[@frame]` or `//joint/pose[@frame]` attribute is not set,
then the parent frames from sdf 1.4 should be used
(see the "Parent frames in sdf 1.4" section of the
[pose frame semantics tutorial](/tutorials?tut=pose_frame_semantics)).
If the `//frame/pose[@frame]` attribute is not set, it should default to
the value of the `//frame[@affixed_to]` attribute.

For example, consider the figure given in the
[previous tutorial about specifying pose](/tutorials?tut=specify_pose)
that shows a parent link `P`, child link `C`, and joint frames `Jp` and `Jc`
on the parent and child respectively.

<!-- Figure Credit: Alejandro Castro -->

[[file:../spec_model_kinematics/joint_frames.svg|600px]]

The `//pose[@frame]` attribute is not specified for the parent link `P`, so it
is specified relative to the model frame `M`, while the pose of the other
elements is specified relative to other named frames.
This allows poses to be defined recursively, and also allows explicitly named
frames `Jp` and `Jc` to be affixed to the parent and child, respectively.

    <model name="M">

      <link name="P">
        <pose>{X_MP}</pose>   <!-- //pose[@frame] unspecified, default to model frame. -->
      </link>

      <link name="C">
        <pose frame="P">{X_PC}</pose>   <!-- Recursive pose definition. -->
      </link>

      <joint name="J" type="fixed">
        <pose frame="P">{X_PJ}</pose>
        <parent>P</parent>
        <child>C</child>
      </joint>

      <frame name="Jp" affixed_to="P">
        <pose frame="J" />
      </frame>

      <frame name="Jc" affixed_to="C">
        <pose frame="J" />
      </frame>

      <frame name="Jc1" affixed_to="J">   <!-- Jc1 == Jc, since J is affixed to C -->
        <pose frame="J" />
      </frame>

      <frame name="Jc2" affixed_to="J" /> <!-- Jc2 == Jc1, since //pose[@frame] defaults to J. -->

    </model>

**TODO(eric)**: How to explicitly refer the model's canonical frame (e.g.
default `//model/pose[@frame]`) to counter the implicit behavior of
`@affixed_to`?

### Example: Parity with URDF

For example, recall the example URDF from the Parent frames in URDF section
of the [Pose Frame Semantics: Legacy Behavior tutorial](/tutorials?tut=pose_frame_semantics)
that corresponds to the following image in the
[URDF documentation](http://wiki.ros.org/urdf/XML/model):

<img src="http://wiki.ros.org/urdf/XML/model?action=AttachFile&do=get&target=link.png"
     alt="urdf coordinate frames"
     height="500"/>

That URDF model can be expressed with identical
kinematics as an SDF by using link and joint names in the pose `frame`
attribute.

    <model name="model">

      <link name="link1"/>

      <joint name="joint1" type="revolute">
        <pose frame="link1">{xyz_L1L2} {rpy_L1L2}</pose>
        <parent>link1</parent>
        <child>link2</child>
      </joint>
      <link name="link2">
        <pose frame="joint1" />
      </link>

      <joint name="joint2" type="revolute">
        <pose frame="link1">{xyz_L1L3} {rpy_L1L3}</pose>
        <parent>link1</parent>
        <child>link3</child>
      </joint>
      <link name="link3">
        <pose frame="joint2" />
      </link>

      <joint name="joint3" type="revolute">
        <pose frame="link3">{xyz_L3L4} {rpy_L3L4}</pose>
        <parent>link3</parent>
        <child>link4</child>
      </joint>
      <link name="link4">
        <pose frame="joint3" />
      </link>

    </model>

The difference between the URDF and SDF expressions is shown in the patch below:

~~~diff
--- model.urdf
+++ model.sdf
@@ -1,26 +1,32 @@
-    <robot name="model">
+    <model name="model">

       <link name="link1"/>

       <joint name="joint1" type="revolute">
-        <origin xyz='{xyz_L1L2}' rpy='{rpy_L1L2}'/>
+        <pose frame="link1">{xyz_L1L2} {rpy_L1L2}</pose>
-        <parent link="link1"/>
+        <parent>link1</parent>
-        <child link="link2"/>
+        <child>link2</child>
       </joint>
-      <link name="link2"/>
+      <link name="link2">
+        <pose frame="joint1" />
+      </link>

       <joint name="joint2" type="revolute">
-        <origin xyz='{xyz_L1L3}' rpy='{rpy_L1L3}'/>
+        <pose frame="link1">{xyz_L1L3} {rpy_L1L3}</pose>
-        <parent link="link1"/>
+        <parent>link1</parent>
-        <child link="link3"/>
+        <child>link3</child>
       </joint>
-      <link name="link3"/>
+      <link name="link3">
+        <pose frame="joint2" />
+      </link>

       <joint name="joint3" type="revolute">
-        <origin xyz='{xyz_L3L4}' rpy='{rpy_L3L4}'/>
+        <pose frame="link3">{xyz_L3L4} {rpy_L3L4}</pose>
-        <parent link="link3"/>
+        <parent>link3</parent>
-        <child link="link4"/>
+        <child>link4</child>
       </joint>
-      <link name="link4"/>
+      <link name="link4">
+        <pose frame="joint3" />
+      </link>

-    </robot>
+    </model>
~~~

These semantics provide powerful expressiveness for constructing models
using relative coordinate frames.
This can reduce duplication of pose transform data and eliminates
the need to use forward kinematics to compute the assembled poses
of links.

One use case is enabling a well-formed SDFormat file to be easily converted to URDF
by directly copying `xyz` and `rpy` values and without performing any
coordinate transformations.
The well-formed SDFormat file must have kinematics with a tree structure,
pose frames specified for joints and child links, and no link poses.
A validator could be created to identify SDF files that can be directly
converted to URDF with minimal modifications based on these principles.

## Example: Parity with URDF using `//model/frame`

One application of the `<frame>` tag is to organize the model so that the pose
values are all stored in a single part of the model and referenced
by name elsewhere.
For example, the following is equivalent to the SDFormat model discussed
in the previous section.

    <model name="model">

      <frame name="joint1_frame" affixed_to="link1">
        <pose>{xyz_L1L2} {rpy_L1L2}</pose>
      </frame>
      <frame name="joint2_frame" affixed_to="link1">
        <pose>{xyz_L1L3} {rpy_L1L3}</pose>
      </frame>
      <frame name="joint3_frame" affixed_to="link3">
        <pose>{xyz_L3L4} {rpy_L3L4}</pose>
      </frame>

      <frame name="link2_frame" affixed_to="joint1"/>
      <frame name="link3_frame" affixed_to="joint2"/>
      <frame name="link4_frame" affixed_to="joint3"/>

      <link name="link1"/>

      <joint name="joint1" type="revolute">
        <pose frame="joint1_frame" />
        <parent>link1</parent>
        <child>link2</child>
      </joint>
      <link name="link2">
        <pose frame="link2_frame" />
      </link>

      <joint name="joint2" type="revolute">
        <pose frame="joint2_frame" />
        <parent>link1</parent>
        <child>link3</child>
      </joint>
      <link name="link3">
        <pose frame="link3_frame" />
      </link>

      <joint name="joint3" type="revolute">
        <pose frame="joint3_frame" />
        <parent>link3</parent>
        <child>link4</child>
      </joint>
      <link name="link4">
        <pose frame="link4_frame" />
      </link>

    </model>

In this case, `joint1_frame` is rigidly affixed to `link1`, `joint3_frame` is
rigidly affixed to `link3`, etc.

## `//link/frame` Examples

The `<frame>` tag can also be attached to a link to create a body-fixed frame
on that link.
This can be used to organize the
collisions, visuals, sensors, and lights attached to a link.

For example, the following model shows a link with two LED's
represented by two sets of co-located collisions, visuals, and lights.

    <model name="model_without_frames">
      <link name="link_with_LEDs">
        <light name="led1_light" type="point">
          <pose>0.1 0 0 0 0 0</pose>
        </light>
        <collision name="led1_collision">
          <pose>0.1 0 0 0 0 0</pose>
          <geometry>
            <box>
              <size>0.01 0.01 0.001</size>
            </box>
          </geometry>
        </collision>
        <visual name="led1_visual">
          <pose>0.1 0 0 0 0 0</pose>
          <geometry>
            <box>
              <size>0.01 0.01 0.001</size>
            </box>
          </geometry>
        </visual>

        <light name="led2_light" type="point">
          <pose>-0.1 0 0 0 0 0</pose>
        </light>
        <collision name="led2_collision">
          <pose>-0.1 0 0 0 0 0</pose>
          <geometry>
            <box>
              <size>0.01 0.01 0.001</size>
            </box>
          </geometry>
        </collision>
        <visual name="led2_visual">
          <pose>-0.1 0 0 0 0 0</pose>
          <geometry>
            <box>
              <size>0.01 0.01 0.001</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>

Note that the pose information is duplicated between the collision, visual
and light elements of each LED.
By using a `<frame>` for each LED, the pose information can be stored
in one place and used by these elements.

    <model name="model_with_frames">
      <link name="link_with_LEDs">
        <frame name="led1_frame">
          <pose>0.1 0 0 0 0 0</pose>
        </frame>
        <frame name="led2_frame">
          <pose>-0.1 0 0 0 0 0</pose>
        </frame>

        <light name="led1_light" type="point">
          <pose frame="led1_frame" />
        </light>
        <collision name="led1_collision">
          <pose frame="led1_frame" />
          <geometry>
            <box>
              <size>0.01 0.01 0.001</size>
            </box>
          </geometry>
        </collision>
        <visual name="led1_visual">
          <pose frame="led1_frame" />
          <geometry>
            <box>
              <size>0.01 0.01 0.001</size>
            </box>
          </geometry>
        </visual>

        <light name="led2_light" type="point">
          <pose frame="led2_frame" />
        </light>
        <collision name="led2_collision">
          <pose frame="led2_frame" />
          <geometry>
            <box>
              <size>0.01 0.01 0.001</size>
            </box>
          </geometry>
        </collision>
        <visual name="led2_visual">
          <pose frame="led2_frame" />
          <geometry>
            <box>
              <size>0.01 0.01 0.001</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>

## Referencing a `//link/frame` from `//model` scope

In addition to being useful for organizing elements within a link,
the `//link/frame` tags can also be useful at the `//model` scope.
To refer to a `<frame>` embedded in a `<link>`, use the link name,
followed by a `/`, followed by the frame name.

    <model name="attach_to_embedded_link_frame">
      <link name="base">
        <frame name="mounting_point1">
          <pose>0.5 0.2 0 0 0 0</pose>
        </frame>
        <frame name="mounting_point2">
          <pose>-0.5 0.2 0 0 0 0</pose>
        </frame>
      </link>

      <!-- SDF 1.4 kinematics: child -> joint -->
      <link name="attachment1_link">
        <pose frame="base/mounting_point1" />
      </link>
      <joint name="attachment1_joint" type="fixed">
        <!-- implicit joint pose at child link -->
        <parent>base</parent>
        <child>attachment1_link</child>
      </joint>

      <!-- URDF kinematics: parent -> joint -> child -->
      <joint name="attachment2_joint" type="fixed">
        <pose frame="base/mounting_point2" />
        <parent>base</parent>
        <child>attachment2_link</child>
      </joint>
      <link name="attachment2_link">
        <pose frame="attachment2_joint" />
      </link>
    </model>

As discussed in a previous section, any `/` or `\` characters in link and
joint names must be escaped, which allows the `/` to be used as a
separator.

    <model name="embedded_link_frame_escaping">
      <link name="escaped\/link\\name">
        <frame name="frame_name">
          <pose>0.5 0.2 0 0 0 0</pose>
        </frame>
      </link>

      <link name="link">
        <pose frame="escaped\/link\\name/frame_name"/> <!-- VALID. -->
      </link>
    </model>

## Valid parent elements for `<frame>` tags

In the SDF 1.5 specification, the `<frame>` element was added to many elements
as a sibling of `<pose>`, but it is not clear that it is useful to allow
so many locations for `<frame>` tags.
This proposal has examples for `<frame>` tags as children of `<model>` and
`<link>` and will restrict the `<frame>` tags to these elements unless a
compelling use case is provided for other elements.

    <model name="valid_frame_locations">
      <frame name="model_frame"> <!-- VALID. -->
        <pose/>
      </frame>

      <link name="link">
        <frame name="link_frame"> <!-- VALID. -->
          <pose/>
        </frame>

        <inertial>
          <frame name="inertial_frame"> <!-- INVALID. -->
            <pose/>
          </frame>
        </inertial>

        <collision name="collision">
          <frame name="collision_frame"> <!-- INVALID. -->
            <pose/>
          </frame>
        </collision>

        <visual name="visual">
          <frame name="visual_frame"> <!-- INVALID. -->
            <pose/>
          </frame>
        </visual>

        <sensor name="sensor">
          <frame name="sensor_frame"> <!-- INVALID. -->
            <pose/>
          </frame>
        </sensor>

        <light name="light">
          <frame name="light_frame"> <!-- INVALID. -->
            <pose/>
          </frame>
        </light>
      </link>

      <joint name="joint">
        <frame name="joint_frame"> <!-- INVALID. -->
          <pose/>
        </frame>

        <sensor name="sensor">
          <frame name="sensor_frame"> <!-- INVALID. -->
            <pose/>
          </frame>
        </sensor>
      </joint>
    </model>

## Referencing implicit frames in `//pose[@frame]`

The `//pose[@frame]` attribute references frames by name.
This proposal includes examples for referencing frames created explicitly using
the `<frame>` tag, as well as referencing the frames implicitly attached to
`<link>` and `<joint>` tags by name.

Requiring unique names for sibling elements simplifies the process of
referencing frames by name, as it is sufficient to refer to a name of an
element within the xml hierarchy without specifying the type.
This allows an element's name to implicitly refer to a frame
attached to that element without worry of name collisions between
sibling elements like links and joints.

As mentioned above, specifying `//pose[@frame]` generally only indicates the
absolute pose, but not the parent link; as such, typically the user will only
need to worry about the initial configuration.

However, all frames that can be refered to via the model should still have
physically meaningful values at non-zero configuration, and thus should be
attached to a meaningful link.

There are several other tags contained by a `<link>` or `<joint>` that contain a
`<pose>`, which could make them eligible for implicit frames.
These include `<collision>`, `<visual>`, `<link><sensor>`, `<joint><sensor>`,
`<light>`, and `<inertial>`.
To simplify the parsing task, these other tags will not be considered as having
implicit frames that may be referenced from `<pose frame=''>` by the name of
the parent tag.

    <model name="implicit_frame_names">
      <frame name="model_frame_name">
        <pose/>
      </frame>

      <link name="link_name">
        <frame name="link_frame_name">
          <pose/>
        </frame>
        <collision name="collision_name"/>
        <visual name="visual_name"/>
        <sensor name="sensor_name"/>
        <light name="light_name"/>

        <frame name="relative_to_link_frame">
          <pose frame="link_frame_name"/> <!-- VALID. -->
        </frame>

        <frame name="relative_to_collision">
          <pose frame="collision_name"/> <!-- INVALID. -->
        </frame>

        <frame name="relative_to_visual">
          <pose frame="visual_name"/> <!-- INVALID. -->
        </frame>

        <frame name="relative_to_sensor">
          <pose frame="sensor_name"/> <!-- INVALID. -->
        </frame>

        <frame name="relative_to_light">
          <pose frame="light_name"/> <!-- INVALID. -->
        </frame>
      </link>

      <joint name="joint_name"/>

      <frame name="relative_to_model_frame">
        <pose frame="model_frame_name"/> <!-- VALID. -->
      </frame>

      <frame name="relative_to_link">
        <pose frame="link_name"/> <!-- VALID. -->
      </frame>

      <frame name="relative_to_joint">
        <pose frame="joint_name"/> <!-- VALID. -->
      </frame>
    </model>

### Joint: Implicit Frame's Affixed Link

Given that joint frames can be implicitly referenced, they should have a
useful non-zero configuration. Since prior versions of SDFormat specified a
joint's pose by default w.r.t. the child link, it is proposed to make the
joint's implicit frame coincide with `Jc`, and thus be affixed to the child
link.

## Model Frame

When a model is defined, a model frame is effectively assigned. This should
define the following:

* The default `model/frame[@affixed_to]` for:
    * `//model/frame`
* The default `pose[@frame]` for:
    * `//model/pose`
    * `//link/pose`
    * `//joint/pose`

To be congruent with prior suggestions in this proposal, the model frame should
physically attached to a link and its value be explicitly referenceable.

### Explicitly Referenceable

While it would be useful to explicitly refer to a model frame by the model
name, there are two complications: it may conflict with link names, and for
model composition, users may be able to override the model name via
`//include`.

This proposal suggests that instead of auto-assigning a model frame, the
specification provides a means to use the *value* of the model frame (rather
than implicitly allocate a name and restrict that name's usage). A user can
achieve this by defining `//model/frame` with an identity pose. Example:

    <model name="test_model">
      <frame name="model_frame"/>
    </model>

Nested models will have their own individual model frames. (See pending Nesting
proposal for nuances.)

Alternatives:

* The model frame is named as the model's name as specified by the file (not
overridden by `//include`). No link, joint, or frame can be specified using
this name.
* The model frame be explicitly referable to using the **reserved name**
`"model_frame"`. No link, joint, or frame can be specified using this name.

### Canonical Link

At present, Gazebo assigns a model a canonical link, which is effectively the
link that any "model-affixed" frames are actually affixed to. This does not
affect the pose. For SDFormat <2, [Gazebo 10.1.0 chooses the first link](https://bitbucket.org/osrf/gazebo/src/0d2a582e4e1443a3b989ef588023a60f61cb56d9/gazebo/physics/Model.cc#lines-130:132) as the canonical link.

The canonical link should become part of SDFormat's specification, and should
be user-configurable but with a default value. These two models are equivalent:

    <model name="test_model" canonical_link="link1">
      <link name="link1"/>
      <link name="link2"/>
    </model>

    <model name="test_model">
      <link name="link1"/>
      <link name="link2"/>
    </model>

For nested models, it is suggested that each model have its *own* link
(contrary to current Gazebo implementation???), since models could be composed,
but may not necessarily be rigidly affixed to each other. For example:

    <model name="top" canonical_link="link1">
      <link name="link1"/>
      <model name="sub" canonical_link="link1">
        <link name="link1"/>
      </model>
    </model>

Atlernatives:

    <!-- //link[@canonical] -->
    <model name="test_model">
      <link name="link1" canonical="true"/>
    </model>

    <!-- //model/canonical_link -->
    <model name="test_model">
      <canonical_link>link1</canonical_link>
    </model>

## Future Proposal: Scoping Limitations

For simplicity, this should start out with a conservative level of scoping
access, to permit functional encapsulation.

This conservative scoping shall be a constraint on which "direction" a frame
can be referred to: a parent element can "dig into" a child element's frame;
however, a child element cannot access its parent element frame.

The motivation for this is that supporting `//pose[@frame]` implicates some
level of "encompassed" parsing; that is, in a `//model` instance, almost all
frames should be known.

If global references are to be permitted, then this would force composition be
driven through SDFormat, which can be seen as a severe limitation. More
concretely, if one model file can reference "up" in the hierarchy, then the
semantics of that model is completely entangled with its context, and permits
its reuse elsewhere.

All frames should be considered relative to the encompassing element, and thus
for a `//model`, all initial poses should ultimately be rendered relative to
the initial model pose. A model should not have access to a global "world" frame
beyond it's own relative scope. Rationale:

* It *really* complicates parsing. To accurately process a model, a user must
now track all frames and cycles *for the entirety* of world creation. Due to use
of absolute coordinates, *no* valid relative poses could be computed until the
very end.
* If a model is permitted to go "up" and leak out of encapsulation, then that
model can *never* be used in a situation where it might be attached elsewhere.
(This has been a pain point in Drake with IIWA models being grounded, using
`//joint/parent = "world"`.)

If a user needs a model fixtured to world, and needs to incorporate offsets into
that fixture, that should be *in the context of application*, and not be
permissible in the model file itself. Has a very explicitly defined scenario for
welding a body to the ground at a specific pose, then they be in the practice of
specifying the scenario as a `//world` element.

### Followup: Restrict World-Welding Joints to `//world/joint`, not `//model/joint`

See above: This admits more explicit denotation of encapsulation, and promotes
better usage.

## Addendum: Model Building, Contrast Model-Absolute vs Element-Relative Coordinates

`X(0)` implies zero configuration, while `X(q)` implies a value at a given
configuration.

The following are two contrasting interpretations of specifying a parent link
`P` and child link `C`, connected by joint `J` at `Jp` and `Jc`, respectively,
with configuration-dependent transform `X_JpJc(q)`, with `X_JpJc(0) = I`.

* Model-Absolute Coordinates:
    * Add `P` at initial pose `X_MP(0)`, `C` at initial pose `X_MC(0)`
    * Add `J` at `X_MJ(0)`, connect:
        * `P` at `X_PJp` (`X_PM(0) * X_MJ(0)`)
        * `C` at `X_CJc` (`X_PC(0) * X_MC(0)`)
* Element-Relative Coordinates:
    * Add `P` and `C`; their poses are ignored unless they have a meaningful
    parent (e.g. a weld or other joint)
    * Add `J`, connect:
        * `P` at `X_PJp`
        * `C` at `X_CJc`
    * `X_MP(q)` is driven by parent relationships
    * `X_MC(q)` is driven by `X_MP(q) * X_PJp * X_JpJc(q) * X_JcC`.
