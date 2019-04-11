# Pose Frame Semantics Proposal

As described in the
[tutorial on existing behavior for pose frame semantics](/tutorials?tut=pose_frame_semantics),
the `frame` attribute string was added to `<pose>` elements in SDF version 1.5,
but semantics were not fully defined, so it has not yet been used.
This document proposes a series of changes for SDF version 2.0 to
support proposed semantics for more expressivity in SDFormat.
This includes the ability to describe the kinematics of a URDF model
with an SDF 2.0 file.

## Element naming rule: unique names for all sibling elements

While it was not explicitly disallowed in previous versions of the spec, it
can be very confusing when sibling elements of any type have identical names.
In practice, many models include the element type in the name, whether numbered
as `link1`/`link2` or used as a suffix `front_right_wheel_joint`
/ `front_right_steering_joint`, which helps to further ensure name uniqueness
across element types.
As such, all named sibling elements must have unique names.


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

## `<pose frame=''>` attribute

Requiring unique names for sibling elements simplifies the process of
referencing frames by name, as it is sufficient to refer to a name of an
element within the xml hierarchy without specifying the type.
This allows an element's name to implicitly refer to a frame
attached to that element without worry of name collisions between
sibling elements like links and joints.

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
        <pose frame="joint1">0 0 0 0 0 0</pose>
      </link>

      <joint name="joint2" type="revolute">
        <pose frame="link1">{xyz_L1L3} {rpy_L1L3}</pose>
        <parent>link1</parent>
        <child>link3</child>
      </joint>
      <link name="link3">
        <pose frame="joint2">0 0 0 0 0 0</pose>
      </link>

      <joint name="joint3" type="revolute">
        <pose frame="link3">{xyz_L3L4} {rpy_L3L4}</pose>
        <parent>link3</parent>
        <child>link4</child>
      </joint>
      <link name="link4">
        <pose frame="joint3">0 0 0 0 0 0</pose>
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
+        <pose frame="joint1">0 0 0 0 0 0</pose>
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
+        <pose frame="joint2">0 0 0 0 0 0</pose>
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
+        <pose frame="joint3">0 0 0 0 0 0</pose>
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

## Empty `<pose/>` element implies identity pose

With the use of the `frame` attribute in `<pose>` elements, there are
many expected cases when a frame is defined relative to another frame
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

## `<model><frame>` tag

The `<frame>` tag was added in version 1.5 of the SDFormat specification,
though it has seen little use due to the lack of well-defined semantics.
Similar to `<link>` and `<joint>`, it has a `name` attribute and may
contain a child pose element.
As with links and joints, a `<frame>` can be referenced by name by sibling
elements using the `<pose frame=''>` attribute,
and would be subject to the unique name requirement discussed above.

One application of the `<frame>` tag is to organize the model so that the pose
values are all stored in a single part of the model and referenced
by name elsewhere.
For example, the following is equivalent to the SDFormat model discussed
in the previous section.

    <model name="model">

      <frame name="joint1_frame">
        <pose frame="link1">{xyz_L1L2} {rpy_L1L2}</pose>
      </frame>
      <frame name="joint2_frame">
        <pose frame="link1">{xyz_L1L3} {rpy_L1L3}</pose>
      </frame>
      <frame name="joint3_frame">
        <pose frame="link3">{xyz_L3L4} {rpy_L3L4}</pose>
      </frame>

      <frame name="link2_frame">
        <pose frame="joint1" />
      </frame>
      <frame name="link3_frame">
        <pose frame="joint2" />
      </frame>
      <frame name="link4_frame">
        <pose frame="joint3" />
      </frame>

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

## `<link><frame>` tag

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

## Referencing a `<link><frame>` from `<model>` scope

In addition to being useful for organizing elements within a link,
the `<link><frame>` tags can also be useful at the `<model>` scope.
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

## Referencing implicit frames in `<pose frame=''>`

The `<pose frame=''>` attribute references frames by name.
This proposal includes examples for referencing frames created explicitly using
the `<frame>` tag, as well as referencing the frames implicitly attached to
`<link>` and `<joint>` tags by name.
A future proposal will address nested models and model composition, which
will likely include referencing the frames implicitly attached to `<model>` tags
by the model name.
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

