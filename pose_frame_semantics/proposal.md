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

## Rename the `//pose[@frame]` attribute to `//pose[@relative_to]`

The `//pose[@frame]` attribute will be renamed to `//pose[@relative_to]` to
avoid confusion between the `//frame` element and `//pose[@frame]` attribute.
The new name is also more descriptive according to the semantics proposed
later in this document, since it specifies that the pose is to be interpreted
relative to the named frame.

## Definition of explicit and implicit frames

A frame is defined by a name, an affixed link (mobilized body) to which the
frame is rigidly affixed, and a pose with respect to another frame.
The pose determines the initial configuration of the frame on the affixed link.
Note that the pose may be expressed relative to a frame that is affixed to a
different link.

Frames can be used to compose information and minimize redundancy
(e.g. specify the bottom center of table, add it to the world with that frame,
then add an object to the top-left-back corner), and can be used to abstract
physical attachments (e.g. specify a camera frame in a model to be used for
inverse kinematics or visual servoing, without the need to also know the
attached link).

### Explicit frame example

A frame can be defined explicitly using the `<frame>` element with
`//frame[@name]` and `//frame[@affixed_to]` attributes along with a `<pose>`
element and `//pose[@relative_to]` attribute.
For example, the following snippet defines an explicit frame `F` affixed to
link `L` with pose `X_AF` relative to frame `A`:

    <frame name="F" affixed_to="L">
      <pose relative_to="A">{X_AF}</pose>
    </frame>

The `//frame[@affixed_to]` and `//pose[@relative_to]` attributes are distinct
in order to support a Model-Absolute paradigm for model building
(see the Addendum on Model Building for further discussion).
The `//frame` element is described in more detail later in this proposal.
For now, note that the `affixed_to` and `relative_to` attributes refer
to other frames by name.

### Implicit frames for links and joints

Since it can be convenient to refer to the link frame specified by
`//link/pose`, each link is given an implicit frame named after itself and
affixed to itself.
Likewise, each joint is given an implicit frame named after itself and affixed
to the child link based on the pose in `//joint/pose`.
The `//pose[@relative_to]` attribute can be used for both link and joint poses to
specify the frame relative to which the pose is expressed.

## Canonical Link

### Implicit frame defined by `//model/pose` affixed to canonical link

Each model has an implicit frame defined by the `//model/pose` element.
This is typically called the "model frame" and
is the frame relative to which all `//link/pose` elements are interpreted
in SDFormat 1.4.
The SDFormat 1.4 specification does not clearly state to which link the
model frame is affixed, but Gazebo has a convention of choosing the first
`<link>` element listed as a child of a `<model>` as the `affixed_to` link
and referring to this as the model's Canonical Link
(see [Model.cc from gazebo 10.1.0](https://bitbucket.org/osrf/gazebo/src/gazebo10_10.1.0/gazebo/physics/Model.cc#lines-130:132)).

The canonical link should become part of SDFormat's specification, and should
be user-configurable but with a default value. These two models are equivalent:

~~~
<model name="test_model" canonical_link="link1">
  <link name="link1"/>
  <link name="link2"/>
</model>
~~~

~~~
<model name="test_model">
  <link name="link1"/>
  <link name="link2"/>
</model>
~~~

Alternatives:

~~~
<!-- //link[@canonical] -->
<model name="test_model">
  <link name="link1" canonical="true"/>
</model>
~~~

~~~
<!-- //model/canonical_link -->
<model name="test_model">
  <canonical_link>link1</canonical_link>
</model>
~~~

### Referencing the implicit model frame

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

## Name conflicts between explicit and implicit frames

As frames are referenced in several attributes by name, it is necessary to
avoid naming conflicts between frames defined in `//model/frame`,
`//model/link`, and `//model/joint`.
This motivates the naming rule proposed in the following section.

### Element naming rule: unique names for all sibling elements

<!-- TODO(eric): These naming rules should stay in this proposal, but should
then transition to a nesting / scoping proposal once they land. -->

While it was not explicitly disallowed in previous versions of the spec, it
can be very confusing when sibling elements of any type have identical names.
In practice, many models include the element type in the name, whether numbered
as `link1`/`link2` or used as a suffix `front_right_wheel_joint`
/ `front_right_steering_joint`, which helps to further ensure name uniqueness
across element types.
Furthermore, the frame semantics proposed in this document use the names of
sibling elements `//model/frame`, `//model/link` and `//model/joint` to refer
to frames.
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
It is a required attribute, and can be used by other frames in the `affixed_to`
and `//pose[@relative_to]` attributes to refer to this frame.
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
It is an optional attribute.
If it is specified, it must contain the name of a sibling explicit or
implicit frame.
If a `//frame` is specified, recursively following the `affixed_to` attributes
of the specified frames must lead to the name of a link.
If the attribute is not specified, the frame is affixed to the model frame
and thus indirectly affixed to the canonical link.

~~~
<model name="frame_affixing">
  <link name="L"/>
  <frame name="F0"/>                  <!-- VALID: Indirectly affixed_to canonical link L via the model frame. -->
  <frame name="F1" affixed_to="L"/>   <!-- VALID: Directly affixed_to link L. -->
  <frame name="F2" affixed_to="F1"/>  <!-- VALID: Indirectly affixed_to link L via frame F1. -->
  <frame name="F3" affixed_to="A"/>   <!-- INVALID: no sibling frame named A. -->
</model>
~~~

~~~
<model name="joint_affixing">
  <link name="P"/>
  <link name="C"/>
  <joint name="J" type="fixed">
    <parent>P</parent>
    <child>C</child>
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

## The `//pose[@relative_to]` attribute

The `//pose[@relative_to]` attribute indicates the frame relative to which the initial
pose of the frame is expressed.
This applies equally to `//frame/pose`, `//link/pose`, and `//joint/pose`
elements.
If `//link/pose[@relative_to]` is not set, it defaults to the model frame,
following the behavior from SDFormat 1.4.
If `//joint/pose[@relative_to]` is not set, it defaults to the child link's
implicit frame, also following the behavior from SDFormat 1.4
(see the "Parent frames in sdf 1.4" section of the
[pose frame semantics tutorial](/tutorials?tut=pose_frame_semantics)).
If the `//frame/pose[@relative_to]` attribute is not set, it should default to
the value of the `//frame[@affixed_to]` attribute.

## Empty `//pose` and `//frame` elements imply identity pose

With the use of the `//pose[@relative_to]` and `//frame[@affixed_to]` attributes,
there are many expected cases when a frame is defined relative to another frame
with no additional pose offset.
To reduce verbosity, empty pose elements are interpreted as equivalent to the
identity pose, as illustrated by the following pairs of equivalent poses:

~~~
<pose />
<pose>0 0 0 0 0 0</pose>
~~~

~~~
<pose relative_to='frame_name' />
<pose relative_to='frame_name'>0 0 0 0 0 0</pose>
~~~

Likewise, empty `//frame` elements are interpreted as having an identity pose
relative to `//frame[@affixed_to]`, as illustrated by the following equivalent
group of frames:

~~~
<frame name="F" affixed_to="A" />
<frame name="F" affixed_to="A">
  <pose />
</frame>
<frame name="F" affixed_to="A">
  <pose relative_to="A" />
</frame>
<frame name="F" affixed_to="A">
  <pose relative_to="A">0 0 0 0 0 0</pose>
</frame>
~~~

## Example using the `//pose[@relative_to]` attribute

For example, consider the following figure from the
[previous tutorial about specifying pose](/tutorials?tut=specify_pose)
that shows a parent link `P`, child link `C`, and joint `J` with joint frames
`Jp` and `Jc` on the parent and child respectively.

<!-- Figure Credit: Alejandro Castro -->

[[file:../spec_model_kinematics/joint_frames.svg|600px]]

An sdformat representation of this model is given below.
The frame named `model_frame` is created so that the implicit model frame
can be referenced explicitly.
The pose of the parent link `P` is specified relative to the implicit
model frame, while the pose of the other
elements is specified relative to other named frames.
This allows poses to be defined recursively, and also allows explicitly named
frames `Jp` and `Jc` to be affixed to the parent and child, respectively.
For reference, equivalent expressions of `Jc` are defined as `Jc1` and `Jc2`.

    <model name="M">

      <frame name="model_frame" />

      <link name="P">
        <pose relative_to="model_frame">{X_MP}</pose>
      </link>

      <link name="C">
        <pose relative_to="P">{X_PC}</pose>   <!-- Recursive pose definition. -->
      </link>

      <joint name="J" type="fixed">
        <pose relative_to="P">{X_PJ}</pose>
        <parent>P</parent>
        <child>C</child>
      </joint>

      <frame name="Jp" affixed_to="P">
        <pose relative_to="J" />
      </frame>

      <frame name="Jc" affixed_to="C">
        <pose relative_to="J" />
      </frame>

      <frame name="Jc1" affixed_to="J">   <!-- Jc1 == Jc, since J is affixed to C -->
        <pose relative_to="J" />
      </frame>

      <frame name="Jc2" affixed_to="J" /> <!-- Jc2 == Jc1, since //pose[@relative_to] defaults to J. -->

    </model>

### Example: Parity with URDF

Recall the example URDF from the Parent frames in URDF section
of the [Pose Frame Semantics: Legacy Behavior tutorial](/tutorials?tut=pose_frame_semantics)
that corresponds to the following image in the
[URDF documentation](http://wiki.ros.org/urdf/XML/model):

<img src="http://wiki.ros.org/urdf/XML/model?action=AttachFile&do=get&target=link.png"
     alt="urdf coordinate frames"
     height="500"/>

That URDF model can be expressed with identical
kinematics as an SDF by using link and joint names in the pose `relative_to`
attribute.

    <model name="model">

      <link name="link1"/>

      <joint name="joint1" type="revolute">
        <pose relative_to="link1">{xyz_L1L2} {rpy_L1L2}</pose>
        <parent>link1</parent>
        <child>link2</child>
      </joint>
      <link name="link2">
        <pose relative_to="joint1" />
      </link>

      <joint name="joint2" type="revolute">
        <pose relative_to="link1">{xyz_L1L3} {rpy_L1L3}</pose>
        <parent>link1</parent>
        <child>link3</child>
      </joint>
      <link name="link3">
        <pose relative_to="joint2" />
      </link>

      <joint name="joint3" type="revolute">
        <pose relative_to="link3">{xyz_L3L4} {rpy_L3L4}</pose>
        <parent>link3</parent>
        <child>link4</child>
      </joint>
      <link name="link4">
        <pose relative_to="joint3" />
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
+        <pose relative_to="link1">{xyz_L1L2} {rpy_L1L2}</pose>
-        <parent link="link1"/>
+        <parent>link1</parent>
-        <child link="link2"/>
+        <child>link2</child>
       </joint>
-      <link name="link2"/>
+      <link name="link2">
+        <pose relative_to="joint1" />
+      </link>

       <joint name="joint2" type="revolute">
-        <origin xyz='{xyz_L1L3}' rpy='{rpy_L1L3}'/>
+        <pose relative_to="link1">{xyz_L1L3} {rpy_L1L3}</pose>
-        <parent link="link1"/>
+        <parent>link1</parent>
-        <child link="link3"/>
+        <child>link3</child>
       </joint>
-      <link name="link3"/>
+      <link name="link3">
+        <pose relative_to="joint2" />
+      </link>

       <joint name="joint3" type="revolute">
-        <origin xyz='{xyz_L3L4}' rpy='{rpy_L3L4}'/>
+        <pose relative_to="link3">{xyz_L3L4} {rpy_L3L4}</pose>
-        <parent link="link3"/>
+        <parent>link3</parent>
-        <child link="link4"/>
+        <child>link4</child>
       </joint>
-      <link name="link4"/>
+      <link name="link4">
+        <pose relative_to="joint3" />
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
pose `relative_to` frames specified for joints and child links, and no link poses.
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
        <pose relative_to="joint1_frame" />
        <parent>link1</parent>
        <child>link2</child>
      </joint>
      <link name="link2">
        <pose relative_to="link2_frame" />
      </link>

      <joint name="joint2" type="revolute">
        <pose relative_to="joint2_frame" />
        <parent>link1</parent>
        <child>link3</child>
      </joint>
      <link name="link3">
        <pose relative_to="link3_frame" />
      </link>

      <joint name="joint3" type="revolute">
        <pose relative_to="joint3_frame" />
        <parent>link3</parent>
        <child>link4</child>
      </joint>
      <link name="link4">
        <pose relative_to="link4_frame" />
      </link>

    </model>

In this case, `joint1_frame` is rigidly affixed to `link1`, `joint3_frame` is
rigidly affixed to `link3`, etc.

## Other uses of `//pose[@relative_to]`

The `//model/frame` and `//pose[@relative_to]` constructs provide much more
flexibility when defining the coordinate frames for sibling `//link`
and `//joint` elements for model kinematics.

A related problem is that there is often duplication of pose information
in elements attached to links.
For example, the following link has two LED light sources, which each have
co-located collision, visual, and light tags, and the pose data is duplicated
within each element.

    <model name="model_with_duplicated_poses">
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

### Possible solution: more implicit frames

This proposal currently only allows implicit frames for `//link` and `//joint`
to be referenced from `//pose[@relative_to]`.
By expanding the number of element types that have implicit frames,
any of these implicit frames could be referenced by name instead of
duplicating the pose data.

For example, if `//light` elements are granted implicit frames,
the example could be rewritten as follows without pose duplication:

    <model name="model_with_implicit_frames">
      <link name="link_with_LEDs">
        <light name="led1_light" type="point">
          <pose>0.1 0 0 0 0 0</pose>
        </light>
        <collision name="led1_collision">
          <pose relative_to="led1_light" />
          <geometry>
            <box>
              <size>0.01 0.01 0.001</size>
            </box>
          </geometry>
        </collision>
        <visual name="led1_visual">
          <pose relative_to="led1_light" />
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
          <pose relative_to="led2_light" />
          <geometry>
            <box>
              <size>0.01 0.01 0.001</size>
            </box>
          </geometry>
        </collision>
        <visual name="led2_visual">
          <pose relative_to="led2_light" />
          <geometry>
            <box>
              <size>0.01 0.01 0.001</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>

The drawbacks of this approach are that increasing the number of implicit
frames increases the size of the frame graph and adds complexity to the
parsing task.

### Possible solution: use `//link/frame`

Instead of adding implicit frames for `//link/collision`, `//link/light`,
`//link/sensor`, and `//link/visual`, explicit frames could be defined
as `//link/frame`.
Any `//frame` elements defined within a `//link` must be affixed to that link.
The `//link/frame[@affixed_to]` and `//link/frame/pose[@relative_to]` attributes
may refer to sibling `//link/frame` elements by name, but if both attributes
are set, the value of `affixed_to` will have no effect.
If the `//link/frame[@affixed_to]` attribute is unspecified, it defaults
to the link's implicit frame.

For example, the model with two LED's is rewritten below using two
`//link/frame` elements.
The `led1_frame` doesn't specify the `//link/frame[@affixed_to]` attribute,
so it is affixed to the implicit link frame by default.
The `led2_frame` is affixed to `led1_frame`, and the `relative_to` attribute
is unset, so it default to be relative to `led1_frame`.
An equivalent `led2_frame_` is given with `relative_to` set instead of
`affixed_to`.

    <model name="model_with_link_frames">
      <link name="link_with_LEDs">
        <frame name="led1_frame">
          <pose>0.1 0 0 0 0 0</pose>
        </frame>
        <frame name="led2_frame" affixed_to="led1_frame">
          <pose>-0.2 0 0 0 0 0</pose>
        </frame>
        <frame name="led2_frame_">
          <pose relative_to="led1_frame">>-0.2 0 0 0 0 0</pose>
        </frame>

        <light name="led1_light" type="point">
          <pose relative_to="led1_frame" />
        </light>
        <collision name="led1_collision">
          <pose relative_to="led1_frame" />
          <geometry>
            <box>
              <size>0.01 0.01 0.001</size>
            </box>
          </geometry>
        </collision>
        <visual name="led1_visual">
          <pose relative_to="led1_frame" />
          <geometry>
            <box>
              <size>0.01 0.01 0.001</size>
            </box>
          </geometry>
        </visual>

        <light name="led2_light" type="point">
          <pose relative_to="led2_frame" />
        </light>
        <collision name="led2_collision">
          <pose relative_to="led2_frame" />
          <geometry>
            <box>
              <size>0.01 0.01 0.001</size>
            </box>
          </geometry>
        </collision>
        <visual name="led2_visual">
          <pose relative_to="led2_frame" />
          <geometry>
            <box>
              <size>0.01 0.01 0.001</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>

Using `//link/frame` elements instead of many implicit frames simplifies
the parsing task, though there may still be data duplication
if the same frame location needs to be referenced from a `//model/frame` scope
and a `//link/frame` scope since non-sibling elements cannot cross-reference
each other.
The forth-coming Nesting proposal should address this concern.

## Element naming rule: reserved names

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
        * `C` at `X_CJc` (`X_CM(0) * X_MJ(0)`)
* Element-Relative Coordinates:
    * Add `P` and `C`; their poses are ignored unless they have a meaningful
    parent (e.g. a weld or other joint)
    * Add `J`, connect:
        * `P` at `X_PJp`
        * `C` at `X_CJc`
    * `X_MP(q)` is driven by parent relationships
    * `X_MC(q)` is driven by `X_MP(q) * X_PJp * X_JpJc(q) * X_JcC`.
