# Pose Frame Semantics Proposal

As described in the
[documentation on existing behavior for pose frame semantics](/tutorials?tut=pose_frame_semantics),
`<frame>` elements were added to several elements, and
the `frame` attribute string was added to `<pose>` elements in SDF version 1.5.
Semantics for the frame element and attribute were not fully defined, however,
so they have not yet been used.
This document proposes a series of changes for SDF version 1.7 to
support semantics for more expressivity of kinematics and coordinate frames
in SDFormat.
This includes the ability to describe the kinematics of a URDF model
with an SDF 1.7 file.

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

## Motivation

Coordinate frames are a fundamental aspect of model specification in SDFormat.
For each element with a [`//pose` tag](/tutorials?tut=specify_pose)
(such as model, link and joint), a coordinate frame is defined, consisting of
an origin and orientation of XYZ coordinate axes.
Coordinate frames are defined by applying the pose transform relative to
another coordinate frame.
In SDF 1.6 and earlier, there are fixed rules for determining the frame
relative to which each subsequent frame is defined
(see the "Parent frames" sections in the documentation for
[Pose frame semantics: legacy behavior](/tutorials?tut=pose_frame_semantics)).

To improve the expressivity of model specification in SDFormat, two features
are being added:

* the ability to define arbitrary coordinate frames within a model
* the ability to choose the frame relative to which each frame is defined

These features allow frames to be used to compose information and minimize redundancy
(e.g. specify the bottom center of table, add it to the world with that frame,
then add an object to the top-left-back corner), and can be used to abstract
physical attachments (e.g. specify a camera frame in a model to be used for
inverse kinematics or visual servoing, without the need to also know the
attached link).

## Terminology for frames and poses

Each pose must be defined **relative to** (or be measured in) a certain frame.
This is captured by the attribute `//pose[@relative_to]`, described below.

Arbitrary frames are defined with the `//frame` tag.
A frame must have a name (`//frame[@name]`),
be **attached to** another frame or link (`//frame[@attached_to]`),
and have a defined pose (`//frame/pose`).
Further details are given below.

It is important to mention:

* A pose being defined **relative to** a given frame does not imply that it
will be **attached to** a given frame.
* A pose's **relative to** frame only defines its *initial configuration*;
any movement due to degrees of freedom will only result in a new pose as
defined by its **attached to** frame.
    * This is done in order to support a "Model-Absolute" paradigm for model
    building; see the Addendum on Model Building for further discussion.

### Explicit vs. Implicit frames

Explicit frames are those defined by `//frame`, described below.
While a `<frame>` element is permitted in many places in sdf 1.5, this proposal
only permits a `<frame>` element to appear in `<model>` (`//model/frame`) and
`<world>` (`//world/frame`) elements.

Implicit frames are introduced for convenience, and are defined by
non-`//frame` elements. The following frame types are implicitly introduced:

* Link frames: each link has a frame named `//link[@name]`, attached to the
  link at its origin defined by `//link/pose`.
* Joint frames: each joint has a frame named `//joint[@name]`, attached to the
  child link at the joint's origin defined by `//joint/pose`.
* Model frame: each model has a frame, but it can only be referenced by a
  `//link/pose` or `//frame/pose` element when its `@relative_to` attribute
  resolves to empty.
* World frame: each world has a fixed inertial reference frame that is
  the default frame to which explicit world frames defined by `//world/frame`
  are attached.
  Model poses defined by `//model/pose` are interpreted relative to the implicit
  world frame when the `//model/pose[@relative_to]` attribute is empty.

These frames and their semantics are described below in more detail.

#### Alternatives considered

Introducing implicit frames for other elements such as `//link/visual`,
`//link/collision`, and `//link/sensor` was considered. However, it was
determined that introducing these implicit frames adds unnecessary complexity
to the SDFormat parser. It would also pollute the frame graph making it less
efficient to traverse.

## Model Frame and Canonical Link

### Implicit frame defined by `//model/pose` attached to canonical link

Each model has an implicit frame defined by the `//model/pose` element.
This is typically called the "model frame" and
is the frame relative to which all `//link/pose` elements are interpreted
in SDFormat 1.4.
The SDFormat 1.4 specification does not clearly state to which link the
model frame is attached, but Gazebo has a convention of choosing the first
`<link>` element listed as a child of a `<model>` as the `attached_to` link
and referring to this as the model's Canonical Link
(see [Model.cc from gazebo 10.1.0](https://bitbucket.org/osrf/gazebo/src/gazebo10_10.1.0/gazebo/physics/Model.cc#lines-130:132)).

The canonical link should become part of SDFormat's specification, and should
be user-configurable but with a default value. These two models are equivalent:

~~~
<!-- //model[@canonical_link] -->
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

Future versions of SDFormat may require that the canonical link always be
explicitly defined.

#### Alternatives considered

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

#### Alternatives considered

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
<sdf version="1.7">
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

#### Alternatives considered

It was considered to specify the frame type in the `//frame[@attached_to]`
and `//pose[@relative_to]` attributes in order to avoid this additional naming
restriction.
For example, the following approach uses a URI with the frame type encoded
as the scheme.

    <model name="model">
      <link name="base"/>
      <frame name="base"/>
      <frame name="link_base">
        <pose relative_to="link://base"/>    <!-- Relative to the link. -->
      <frame name="frame_base">
        <pose relative_to="frame://base"/>   <!-- Relative to the frame. -->
    </model>

While an approach like this would avoid the need for the new naming
restrictions, it would either require always specifying the frame type in
addition to the frame name or add complexity to the specification by allowing
multiple ways to specify the same thing.
Moreover, it was mentioned above that it can be very confusing when sibling
elements of any type have identical names, which mitigates the need to
support non-unique names for sibling elements.
As such, the naming restriction is preferred.

## Details of `//model/frame`

The `//model/frame` has two attributes, `name` and `attached_to`, and a child
`<pose>` element that specifies the initial pose of the frame. Further details
of the attributes of `//model/frame` are given below.

### The `//model/frame[@name]` attribute

The `//model/frame[@name]` attribute specifies the name of a `<frame>`.
It is a required attribute, and can be used by other frames in the `attached_to`
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

### The `//model/frame[@attached_to]` attribute

The `//model/frame[@attached_to]` attribute specifies the link to which the
`<frame>` is attached.
It is an optional attribute.
If it is specified, it must contain the name of a sibling explicit or
implicit frame.
Cycles in the `attached_to` graph are not allowed.
If a `//frame` is specified, recursively following the `attached_to` attributes
of the specified frames must lead to the name of a link.
If the attribute is not specified, the frame is attached to the model frame
and thus indirectly attached to the canonical link.

~~~
<model name="frame_attaching">
  <link name="L"/>
  <frame name="F00"/>                 <!-- VALID: Indirectly attached_to canonical link L via the model frame. -->
  <frame name="F0" attached_to=""/>    <!-- VALID: Indirectly attached_to canonical link L via the model frame. -->
  <frame name="F1" attached_to="L"/>   <!-- VALID: Directly attached_to link L. -->
  <frame name="F2" attached_to="F1"/>  <!-- VALID: Indirectly attached_to link L via frame F1. -->
  <frame name="F3" attached_to="A"/>   <!-- INVALID: no sibling frame named A. -->
</model>
~~~

~~~
<model name="joint_attaching">
  <link name="P"/>
  <link name="C"/>
  <joint name="J" type="fixed">
    <parent>P</parent>
    <child>C</child>
  </joint>
  <frame name="F1" attached_to="P"/>   <!-- VALID: Directly attached_to link P. -->
  <frame name="F2" attached_to="C"/>   <!-- VALID: Directly attached_to link C. -->
  <frame name="F3" attached_to="J"/>   <!-- VALID: Indirectly attached_to link C via joint J. -->
  <frame name="F4" attached_to="F3"/>  <!-- VALID: Indirectly attached_to link C via frame F3. -->
</model>
~~~

~~~
<model name="frame_attaching_cycle">
  <link name="L"/>
  <frame name="F1" attached_to="F2"/>
  <frame name="F2" attached_to="F1"/>  <!-- INVALID: cycle in attached_to graph does not lead to link. -->
</model>
~~~

## Details of `//world/frame`

The `//world/frame` has two attributes, `name` and `attached_to`, and a child
`<pose>` element that specifies the initial pose of the frame. Further details
of the attributes of this `//world/frame` are given below.

### The `//world/frame[@name]` attribute

The `//world/frame[@name]` attribute specifies the name of the frame. To avoid
ambiguity, sibling frames—explicit frames specified by `//world/frame` and
implicit frames specified by `//world/model`—must have unique names.

### The `//world/frame[@attached_to]` attribute

The `//world/frame[@attached_to]` attribute specifies another frame to which
this frame is attached. A `//world/frame` can be attached to an implicit frame
(defined by `//world` or `//world/model`) or to an explicit frame defined by
`//world/frame`. If the `//world/frame[@attached_to]` attribute is not
specified or is left empty, the frame will be attached to the world frame. If
the attribute is specified, it must refer to a sibling `//world/frame` or
`//world/model`.

When a a `//world/frame` is attached to a `//world/model`, it is indirectly
attached to the canonical link of the model.

Similar to `//model/frame`, cycles in the `attached_to` graph are not allowed.
If a `//world/frame` is specified, recursively following the `attached_to`
attributes of the specified frames must lead to the implicit world frame or to
the canonical link of a sibling model.

~~~
<world name="frame_attaching">
  <frame name="F0"/>                   <!-- VALID: Indirectly attached_to the implicit world frame. -->
  <frame name="F1" attached_to=""/>    <!-- VALID: Indirectly attached_to the implicit world frame. -->
  <frame name="F2" attached_to="F1"/>  <!-- VALID: Directly attached to F1, indirectly attached_to the implicit world frame via F1. -->
  <frame name="F3" attached_to="A"/>   <!-- INVALID: no sibling frame named A. -->

  <model name="M0">
    <link name="L"/>                   <!-- Canonical link. -->
  </model>
  <frame name="F4" attached_to="M0"/>  <!-- Valid: Indirectly attached_to to the canonical link, L, of M0. -->
</world>
~~~

~~~
<world name="frame_attaching_cycle">
  <frame name="F1" attached_to="F2"/>
  <frame name="F2" attached_to="F1"/>  <!-- INVALID: cycle in attached_to graph does not lead to the implicit world frame. -->
</world>
~~~

## The `//pose[@relative_to]` attribute

The `//pose[@relative_to]` attribute indicates the frame relative to which the initial
pose of the frame is expressed.
This applies equally to `//frame/pose`, `//link/pose`, and `//joint/pose`
elements.
If `//link/pose[@relative_to]` is empty or not set, it defaults to the model frame,
following the behavior from SDFormat 1.4.
If `//joint/pose[@relative_to]` is empty or not set, it defaults to the child link's
implicit frame, also following the behavior from SDFormat 1.4
(see the "Parent frames in sdf 1.4" section of the
[pose frame semantics documentation](/tutorials?tut=pose_frame_semantics)).
If the `//frame/pose[@relative_to]` attribute is empty or not set, it should default to
the value of the `//frame[@attached_to]` attribute.
Cycles in the `relative_to` attribute graph are not allowed and must be
checked separately from the `attached_to` attribute graph.
Following the `relative_to` attributes of the specified frames must lead to
a frame expressed relative to the model frame. The exceptions to this rule are
`//world/frame/pose[@relative_to]` and `//world/model/pose[@relative_to]` in
which the terminal frame is the implicit world frame.

~~~
<model name="link_pose_relative_to">
  <link name="L1">
    <pose>{X_ML1}</pose>                    <!-- Pose relative_to model frame (M) by default. -->
  </link>
  <link name="L2">
    <pose relative_to="">{X_ML2}</pose>     <!-- Pose relative_to model frame (M) by default. -->
  </link>
  <link name="L3">
    <pose relative_to="L1">{X_L1L3}</pose>  <!-- Pose relative_to link frame (L1 -> M). -->
  </link>

  <link name="cycle1">
    <pose relative_to="cycle2">{X_C1C2}</pose>
  </link>
  <link name="cycle2">
    <pose relative_to="cycle1">{X_C2C1}</pose>  <!-- INVALID: cycle in relative_to graph does not lead to model frame. -->
  </link>
</model>
~~~

~~~
<model name="joint_pose_relative_to">
  <link name="P1"/>                         <!-- Link pose relative to model frame (M) by default. -->
  <link name="C1"/>                         <!-- Link pose relative to model frame (M) by default. -->
  <joint name="J1" type="fixed">
    <pose>{X_C1J1}</pose>                   <!-- Joint pose relative to child link frame (C1 -> M) by default. -->
    <parent>P1</parent>
    <child>C1</child>
  </joint>

  <link name="P2"/>                         <!-- Link pose relative to model frame (M) by default. -->
  <joint name="J2" type="fixed">
    <pose relative_to="P2">{X_P2J2}</pose>  <!-- Joint pose relative to link frame P2 -> M. -->
    <parent>P2</parent>
    <child>C2</child>
  </joint>
  <link name="C2">
    <pose relative_to="J2">{X_J2C2}</pose>  <!-- Link pose relative to joint frame J2 -> P2 -> M. -->
  </link>

  <link name="P3"/>
  <link name="C3">
    <pose relative_to="J3">{X_J3C3}</pose>
  </link>
  <joint name="J3" type="fixed">
    <pose relative_to="C3">{X_C3J3}</pose>  <!-- INVALID: cycle in relative_to graph does not lead to model frame. -->
    <parent>P3</parent>
    <child>C3</child>
  </joint>
</model>
~~~

~~~
<model name="frame_pose_relative_to">
  <link name="L">
    <pose>{X_ML}</pose>                     <!-- Link pose relative_to the model frame (M) by default. -->
  </link>

  <frame name="F0">                         <!-- Frame indirectly attached_to canonical link L via model frame. -->
    <pose>{X_MF0}</pose>                    <!-- Pose relative_to the attached_to frame (M) by default. -->
  </frame>

  <frame name="F1" attached_to="L">          <!-- Frame directly attached_to link L. -->
    <pose>{X_LF1}</pose>                    <!-- Pose relative_to the attached_to frame (L -> M) by default. -->
  </frame>
  <frame name="F2" attached_to="L">          <!-- Frame directly attached_to link L. -->
    <pose relative_to="">{X_LF2}</pose>     <!-- Pose relative_to the attached_to frame (L -> M) by default. -->
  </frame>
  <frame name="F3">                         <!-- Frame indirectly attached_to canonical link L via model frame. -->
    <pose relative_to="L">{X_LF3}</pose>    <!-- Pose relative_to link frame L -> M. -->
  </frame>

  <frame name="cycle1">
    <pose relative_to="cycle2">{X_C1C2}</pose>
  </frame>
  <frame name="cycle2">
    <pose relative_to="cycle1">{X_C2C1}</pose>  <!-- INVALID: cycle in relative_to graph does not lead to model frame. -->
  </frame>
</model>
~~~

~~~
<world name="world_frame_pose_relative_to"> <!-- Implicit world frame. Referred to as (W) -->
  <frame name="F0">                         <!-- Frame indirectly attached_to the implicit world frame. -->
    <pose>{X_WF0}</pose>                    <!-- Pose relative_to the attached_to frame (W) by default. -->
  </frame>

  <frame name="F1" attached_to="F0">        <!-- Frame directly attached_to another explicit frame (F0). -->
    <pose>{X_F0F1}</pose>                   <!-- Pose relative_to the attached_to frame (F0 -> W) by default. -->
  </frame>
  <frame name="F2" attached_to="F0">        <!-- Frame directly attached_to another explicit frame (F0). -->
    <pose relative_to="">{X_F0F2}</pose>    <!-- Pose relative_to the attached_to frame (F0 -> W) by default. -->
  </frame>
  <frame name="F3">                         <!-- Frame indirectly attached_to the implicit world frame. -->
    <pose relative_to="F0">{X_F0F3}</pose>  <!-- Pose relative_to frame (F0 -> W). -->
  </frame>

  <frame name="cycle1">
    <pose relative_to="cycle2">{X_C1C2}</pose>
  </frame>
  <frame name="cycle2">
    <pose relative_to="cycle1">{X_C2C1}</pose>  <!-- INVALID: cycle in relative_to graph does not lead to world frame. -->
  </frame>
</model>
~~~

The following example may look like it has a graph cycle since frame `F1` is
`attached_to` link `L2`, and the pose of link `L2` is `relative_to` frame `F1`.
It is not a cycle, however, since the `attached_to` and `relative_to` attributes
have separate, valid graphs.

~~~
<model name="not_a_cycle">
  <link name="L1">
    <pose>{X_ML1}</pose>                    <!-- Pose relative to model frame (M) by default. -->
  </link>

  <frame name="F1" attached_to="L2">         <!-- Frame directly attached to link L2. -->
    <pose relative_to="L1">{X_L1F1}</pose>  <!-- Pose relative to implicit link frame L1 -> M. -->
  </frame>

  <link name="L2">
    <pose relative_to="F1">{X_F1L2}</pose>  <!-- Pose relative to frame F1 -> L1 -> M. -->
  </link>
</model>
~~~

## Empty `//pose` and `//frame` elements imply identity pose

With the use of the `//pose[@relative_to]` and `//frame[@attached_to]` attributes,
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
relative to `//frame[@attached_to]`, as illustrated by the following equivalent
group of frames:

~~~
<frame name="F" attached_to="A" />
<frame name="F" attached_to="A">
  <pose />
</frame>
<frame name="F" attached_to="A">
  <pose relative_to="A" />
</frame>
<frame name="F" attached_to="A">
  <pose relative_to="A">0 0 0 0 0 0</pose>
</frame>
~~~

## Example using the `//pose[@relative_to]` attribute

For example, consider the following figure from the
[previous documentation about specifying pose](/tutorials?tut=specify_pose)
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
frames `Jp` and `Jc` to be attached to the parent and child, respectively.
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

      <frame name="Jp" attached_to="P">
        <pose relative_to="J" />
      </frame>

      <frame name="Jc" attached_to="C">
        <pose relative_to="J" />
      </frame>

      <frame name="Jc1" attached_to="J">   <!-- Jc1 == Jc, since J is attached to C -->
        <pose relative_to="J" />
      </frame>

      <frame name="Jc2" attached_to="J" /> <!-- Jc2 == Jc1, since //pose[@relative_to] defaults to J. -->

    </model>

### Example: Parity with URDF

Recall the example URDF from the Parent frames in URDF section
of the [Pose Frame Semantics: Legacy Behavior documentation](/tutorials?tut=pose_frame_semantics)
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

#### Alternatives considered

An even simpler approach to getting parity with URDF would be to add an
attribute `//joint[@attached_to_child]` that specifies whether the implicit
joint frame is attached to the child link (true) or the parent link (false).
If `//joint/pose[@relative_to]` is unset, this attribute would determine
the default value of `//joint/pose[@relative_to]`.
For backwards compatibility, the attribute would default to true.
In this example, setting that attribute to false would eliminate the need
to specify the `//joint/pose[@relative_to]` attribute and the duplication
of the parent link name.
As seen below, the `//link/pose[@relative_to]` attributes still need to be set:

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

      <joint name="joint2" type="revolute" attached_to_child="false">
        <pose>{xyz_L1L3} {rpy_L1L3}</pose>
        <parent>link1</parent>
        <child>link3</child>
      </joint>
      <link name="link3">
        <pose relative_to="joint2" />
      </link>

      <joint name="joint3" type="revolute" attached_to_child="false">
        <pose>{xyz_L3L4} {rpy_L3L4}</pose>
        <parent>link3</parent>
        <child>link4</child>
      </joint>
      <link name="link4">
        <pose relative_to="joint3" />
      </link>

    </model>

This change was not included since parity with URDF can already be achieved
with the other propsed functionality.

## Example: Parity with URDF using `//model/frame`

One application of the `<frame>` tag is to organize the model so that the pose
values are all stored in a single part of the model and referenced
by name elsewhere.
For example, the following is equivalent to the SDFormat model discussed
in the previous section.

    <model name="model">

      <frame name="joint1_frame" attached_to="link1">
        <pose>{xyz_L1L2} {rpy_L1L2}</pose>
      </frame>
      <frame name="joint2_frame" attached_to="link1">
        <pose>{xyz_L1L3} {rpy_L1L3}</pose>
      </frame>
      <frame name="joint3_frame" attached_to="link3">
        <pose>{xyz_L3L4} {rpy_L3L4}</pose>
      </frame>

      <frame name="link2_frame" attached_to="joint1"/>
      <frame name="link3_frame" attached_to="joint2"/>
      <frame name="link4_frame" attached_to="joint3"/>

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

In this case, `joint1_frame` is rigidly attached to `link1`, `joint3_frame` is
rigidly attached to `link3`, etc.

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

## Phases of parsing kinematics of an sdf 1.7 model

This section describes phases for parsing the kinematics of an sdf 1.7 model.
It does not discuss proper validation of collision and visual geometries,
link inertia, nested models, and many other parameters.
Several of these phases are similar to the phases of parsing an sdf 1.4
model in the [Legacy behavior documentation](/tutorials?tut=pose_frame_semantics).
In phases that differ from sdf 1.4, *italics* are used to signal the difference.
For new phases, the ***Title:*** is highlighted.

There are *seven* phases for validating the kinematics data in a model:

1.  **XML parsing and schema validation:**
    Parse model file from XML into a tree data structure,
    ensuring that there are no XML syntax errors and that the XML
    data complies with the [schema](http://sdformat.org/schemas/root.xsd).
    Schema `.xsd` files are generated from the `.sdf` specification files
    when building `libsdformat` with the
    [xmlschema.rb script](https://bitbucket.org/osrf/sdformat/src/sdformat6_6.2.0/tools/xmlschema.rb).

2.  **Name attribute checking:**
    Check that name attributes are not an empty string `""`, and that sibling
    elements of *any* type have unique names.
    This includes but is not limited to models, actors, links, joints,
    collisions, visuals, sensors, and lights.
    This step is distinct from validation with the schema because the schema
    only confirms the existence of name attributes, not their content.
    Note that `libsdformat` does not currently perform this check when loading
    an SDF using `sdf::readFile` or `sdf::readString` (see
    [issue sdformat#216](https://bitbucket.org/osrf/sdformat/issues/216).

3.  **Joint parent/child name checking:**
    For each joint, check that the parent and child link names are different
    and that each match the name of a sibling link to the joint,
    with the following exception:
    if "world" is specified as a link name but there is no sibling link
    with that name, then the joint is attached to a fixed reference frame.

4.  ***Check `//model/frame[@attached_to]` attribute values:***
    For each `//model/frame`, if the `attached_to` attribute exists and is not
    an empty string `""`, check that the value of the `attached_to` attribute
    matches the name of a sibling link, joint, or frame.

5.  ***Check `//model/frame[@attached_to]` graph:***
    Construct an `attached_to` directed graph for the model with each vertex
    representing a frame:

    5.1 Add a vertex for the implicit frame of each link in the model.

    5.2 Add a vertex for the implicit model frame with an edge connecting to the
        vertex of the model's canonical link.

    5.3 Add vertices for the implicit frame of each joint with an edge
        connecting from the joint to the vertex of its child link.

    5.4 For each `//model/frame`:

    5.4.1 Add a vertex to the graph.

    5.4.2 If `//model/frame[@attached_to]` exists and is not empty,
          add an edge from the added vertex to the vertex
          named in the `//model/frame[@attached_to]` attribute.

    5.4.3 Otherwise (ie. if the `//model/frame[@attached_to]` attribute
          does not exist or is an empty string `""`),
          add an edge from the added vertex to the model frame vertex.

    5.5 Verify that the graph has no cycles and that by following the directed
        edges, every vertex is connected to a link.
        To identify the link to which each frame is attached, start from the
        vertex for that frame, and follow the directed edges until a link
        is reached.

6.  ***Check `//pose[@relative_to]` attribute values:***
    For each `//model/link/pose`, `//model/joint/pose` and `//model/frame/pose`
    if the `relative_to` attribute exists and is not an empty string `""`,
    check that the value of the `relative_to` attribute
    matches the name of a link, joint, or frame that is a sibling of the element
    that contains the `//pose`.

7.  ***Check `//pose[@relative_to]` graph:***
    Construct a `relative_to` directed graph for the model with each vertex
    representing a frame:

    7.1 Add a vertex for the implicit model frame.

    7.2 Add vertices for each `//model/link`, `//model/joint`, and
        `//model/frame`.

    7.3 For each `//model/link`:

    7.3.1 If `//link/pose[@relative_to]` exists and is not empty,
          add an edge from the link vertex to the vertex named in
          `//link/pose[@relative_to]`.

    7.3.2 Otherwise (ie. if `//link/pose` or `//link/pose[@relative_to]` do not
          exist or `//link/pose[@relative_to]` is an empty string `""`)
          add an edge from the link vertex to the implicit model frame vertex.

    7.4 For each `//model/joint`:

    7.4.1 If `//joint/pose[@relative_to]` exists and is not empty,
          add an edge from the joint vertex to the vertex named in
          `//joint/pose[@relative_to]`.

    7.4.2 Otherwise (ie. if `//joint/pose` or `//joint/pose[@relative_to]` do not
          exist or `//joint/pose[@relative_to]` is an empty string `""`)
          add an edge from the joint vertex to
          the child link vertex named in `//joint/child`.

    7.5 For each `//model/frame`:

    7.5.1 If `//frame/pose[@relative_to]` exists and is not empty,
          add an edge from the frame vertex to the vertex named in
          `//frame/pose[@relative_to]`.

    7.5.2 Otherwise if `//frame[@attached_to]` exists and is not empty
          (ie. if `//frame[@attached_to]` exists and is not an empty string `""`
          and one of the following is true: `//frame/pose` does not exist,
          `//frame/pose[@relative_to]` does not exist, or
          `//frame/pose[@relative_to]` is an empty string `""`)
          add an edge from the frame vertex to the vertex named in
          `//frame[@attached_to]`.

    7.5.3 Otherwise (ie. if neither `//frame[@attached_to]` nor
          `//frame/pose[@relative_to]` are specified)
          add an edge from the frame vertex to the implicit model frame vertex.

    7.6 Verify that the graph has no cycles and that by following the directed
        edges, every vertex is connected to the implicit model frame.

## Phases of parsing kinematics of an sdf 1.7 world

This section describes phases for parsing the kinematics of an sdf 1.7 world.
Several of these phases are similar to the phases of parsing an sdf 1.4
world in the [Legacy behavior documentation](/tutorials?tut=pose_frame_semantics).
In phases that differ from that document, *italics* are used to signal the difference.
For new phases, the ***Title:*** is highlighted.

There are *seven* phases for validating the kinematics data in a world:

1.  **XML parsing and schema validation:**
    Parse world file from XML into a tree data structure,
    ensuring that there are no XML syntax errors and that the XML
    data complies with the [schema](http://sdformat.org/schemas/root.xsd).
    Schema `.xsd` files are generated from the `.sdf` specification files
    when building `libsdformat` with the
    [xmlschema.rb script](https://bitbucket.org/osrf/sdformat/src/sdformat6_6.2.0/tools/xmlschema.rb).

2.  **Name attribute checking:**
    Check that name attributes are not an empty string `""`, and that sibling
    elements of *any* type have unique names.
    This check can be limited to `//world/model[@name]`
    *and `//world/frame[@name]`*
    since other names will be checked in the following step.
    This step is distinct from validation with the schema because the schema
    only confirms the existence of name attributes, not their content.
    Note that `libsdformat` does not currently perform this check when loading
    an SDF using `sdf::readFile` or `sdf::readString` (see
    [issue sdformat#216](https://bitbucket.org/osrf/sdformat/issues/216).

3.  **Model checking:**
    Check each model according to the *seven* phases of parsing kinematics of an
    sdf model.

4.  ***Check `//world/frame[@attached_to]` attribute values:***
    For each `//world/frame`, if the `attached_to` attribute exists and is not
    an empty string `""`, check that the value of the `attached_to` attribute
    matches the name of a sibling model or frame.

5.  ***Check `//world/frame[@attached_to]` graph:***
    Construct an `attached_to` directed graph for the world with each vertex
    representing a frame:

    5.1 Add a vertex for the implicit world frame.

    5.2 Add a vertex for each model in the world.

    5.3 For each `//world/frame`:

    5.3.1 Add a vertex to the graph.

    5.3.2 If `//world/frame[@attached_to]` exists and is not empty,
          add an edge from the added vertex to the vertex named in the
          `//world/frame[@attached_to]` attribute.

    5.3.3 Otherwise (ie. if the `//world/frame[@attached_to]` attribute
          does not exist or is an empty string `""`),
          add an edge from the added vertex to the implicit world frame vertex.

    5.4 Verify that the graph has no cycles and that by following the directed
        edges, every vertex is connected to a model or the implicit world frame.
        If the directed edges lead from a vertex to the implicit world frame,
        then the `//world/frame` corresponding to that vertex is a fixed
        inertial frame.
        If the directed edges lead to a model, then the `//world/frame`
        corresponding to that vertex is attached to the canonical link of that
        model.

6.  ***Check `//pose[@relative_to]` attribute values:***
    For each `//model/pose` and `//world/frame/pose`
    if the `relative_to` attribute exists and is not an empty string `""`,
    check that the value of the `relative_to` attribute
    matches the name of a model or frame that is a sibling of the element
    that contains the `//pose`.

7.  ***Check `//pose[@relative_to]` graph:***
    Construct a `relative_to` directed graph for the model with each vertex
    representing a frame:

    7.1 Add a vertex for the implicit world frame.

    7.2 Add vertices for each `//world/model` and `//world/frame`.

    7.3 For each `//world/model`:

    7.3.1 If `//world/model/pose[@relative_to]` exists and is not empty,
          add an edge from the model vertex to the vertex named in
          `//world/model/pose[@relative_to]`.

    7.3.2 Otherwise (ie. if `//world/model/pose` or
          `//world/model/pose[@relative_to]` do not
          exist or `//world/model/pose[@relative_to]` is an empty string `""`)
          add an edge from the model vertex to the implicit world frame vertex.

    7.4 For each `//world/frame`:

    7.4.1 If `//frame/pose[@relative_to]` exists and is not empty,
          add an edge from the frame vertex to the vertex named in
          `//frame/pose[@relative_to]`.

    7.4.2 Otherwise if `//frame[@attached_to]` exists and is not empty
          (ie. if `//frame[@attached_to]` exists and is not an empty string `""`
          and one of the following is true: `//frame/pose` does not exist,
          `//frame/pose[@relative_to]` does not exist, or
          `//frame/pose[@relative_to]` is an empty string `""`)
          add an edge from the frame vertex to the vertex named in
          `//frame[@attached_to]`.

    7.4.3 Otherwise (ie. if neither `//frame[@attached_to]` nor
          `//frame/pose[@relative_to]` are specified)
          add an edge from the frame vertex to the implicit world frame vertex.

    7.5 Verify that the graph has no cycles and that by following the directed
        edges, every vertex is connected to the implicit world frame.

## Addendum: Model Building, Contrast "Model-Absolute" vs "Element-Relative" Coordinates

`X(0)` implies zero configuration, while `X(q)` implies a value at a given
configuration.

The following are two contrasting interpretations of specifying a parent link
`P` and child link `C`, connected by joint `J` at `Jp` and `Jc`, respectively,
with configuration-dependent transform `X_JpJc(q)`, with `X_JpJc(0) = I`.

* "Model-Absolute" Coordinates:
    * Add `P` at initial pose `X_MP(0)`, `C` at initial pose `X_MC(0)`
    * Add `J` at `X_MJ(0)`, connect:
        * `P` at `X_PJp` (`X_PM(0) * X_MJ(0)`)
        * `C` at `X_CJc` (`X_CM(0) * X_MJ(0)`)
* "Element-Relative" Coordinates:
    * Add `P` and `C`; their poses are ignored unless they have a meaningful
    parent (e.g. a weld or other joint)
    * Add `J`, connect:
        * `P` at `X_PJp`
        * `C` at `X_CJc`
    * `X_MP(q)` is driven by parent relationships
    * `X_MC(q)` is driven by `X_MP(q) * X_PJp * X_JpJc(q) * X_JcC`.
