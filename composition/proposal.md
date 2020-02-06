# Model composition: Proposed behavior

## Introduction

This proposal suggests changes to the semantics of composition, targetting
SDFormat 1.8. As described in the
[legacy behavior tutorial](/tutorials?tut=composition), SDFormat 1.6 has the
`//include` tag which admits composition, but it does not have many explicit
provisions for encapsulation and modularity, as well as the ability to include
models of other formats. This changes proposed here intend to specify the
encapsulation to admit better semantics for assembly, and a means to make the
`//include` tag admit other models.

## Document summary

The proposal includes the following sections:

*   Motivation: background and rationale.
*   Proposed changes: Each additon / subtraction to SDFormat and `libsdformat`.
*   Examples: Long form code samples.

## Motivation

When constructing models of robots, composition / nesting is extremely useful
when making nontrivial assemblies, and especially when handling different
versions of worlds or robots (e.g. adding multiple instances of a robot, or
swapping out grippers).

At present (SDFormat 1.7), the nesting a model via the `//include` tag or via
direct elements has not been explicitly specified to perform the same
operations, part of the reason being that nesting directly is not actually implemented in the current `libsdformat` (9.1.0). Nesting of models generally
implies that the elements can be referred via a form of scope, such as
`{scope}::{name}`. However, `::` is not a special token and thus can be
used to create "false" hierarchy or potential collision. Additionally, there is
no way for elements within the same file to refer "up" to another element, e.g. with in a robot assembly, adding a weld between a gripper and an arm when the
two are sibiling models.

For posturing an included model, there is no means by which the user can
specify which included frame to posture via `//include/pose`. The target frame
to move will be the
[canonical link](/tutorials?tut=pose_frame_semantics_proposal#2-model-frame-and-canonical-link).
Therfore, if you wanted to weld a gripper to an end effector, but the canonical
link for the gripper is not at the weld point (or it has multiple potential
weld points), you must duplicate this information in the top-level.

For including models, it is nice to have access to other model types, e.g.
including a custom model specified as a `*.yaml` or connecting to some other
legacy format. Generally, the interface between models only really needs access
to explicit and implicit frames (for welding joints, attaching sensors, etc.).
The present implementation of `//include` requires that SDFormat know
*everything* about the included model, whereas a user cound instead provide an
adapter to provide the minimal information necessary for assembly.

## Proposed changes

### 0 Terminology

* **interface elements** - Minimum elements necessary for assembling models.
These should really be model names and frames (implicit and explicit), and
transitively, the links they refer to.

### 1 Nesting and Encapsulation

#### 1.1 Standalone Components Individual Files

To enable "bottom-up" assemblies (or piece part design) to maximize modularity,
individual files should generally be viewed as standalone: all references in
the file should only refer to items "under" that file (e.g. links, joints, or
frames defined in the file, or links, joints, or frames defined in included
files).

Individual files should *not* be able to have a deferred reference to
something that isn't defined in that file (like Python modules).

In conjunction with the [pose frame semantics proposal](/tutorials?tut=pose_frame_semantics_proposal),
all initially specified `//pose` elements within a file should be rigidly
related to one another, and should not be able to have their poses mutated by
an external site (in order to simplify pose resolution).

#### 1.2 Interface Elements

Assemblies require interfaces, and those interfaces should be able to conceal internal details while still providing useful abstractions, such as welding
points when swapping out components.

As such, you should consider levels of abstraction and interface. In this case,
frames will be in the main interface point.

This permits a user to specify a mounting point as an interface, move that
within the model, and not have to update other downstream components.

##### 1.2.2 `//joint/parent` and `//joint/child` become frames, not just links

Assuming that assembly happens either posturing / attaching frames
(`//pose/@relative_to` and `//frame/@attached_to`) or joints (`//joint`), then
it would be ideal to have all of these items refer to frames (implicit and
explicit). `//pose` and `//frame` already refer to frames, so making joints
to refer to frames would also simplify things.

This allows easier swapping out of components.

**WARNING**: This would motivate preserving frames through saving SDFormat
files via Gazebo / libsdformat, esp. if they become the "API".

##### 1.2.3 Frame Naming Suggestion

The frames in a model (implicit and explicit) should be considered the public
"API" of the model.

If an intermediate element needs to be used, but should not be public, consider
prefixing the names wiht a single `_` to indicate they should be private, like
in Python.

#### 1.3 Name Scoping and Cross-Referencing

#### 1.3.1 Reserved Delimiter Token `::`

The delimiter token `::` is intended to form scope, and thus should be
reserved. No element names can be defined using this token.

*Alternatives Considered*: It would be more ideal to use `/` as the delimiter
token, more in line with ROS. However, for legacy with existing Gazebo usages,
SDFormat will stick with `::` for now.

#### 1.3.2 Reference Types

There are only two types of references allowed: **relative references**
(e.g. `mid_link`, `mid_model::mid_link`) or **absolute references**
(e.g. `::top_model::mid_model::mid_link`).

Relative references can have no delimiters (immediate neighbors) or it can have
delimiters, but the it can only reference *down* into nested models; relative
references can not be made *up* into parent models. The only way to go up is to
use absolute references.

References contexts are defined by the files they belong in, as well as the element composition. Absolute references are anchored according to the current
*file* and its root element. If a document is a model, then the root model is
*not* part of the absolute scope. However, if the document is a world file,
then any root model is part of the absolute scope.

This convention is chosen to be a conservative start and avoid the need for
shadowing logic or trying to figure out relative reference syntax using `::`.

The following inline examples have repeated elements just to show different
flavors of the same expression, or invalid versions of an given expression. For
a file whose root is a model:

~~~xml
<sdf version="1.8">
  <model name="top_model">
    <frame name="top_frame"/>

    <link name="top_link">
      <pose relative_to="top_frame"/>  <!-- VALID -->
      <pose relative_to="::top_frame"/>  <!-- VALID: Same as above -->
      <pose relative_to="::top_model::top_frame"/>  <!-- ERROR: Root model not part of scope. -->
    </link>

    <model name="mid_model">
      <link name="mid_link">
        <pose relative_to="::top_link"/>  <!-- VALID. -->
        <pose relative_to="top_link"/>  <!-- ERROR: Shadowing. -->
      </link>

      <model name="bottom_model">
        <link name="bottom_link">
          <pose relative_to="::mid_model::mid_link"/>  <!-- VALID -->
          <pose relative_to="mid_model::mid_link"/>  <!-- ERROR: Shadowing. -->
          <pose relative_to="mid_link"/>  <!-- ERROR: Shadowing. -->
        </link>
      </model>

      <frame name="mid_to_bottom" attached_to="bottom_model::bottom_link"/>  <!-- VALID -->
      <frame name="mid_to_bottom" attached_to="::mid_model::bottom_model::bottom_link"/>  <!-- VALID -->
      <frame name="mid_to_bottom" attached_to="bottom_link"/>  <!-- ERROR: Bad scope. -->
      <frame name="mid_to_bottom" attached_to="mid_model::bottom_model::bottom_link"/>  <!-- ERROR: Shadowing. -->
    </model>
  </model>
</sdf>
~~~

For a world file:

~~~xml
<sdf version="1.8">
  <world name="simple_world">

    <frame name="world_frame"/>

    <model name="top_model">
      <frame name="top_frame" relative_to="world_frame"/> <!-- VALID / ERROR ???? -->
      <!-- Question: Is the above valid or an error?
      With current setup, can `//model/link/pose` or `//model/joint/pose` have
      `@relative_to` refer to a world frame? Or is it only via `//model/pose`?
      This may make backwards compatibility hard...
      -->
      <frame name="top_frame" relative_to="::world_frame"/>  <!-- VALID -->
      <frame name="top_frame" relative_to="::simple_world::world_frame"/>  <!-- ERROR: Root world not part of scope. -->

      <link name="top_link">
        <pose relative_to="top_frame"/>  <!-- VALID -->
        <pose relative_to="::top_model::top_frame"/>  <!-- VALID -->
        <pose relative_to="::top_frame"/>  <!-- ERROR: Bad scope. -->
        <pose relative_to="top_model::top_frame"/>  <!-- ERROR: Shadowing. -->
      </link>
    </model>

    <joint name="top_model_weld">

      <parent>world</parent>  <!-- VALID -->
      <parent>world_frame</parent>  <!-- VALID -->
      <parent>::world_frame</parent>  <!-- VALID -->
      <parent>::simple_world::world_frame</parent>  <!-- ERROR -->

      <child>top_model::top_link</child>  <!-- VALID -->
      <child>::top_model::top_link</child>  <!-- VALID -->
      <child>top_link</child>  <!-- INVALID -->

    </joint>

  </world>
</sdf>
~~~

##### 1.3.2 Scope of Interface Elements

To avoid the complication of inter-element ambiguity, or multiple levels of
scope resolution, all interface elements within an immediate `//model` should be
referencable by only *one* level of nesting. If there are two levels of nesting
for a name (e.g. `a::b::c`), then `a` and `b` will be models, and `c` will most
likely be a frame. `b` will never be a link or a visual or anything else.

##### 1.3.3 No Implicit Name References

There is no "implicit name" resolution; if a name does not exist in a
requested scope, it is an error.

###### 1.3.3.1 Model Frame Cross-References

For a model named `{name}`, the only way to refer to the model frame is by
specifying `{name}::__model__`. Referring to `{name}` is invalid.

This implies that for a name like `a::b`, `a` is a model, `b` is a frame. For a
name like `a::b::c`, `a` and `b` are models, and `c` is the frame.

##### 1.3.4 Cross-Referencing Rules

Cross-referencing is only allowed between elements *in or under the same file*.

Scopes via composition are defined by the *instantiated* (or overriden) model
name, not the model name specified by the included file.

Only the following attributes / properties are presently permitted to cross
boundaries, either up or down depending on the file:

* `//joint/parent` and `//joint/child`
* `//frame/@attached_to`
* `//model/@canonical_link` can *only* cross model boundaries down (referring
to children), not up (referring to parents).

As a note, the default `//pose/@relative_to` will always refer to the
immediate parent `//model`.

#### 1.4 `//include` Semantics

##### 1.4.1 Permissible `//include` Elements

`//include` can *only* be used to include models, not worlds.

Only the following elements can overridden in a model via an
`/include`:

* `//include/name`
* `//include/pose`

Only the following elements can be added to a model via an `/include`:

* `//include/joint`
* `//include/frame`

<!-- Permit others? -->

##### 1.4.2 `//include/pose`

`//include/pose` dictactes how to override the model's pose defined via
`//model/pose`. When this is specified, *all* poses in the included model will
be rigidly transformed by the given pose (regardless of joint fixtures),
relative to the included `//model/@canonical_link`.

The scope of `//include/pose` should be evaluated with respect the enclosing
scope of the `//include` tag, *not* the scope of the included model. This does
mean that the semantics are slightly different from what would be used in a
nest model pose, e.g. `//model/model/pose`.

For example:

~~~xml
<model name="super_model">
  <frame name="super_frame"/>

  <include>
    <uri>file://mug.sdf</uri>

    <pose relative_to="super_frame">  <!-- VALID -->
    <pose relative_to="::super_frame">  <!-- VALID -->
    <pose relative_to="mug::super_frame">  <!-- ERROR -->
    <pose relative_to="::mug::super_frame">  <!-- ERROR -->
  </include>
</model>
~~~

##### 1.4.3 Placement frame: `//include/@model_pose_frame`

It is useful to place an object using a semantic relationship between two
objects, e.g. place the bottom-center of a mug upright on the top-center of a
table. To do this, you can the specify the frame for which the `//include/pose`
should change.

This can be achieved by specifying `//include/@model_pose_frame`. If this
attribute is specfied, then `//include/pose` *must* be specified, as
any information in the included `//model/pose` will no longer be relevant.

As an example:

~~~xml
<model name="super_model">
  <include model_pose_frame="bottom_left_leg">
    <name>table</name>
    <uri>file://table.sdf</uri>
    <pose/>
  </include>
  <include model_pose_frame="bottom_center">
    <name>mug</name>
    <uri>file://mug.sdf</uri>
    <pose relative_to="table::top_center"/>
  </include>
</model>
~~~

##### 1.4.4 Permit files directly in `//include/uri`

Specifying a directory permits usage of `model.config` manifests, which permits
better compatibilty for a model when being loaded by software with different
SDFormat specification support. However, it then requires overhead that may not
matter for some applications.

This proposal suggests that `//include/uri` permits referencing files directly.
If a file is relative, then it should be evaluated next to the file itself.

**WARNING**: In general, it is suggested to use `package://` URIs for ROS
models, and `model://` URIs for Gazebo models.

#### 1.5 Minimal Interface Types for Non-SDFormat Models

**TODO**: Add this in.

### Examples

#### 1 Weld Arm and Gripper

This example shows composing a simple "arm" (base frame `A`) with a simple
"gripper" (base frame `G`) by defining a connecting frame / mounting point on
both (`Ca` on the arm, `Cg` on the gripper), and then posturing and affixing
the frames.

##### 1.1 Positive Example

The following is intended to work:

~~~xml
<!-- arm.sdf -->
<model name="arm">
    <link name="body"/>

    <frame name="gripper_mount" attached_to="body">
      <pose>{X_ACa}</pose>
    </frame>
</model>
~~~

~~~xml
<!-- gripper.sdf -->
<model name="gripper">
  <link name="body"/>

  <frame name="mount_point" attached_to="body">
    <pose>{X_GCg}</pose>
  </frame>
</model>
~~~

~~~xml
<!-- arm_and_gripper.sdf -->
<model name="arm_and_gripper">
  <include file="arm.sdf">
    <uri>arm.sdf</uri>
  </include>
  <include model_pose_frame="mount_point">
    <uri>gripper.sdf</uri>
    <!-- Place model to make Cm and Ca coincide on both models. -->
    <model_pose_frame>mount_point</model_pose_frame>
    <pose relative_to="arm::gripper_mount"/>
  </include>

  <!-- Physically affix the two frames (i.e. their attached links). -->
  <joint name="weld" type="fixed">
    <!-- N.B. This joint's origin will be defined at X_MCg == X_MCa -->
    <parent>arm::gripper_mount</parent>
    <child>gripper::mount_point</child>
  </joint>
</model>
~~~

Note how there are no repetitions / inversions of values like `{X_ACa}` and
`{X_GCg}`.

##### 1.2 Negative Example

You cannot achieve the above by defining the weld in the gripper itself, e.g. by modifying the gripper and compoisiton file:

~~~xml
<!-- BAD_gripper_with_weld.sdf -->
<model name="gripper">
  <pose>{X_AG}</pose>
  <link name="gripper">
  <joint name="weld" type="fixed">
    <parent>::arm::body</parent> <!-- ERROR: Does not exist in this file -->
    <child>body</child>
  </joint>
</model>
~~~

~~~xml
<!-- BAD_arm_and_gripper.sdf -->
<model name="arm_and_gripper">
  <include>
    <uri>arm.sdf</uri>
  </include>
  <include>
    <uri>gripper_with_weld.sdf</uri>
  </include>
</model>
~~~

This implies that for scoping, it is *extremely* important that the parser to
know that it's working with a single model file.

#### 2 Robot Arm with Gripper and Intermediate Flanges

This is the same as above, but we define an intermediate model with a flange.

Frames:

* `A` - arm
* `F` - flange origin
* `G` - gripper physical origin
* `Da` - connecting frame on arm (for flange)
* `Df` - connecting frame on flange (for arm)
* `Cf` - connecting frame (mount) on flange (for gripper)
* `Cg` - connecting frame (mount) on gripper (for flange)

Files:

~~~xml
<!-- arm.sdf -->
<model name="arm">
  <link name="link"/>
  <frame name="flange_mount">
    <pose frame="link">{X_ADa}</pose>
  </frame>
</model>
~~~

~~~xml
<!-- flange_electric.sdf -->
<model name="flange_electric">
  <link name="body"/>

  <frame name="mount">
    <pose frame="body">{X_FDf}</pose>
  </frame>

  <frame name="gripper_mount">
    <pose frame="body">{X_FCf}</pose>
  </frame>
</model>
~~~

~~~xml
<!-- flange_pneumatic.sdf -->
<model name="flange_pneumatic">
  <link name="body"/>

  <frame name="mount">
    <pose frame="body">{X_FDf}</pose>
  </frame>

  <frame name="gripper_mount">
    <pose frame="body">{X_FCf}</pose>
  </frame>
</model>
~~~

~~~xml
<!-- gripper.sdf -->
<model name="gripper">
  <link name="gripper"/>

  <frame name="mount">
      <pose>{X_GCg}</pose>
  </frame>
</model>
~~~

With the proposed nesting, defining `R1` as `robot_1`s model frame, and `R2`
`robot_2`s model frame:

```xml
<model name="super_armio_bros">
<!-- N.B. This could also be defined as a //world element. -->

  <!-- Arm + Electric Flange + Gripper -->
  <model name="robot_1">
    <include file="arm.sdf">
      <name>arm</name>
      <uri>file://arm.sdf</uri>
      <pose>{X_MR1}</pose>
    </include>
    <include model_pose_frame="mount">
      <name>flange</name>
      <uri>file://flange_electric.sdf</uri>
      <pose relative_to="arm::flange_mount"/>
    </include>
    <joint type="weld">
      <parent>arm::flange_mount</parent>
      <child>flange::mount</child>
    </joint>
    <include model_pose_frame="mount">
      <uri>file://gripper.sdf</uri>
      <pose relative_to="flange::gripper_mount"/>
    </include>
    <joint type="weld">
      <parent>flange::gripper_mount</parent>
      <child>gripper::mount</child>
    </joint>
  </model>

  <!-- Arm + Pneumatic Flange + Gripper -->
  <model name="robot_2">
    <include file="arm.sdf">
      <name>arm</name>
      <uri>file://arm.sdf</uri>
      <pose relative_to="::robot_1::__model__">{X_R1R2}</pose>
    </include>
    <include model_pose_frame="mount">
      <name>flange</name>
      <uri>file://flange_pneumatic.sdf</uri>
      <pose relative_to="arm::flange_mount"/>
    </include>
    <joint type="weld">
      <parent>arm::flange_mount</parent>
      <child>flange::mount</child>
    </joint>
    <include model_pose_frame="mount">
      <uri>file://gripper.sdf</uri>
      <pose relative_to="flange::gripper_mount"/>
    </include>
    <joint type="weld">
      <parent>flange::gripper_mount</parent>
      <child>gripper::mount</child>
    </joint>
  </model>

</model>
```
