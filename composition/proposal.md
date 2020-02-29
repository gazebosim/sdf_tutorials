# Model composition: Proposed behavior

* **Authors**:
Eric Cousineau `<eric.cousineau@tri.global>`,
Steven Peters `<scpeters@osrfoundation.org>`,
Addisu Taddese  `<addisu@openrobotics.org>`
* **Status**: Draft
* **SDFormat Version**: 1.8
* **`libsdformat` Version**: 10.0

## Introduction

This proposal suggests changes to the semantics of composition, targetting
SDFormat 1.8. As discussed in the
[legacy behavior tutorial](/tutorials?tut=composition), SDFormat models are the
fundamental building blocks of a world in SDFormat. As of SDFormat 1.4, models
that are defined in separate files can be added into a world multiple times
with distinct name and pose using the `<include>` tag. As of SDFormat 1.5,
the `<include>` tag can be used within a model to construct a composite that
combines other models from separate files.

The existing behavior enables a basic form of composition,
but it does not have many explicit
provisions for encapsulation and modularity or the ability to include
models specified in a format other than SDFormat.
The changes proposed here intend to improve encapsulation and
semantics for assembly, and to define an interface for using the `<include>`
tag with models specified in other formats.

## Document summary

The proposal includes the following sections:

*   Motivation: background and rationale.
*   Proposed changes: Each additon / subtraction to SDFormat and `libsdformat`.
*   Examples: Long form code samples.

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
[`__model__` frame](/tutorials?tut=pose_frame_semantics_proposal#2-model-frame-and-canonical-link).
Therfore, if you wanted to weld a gripper to an end effector, but the canonical
link for the gripper is not at the weld point (or it has multiple potential
weld points), you must duplicate this information in the top-level.

For including models, it is nice to have access to other model types, e.g.
including a custom model specified as a `*.yaml` or connecting to some other
legacy format. Generally, the interface between models only really needs access
to explicit and implicit frames (for welding joints, attaching sensors, etc.).
The present implementation of `//include` requires that SDFormat know
*everything* about the included model, whereas a user could instead provide an
adapter to provide the minimal information necessary for assembly.

### Scope of the proposal

There are existing solutions to handle composition. Generally, those
solutions are some form of text / XML generation (e.g. `xacro`, or Python /
Ruby scripts). These methods can provide for more advanced things, like
paramterization, conditional branching, looping, working up towards Turing
completeness. However, these methods may not have a firm grasp of the semantics
of the data they are manipulating, and thus can undermine encapsulation, and
can add a layer of complexity when errors (syntatic, semantic, or design) are
introduced.

This proposal is only for the process of incorporating existing models and
posturing them in a physical fashion, but it is not to be used for mutating
those models internally (adding, changing, or removing elements or attributes
internal to the model). Those use cases may be more complex, and thus it is
left to downstream usages to decide whether to use
[Custom Elements](/tutorials?tut=custom_elements_attributes_proposal), or use
text processing as mention above.

As the focus of this proposal is physical composition (e.g. elements that can
generally exist without mutual exclusion), this proposal will not tackle
including sub-properties of `/world` elements, given that two worlds may not be
able to coexist in the same space (e.g. they may have different global
properties).

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
related to one another, and should not be able to have their relative
zero-configuration poses mutated after being included (in order to simplify
pose resolution).

#### 1.2 Interface Elements

##### 1.2.1 Frames as primary interface elements

Assemblies require interfaces, and those interfaces should be able to conceal
internal details while still providing useful abstractions, such as welding
points when swapping out components.

As such, you should consider levels of abstraction and interface. In this case,
frames will be in the main interface point.

This permits a user to specify a mounting point as an interface, move that
within the model, and not have to update other downstream components.

##### 1.2.2 `//joint/parent` and `//joint/child` refer to frames, not just links

Assuming that assembly happens either by posturing / attaching frames
(`//pose/@relative_to` and `//frame/@attached_to`) or joint connections
(`//joint/parent` and `//joint/child`), then it would be ideal to have all of
these items refer to frames (implicit and explicit). `//pose` and `//frame`
already refer to frames, so making joints to refer to frames would also
simplify things.

The implementation for this should generally stay the same, with the only
change being that the frame's underlying link would be referred to, and the
default `//joint/pose/@relative_to` would end up incorporting the child frame's
pose.

This allows easier swapping out of components.

**WARNING**: This would motivate preserving frames through saving SDFormat
files via Gazebo / libsdformat, esp. if they become the "API".

##### 1.2.3 Frame, Link, and Joint Naming Suggestion

The frames, links, and joints in a model (implicit and explicit) should be
considered the public "API" of the model.

If an intermediate element needs to be used, but should not be public, consider
prefixing the names with a single `_` to indicate they should be private, like
in Python.

#### 1.3 Name Scoping and Cross-Referencing

#### 1.3.1 Reserved Delimiter Token `::`

The delimiter token `::` is intended to form scope, and thus should be
reserved. No element names can be defined using this token.

*Alternatives Considered*: It would be more ideal to use `/` as the delimiter
token, more in line with ROS. However, for legacy with existing Gazebo usages,
SDFormat will stick with `::` for now.

#### 1.3.2 Reference Types

As a conservative initial behavior, only **relative references** should be
permitted. Those can either go *down* into nested models (e.g. `mid_link`,
`mid_model::mid_link`), or can go *up*  using the `^` symbol (e.g.
`^parent_frame`, `^parent_model::mid_link`).

As a conservative initial behavior, shadowing will not be permitted. This means
that frames may only be referenced within their own scope, and cannot be
referenced implicity in nested scopes. This ensures that each model is an
explicit unit; any dependencies external to the
model (but within the same file) will be visible with `^`. Additionally, it avoids potential ambiguities (e.g. a parent frame with the same name as a
sibling frame).

The parents and children of elements are defined by the model nesting structure
(e.g. a model and its child models), not by physical topology (joint
parent and child links). To be more concise, the `^` token will give you access
to the sibling elements of the model from which `^` is used.

To enforce encapsulation, relative references are
bounded according to the current *file* and its root element;
e.g. the root element of a document (a model or world) cannot access parent
elements.

For simplicity, only **one** upwards reference can be made: `^parent` is valid,
but `^^grandparent` is not. If such a connection is necessary, use intermediate
frames.

These conventions are chosen to be a conservative start and avoid the need for
shadowing / recursion logic. The relative nature of referencing is chosen to
permit easier manual editing / composition of documents.

The following inline examples have repeated elements just to show different
flavors of the same expression, or invalid versions of an given expression. For
a file whose root is a model:

~~~xml
<sdf version="1.8">
  <model name="top_model">
    <frame name="top_frame"/>

    <link name="top_link">
      <pose relative_to="top_frame"/>  <!-- VALID -->
      <pose relative_to="^some_unknown_frame"/>  <!-- ERROR: Violates encapsulation. -->
      <pose relative_to="^top_model::top_frame"/>  <!-- ERROR: Violates encapsulation. -->
    </link>

    <model name="mid_model">
      <link name="mid_link">
        <pose relative_to="^top_link"/>  <!-- VALID. -->
        <pose relative_to="^^some_unknown_frame"/>  <!-- ERROR: Cannot use ^^. -->
        <pose relative_to="top_link"/>  <!-- ERROR: Shadowing is invalid. -->
      </link>

      <model name="bottom_model">
        <link name="bottom_link">
          <pose relative_to="^mid_link"/>  <!-- VALID -->
          <pose relative_to="^^top_frame"/>  <!-- ERROR: Cannot use ^^. Use `^mid_link` as intermediate instead. -->
          <pose relative_to="mid_model::mid_link"/>  <!-- ERROR: Shadowing. -->
          <pose relative_to="mid_link"/>  <!-- ERROR: Shadowing. -->
          <pose relative
        </link>

        <frame name="bottom_frame" attached_to="bottom_link"/>  <!-- VALID -->
        <frame name="bottom_frame" attached_to="^bottom_model::bottom_link"/>  <!-- VALID, but not recommended -->
      </model>

      <!-- Because shadowing is disallowed, the reference to `mid_model` within
      `bottom_model_2` is explicit and can only ever refer to one thing. -->
      <model name="bottom_model_2">
        <model name="mid_model">
          <link name="mid_link"/>
        </model>

        <link name="bottom_link">
          <pose relative_to="mid_model::mid_link"/>  <!-- VALID -->
        </model>
      </model>

      <frame name="mid_to_bottom" attached_to="bottom_model::bottom_link"/>  <!-- VALID -->
      <frame name="mid_to_bottom" attached_to="^mid_model::bottom_model::bottom_link"/>  <!-- VALID, but not recommended -->
      <frame name="mid_to_bottom" attached_to="bottom_link"/>  <!-- ERROR: Bad scope. -->
      <frame name="mid_to_bottom" attached_to="mid_model::bottom_model::bottom_link"/>  <!-- ERROR: Shadowing is invalid. -->
    </model>
  </model>
</sdf>
~~~

For a world file:

~~~xml
<sdf version="1.8">
  <world name="simple_world">

    <frame name="world_frame"/>

    <frame name="world_scope_frame" attached_to="world_frame"/>  <!-- VALID -->
    <frame name="world_scope_frame" attached_to="^simple_world::world_frame"/>  <!-- ERROR: Violates encapsulation. -->

    <model name="top_model">
      <frame name="top_frame" attached_to="^world_frame"/>  <!-- VALID -->
      <frame name="top_frame" attached_to="world_frame"/> <!-- ERROR: Shadowing is invalid.-->
      <frame name="top_frame" attached_to="^^simple_world::world_frame"/>  <!-- ERROR: Cannot go up twice. -->

      <link name="top_link">
        <pose relative_to="top_frame"/>  <!-- VALID -->
        <pose relative_to="^top_model::top_frame"/>  <!-- VALID, but not recommended -->
        <pose relative_to="^top_frame"/>  <!-- ERROR: Bad scope. -->
        <pose relative_to="top_model::top_frame"/>  <!-- ERROR: Shadowing is invalid. -->
      </link>
    </model>

    <joint name="top_model_weld">

      <parent>world</parent>  <!-- VALID -->
      <parent>world_frame</parent>  <!-- VALID -->

      <child>top_model::top_link</child>  <!-- VALID -->
      <child>top_link</child>  <!-- INVALID -->

    </joint>

  </world>
</sdf>
~~~

**Alternatives Considered for Reference Types**

It was considered to not permit upwards references and only use downward or
absolute references. This looks a bit better syntactically, but makes the
references more dependent on the full context of a file. Relative references
are more local.

**Alternatives Considered for Reference Syntax**

* Use `/` instead of `::`, and permit `../` for upwards references.
    * This looks a bit more like a filesystem (more relevant to these semantics). However, there is inertia due to Gazebo's usage of `::`
    for composition, in both SDFormat files and for models (and IPC channels /
    topics in general).
* Upwards references:
    * `..::parent` - hard to parse, but would be better for changing the
    separator later.
    * `^::parent` - perhaps better?
    * `^:parent` - Also better, maybe?

##### 1.3.2 Scope of Interface Elements

To avoid the complication of inter-element ambiguity, or multiple levels of
scope resolution, all interface elements within an immediate `//model` should be
referencable by only *one* level of nesting. If there are two levels of nesting
for a name (e.g. `a::b::c`), then `a` and `b` will be models, and `c` will most
likely be a frame. `b` will never be a link or a visual or anything else.

##### 1.3.3 No Implicit Name References

There is no "implicit name" resolution; if a name does not exist in a
requested scope, it is an error. Additionally, no frame should ever have an empty
name, e.g. `{name}::` should never be a valid reference.

###### 1.3.3.1 Model Frame Cross-References

For a model named `{name}`, the only way to refer to the model frame is by
specifying `{name}::__model__`. Referring to `{name}` is invalid.

This implies that for a name like `a::b`, `a` is a model, `b` is a frame. For a
name like `a::b::c`, `a` and `b` are models, and `c` is the frame.

##### 1.3.4 Cross-Referencing Rules

Cross-referencing is only allowed between elements *in or under the same file*.

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

###### 1.4.2 `//include/name` and Cross-References

Scopes via composition are defined by the *instantiated* model name
(`//include/name`), not the *include-file-specified* model name
(`//model/@name`).

This should not be an issue, as there should be no elements within a file that
explicitly depend on `//model@name`.

As an example:

~~~xml
<!-- mid_model.sdf -->
<sdf version="1.8">
  <model name="mid_model">
    <link name="mid_link"/>
  </model>
</sdf>

<!-- top_model.sdf -->
<sdf version="1.8">
  <model name="top_model">
    
    <include>
      <uri>file://mid_model.sdf</uri>
      <name>my_custom_name</name>
    </include>

    <frame name="top_to_mid" attached_to="my_custom_name::mid_link"/>  <!-- VALID -->
    <frame name="top_to_mid" attached_to="mid_model::mid_link"/> <!-- INVALID: Name does not exist in this context. -->

  </model>
</sdf>
~~~

##### 1.4.3 `//include/pose`

`//include/pose` dictates how to override the model's pose defined via
`//model/pose`. When this is specified, *all* poses in the included model will
be rigidly transformed by the given pose (regardless of joint fixtures),
relative to the included `//model/@canonical_link`.

The scope of `//include/pose` should be evaluated with respect the enclosing
scope of the `//include` tag, *not* the scope of the included model. This does
mean that the semantics are slightly different from what would be used in a
nested model pose, e.g. `//model/model/pose`.

For example:

~~~xml
<model name="super_model">
  <frame name="super_frame"/>

  <include>
    <uri>file://mug.sdf</uri>

    <pose relative_to="super_frame">  <!-- VALID -->
    <pose relative_to="^super_frame">  <!-- ERROR: Violates encapsulation. -->
    <pose relative_to="mug::super_frame">  <!-- ERROR: Bad frame. -->
  </include>
</model>
~~~

##### 1.4.4 Placement frame: `//include/@model_pose_frame`

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

##### 1.4.5 Permit files directly in `//include/uri`

Specifying a directory permits usage of `model.config` manifests, which permits
better compatibilty for a model when being loaded by software with different
SDFormat specification support. However, it then requires overhead that may not
matter for some applications.

This proposal suggests that `//include/uri` permits referencing files directly.
If a file is relative, then it should be evaluated next to the file itself.

**WARNING**: In general, it is suggested to use `package://` URIs for ROS
models, and `model://` URIs for Gazebo models.

#### 1.5 Minimal Interface Types for Non-SDFormat Models

**TODO(eric.cousineau)**: Add this in a follow-up PR.

#### 1.6 Proposed Parsing Stages

**TODO(eric.cousineau)**: Add this in a follow-up PR.

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

You cannot achieve the above by defining the weld in the gripper itself, e.g. by modifying the gripper and composition file:

~~~xml
<!-- BAD_gripper_with_weld.sdf -->
<model name="gripper">
  <pose>{X_AG}</pose>
  <link name="gripper">
  <joint name="weld" type="fixed">
    <parent>^arm::body</parent> <!-- ERROR: Does not exist in this file -->
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
      <pose relative_to="^robot_1::__model__">{X_R1R2}</pose>
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
