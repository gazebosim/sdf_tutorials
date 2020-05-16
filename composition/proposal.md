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

As of SDFormat 1.7, nesting a model with a `//model/include` tag has different
behavior than direct nesting with `//model/model`,
part of the reason being that nesting directly with `//model/model` is not yet
implemented in the current `libsdformat` (9.1.0). Nesting of models generally
implies that the elements can be referenced via a form of scope, such as
`{scope}::{name}`. However, `::` is not a special token and thus can be
used to create "false" hierarchy or potential name collisions. Additionally, there is
no way for elements within the same file to refer "up" to another element, e.g. with in a robot assembly, adding a weld between a gripper and an arm when the
two are sibiling models.

For posturing an included model, there is no means by which the user can
specify which included frame to posture via `//include/pose`. The target frame
to move currently can only be the
[`__model__` frame](/tutorials?tut=pose_frame_semantics_proposal#2-model-frame-and-canonical-link).
Therfore, if you wanted to weld a gripper to an end effector, but the canonical
link for the gripper is not at the weld point (or it has multiple potential
weld points), you must duplicate this pose information in the top-level.

For including models, it is nice to have access to other model types, e.g.
including a custom model specified as a `*.yaml` or connecting to some other
legacy format. Generally, the interface between models only really needs access
to explicit and implicit frames (for welding joints, attaching sensors, etc.).
The present implementation of `//include` requires that SDFormat know
*everything* about the included model, whereas a user could instead provide an
adapter to provide the minimal information necessary for assembly.

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
changes being that the joint's parent and child links are the links to which
the parent and child frames are attached, and that the implicit joint frame
is attached to the joint's child frame. This implies that the
default value of `//joint/pose/@relative_to` is relative to the child frame's
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

##### 1.2.4 Reintroduce `//world/joint`

Given that assembly is achieved using joints, both inside `//model` and
`//world` elements, `//world` elements should be able to provide a mechanism to
make assemblies without having to make a wrapping `//model`.

To this end, the next specification of SDFormat should reintroduce
`//world/joint`, but ensure that it is explicitly supported in both
specification and software.

#### 1.3 Name Scoping and Cross-Referencing

#### 1.3.1 Reserved Delimiter Token `::`

The delimiter token `::` is intended to form scope, and thus should be
reserved. No element names can be defined using this token.

*Alternatives Considered*: It would be more ideal to use `/` as the delimiter
token, more in line with ROS. However, for legacy with existing Gazebo usages,
SDFormat will stick with `::` for now.

#### 1.3.2 Reference Types

As a conservative initial behavior, only **relative references** should be
permitted. Those can only go *down* into the current or nested models (e.g.
`mid_link`, `mid_model::mid_link`).

As a conservative initial behavior, shadowing will not be permitted. This means
that frames may only be referenced within their own scope, and cannot be
referenced implicity in nested scopes. This ensures that each model is an
explicit unit; any dependencies external to the
model (but within the same file) will not be visible. Additionally, it avoids potential ambiguities (e.g. a parent frame with the same name as a
sibling frame).

The parents and children of elements are defined by the model nesting structure
(e.g. a model and its child models), not by physical topology (joint
parent and child links).

As a side effect, encapsulation is enforced: relative references are
bounded according to the current *file*.

These conventions are chosen to be a conservative start and avoid the need for
shadowing / recursion logic. The relative nature of referencing is chosen to
permit easier manual editing / composition of documents.

The following inline examples have repeated elements just to show different
flavors of the same expression, or invalid versions of a given expression. For
a file whose root is a model:

~~~xml
<sdf version="1.8">
  <model name="top_model">
    <frame name="top_frame"/>

    <link name="top_link">
      <pose relative_to="top_frame"/>  <!-- VALID -->
      <pose relative_to="some_unknown_frame"/>  <!-- ERROR: Violates encapsulation. -->
      <pose relative_to="top_model::top_frame"/>  <!-- ERROR: Shadowing. Defined in outer scope. -->
    </link>

    <model name="mid_model">
      <pose relative_to="top_link"/>  <!-- VALID. -->

      <link name="mid_link">
        <pose/>  <!-- VALID: Same as relative_to="__model__" -->
        <pose relative_to="top_link"/>  <!-- ERROR: Shadowing. Defined in outer scope. -->
      </link>

      <model name="bottom_model">
        <pose relative_to="mid_link"/>  <!-- VALID -->

        <link name="bottom_link">
          <pose/>  <!-- VALID -->
          <pose relative_to="mid_link"/>  <!-- ERROR: Shadowing. Defined in outer scope. -->
          <pose relative_to="mid_model::mid_link"/>  <!-- ERROR: Shadowing. Defined in outer scope. -->
          <pose relative_to="top_frame"/>  <!-- ERROR: Shadowing. Defined in outer scope. -->
        </link>

        <frame name="bottom_frame" attached_to="bottom_link"/>  <!-- VALID -->
        <frame name="bottom_frame" attached_to="bottom_model::bottom_link"/>  <!-- ERROR: Shadowing. Defined in outer scope. -->
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

      <frame name="mid_to_top" attached_to="top_frame"/>  <!-- ERROR: Shadowing. Defined in outer scope. -->

      <frame name="mid_to_bottom" attached_to="bottom_model::bottom_link"/>  <!-- VALID -->
      <frame name="mid_to_bottom" attached_to="bottom_link"/>  <!-- ERROR: Bad scope. -->
      <frame name="mid_to_bottom" attached_to="mid_model::bottom_model::bottom_link"/>  <!-- ERROR: Shadowing. Defined in outer scope. -->
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
    <frame name="world_scope_frame" attached_to="simple_world::world_frame"/>  <!-- ERROR: Shadowing. Defined in outer scope. -->

    <model name="top_model">
      <pose relative_to="world_frame"/>  <!-- VALID -->

      <frame name="top_frame"/>  <!-- VALID: Same as relative_to="__model__" -->
      <frame name="top_frame" attached_to="world_frame"/>  <!-- ERROR: Shadowing. Defined in outer scope.-->

      <link name="top_link">
        <pose relative_to="top_frame"/>  <!-- VALID -->
        <pose relative_to="top_model::top_frame"/>  <!-- ERROR: Shadowing -->
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

It was considered to only allow downwards references or a single upwards reference using the `^` character. However, there was a lack of a sufficiently
motivating example, so this was removed from the proposal.

It was also considered to not permit upwards references and only use downward or
absolute references. This looks a bit better syntactically, but makes the
references more dependent on the full context of a file. Relative references
are more local.

**Alternatives Considered for Reference Syntax**

* Use `/` instead of `::`, and permit `../` for upwards references.
    * This looks a bit more like a filesystem (more relevant to these semantics). However, there is inertia due to Gazebo's usage of `::`
    for composition, in both SDFormat files and for models (and IPC channels /
    topics in general).
* Upwards references:
    * `^parent` - was original used, but removed from proposal
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

##### 1.3.3 Model Frame References

For a model named `{name}`, model frames can be referenced by their name.
`{name}::__model__` is also valid, but `{name}` is preferred instead.

As shown in the
[World parsing phases](/tutorials?tut=pose_frame_semantics_proposal#2-world)
section, it is possible to refer to a model's frame using `//world/model/@name`
from inside a `//world`.
However, as indicated in the corresponding
[Model section](/tutorials?tut=pose_frame_semantics_proposal#1-model), it is
not currently supported to refer to a nested model's frame using
`//model/model/@name` from inside a `//model`. To that end, the Model parsing
phase for Step 7 should change the description of implicit frames from:

"name of a link, joint, or frame in this model's scope"

to:

"name of a link, joint, frame, or nested model in this model's scope"

##### 1.3.4 Cross-Referencing Rules

Cross-referencing should be the mechanism by which encapsulation is enforced.
Given that references are only allowed to sibling or descendent elements
(e.g. *in or under the same file*), this is implicitly enforced.

#### 1.4 `//include` Semantics

##### 1.4.1 Permissible `//include` Elements

`//include` can *only* be used to include models, not worlds.

Only the following elements can overridden in a model via an
`/include`:

* `//include/name`
* `//include/pose`
* `//include/static` - This is very nuanced. Please see the section below.

Only the following elements can be added to a model via an `/include`:

* `//include/joint`
* `//include/frame`
* `//include/plugin` - note that this is generally Gazebo-specific.

<!-- Permit others? -->

##### 1.4.2 `//include/name` and Cross-References

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

The scope of `//include/pose` should be evaluated with respect to the enclosing
scope of the `//include` tag, *not* the scope of the included model. This means
that the semantics are the same as they would be for a nested model pose
(i.e. `//model/model/pose`).

For example:

~~~xml
<model name="super_model">
  <frame name="super_frame"/>

  <include>
    <uri>file://mug.sdf</uri>

    <pose relative_to="super_frame">  <!-- VALID -->
    <pose relative_to="mug::super_frame">  <!-- ERROR: Bad frame. -->
  </include>
</model>
~~~

##### 1.4.4 Placement frame: `//include/placement_frame`

It is useful to place an object using a semantic relationship between two
objects, e.g. place the bottom-center of a mug upright on the top-center of a
table. To do this, you can the specify the frame for which the `//include/pose`
should change.

This can be achieved by specifying `//include/placement_frame`. If this
element is specfied, then `//include/pose` *must* be specified, as
any information in the included `//model/pose` will no longer be relevant.

As an example:

~~~xml
<model name="super_model">
  <include>
    <name>table</name>
    <uri>file://table.sdf</uri>
    <placement_frame>bottom_left_leg</placement_frame>
    <pose/>
  </include>
  <include>
    <name>mug</name>
    <uri>file://mug.sdf</uri>
    <placement_frame>bottom_center</placement_frame>
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

##### 1.4.6 `//include/static`

**TODO(eric)**: Reconsider this in future iterations of the proposal.

This allows the `//model/static` element to be overridden and will affect
*all* models transitively included via the `//include` element, and can *only*
change values from false to true; `//include/static` can only be false if all
models included via the file are non-static.

This requires `//include/static` to be stored as a tri-state value
(unspecified, true, false).

**Alternatives Considered**:

The value could be overridden to true or false.
However, implementations would have to be careful that `@attached_to` is
properly respected when overriding `//model/static`. Normally, if a model is
static, its frames will be attached to the world. However, if a model is
non-static, the attached-to frame should only be within the model itself (to
observe proper encapsulation).

Additionally, it may not be possible to force certain models to be non-static.
For example, if a `//include/static` is set to false, but the included model is
normally a static model with only frames (which is valid), an error should be
thrown.

It would also generally be *not* suggested to force a static model to be
non-static, as the static model may not be designed for non-static use, and
waste time on debugging.

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

    <frame name="gripper_mount">
      <pose>{X_ACa}</pose>
    </frame>
</model>
~~~

~~~xml
<!-- gripper.sdf -->
<model name="gripper">
  <link name="body"/>

  <frame name="mount_point">
    <pose>{X_GCg}</pose>
  </frame>
</model>
~~~

~~~xml
<!-- arm_and_gripper.sdf -->
<model name="arm_and_gripper">
  <include>
    <uri>file://arm.sdf</uri>
  </include>
  <include>
    <uri>file://gripper.sdf</uri>
    <!-- Place model to make Cg and Ca coincide on both models. -->
    <placement_frame>mount_point</placement_frame>
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
    <parent>arm::body</parent> <!-- ERROR: Does not exist in this file -->
    <child>body</child>
  </joint>
</model>
~~~

~~~xml
<!-- BAD_arm_and_gripper.sdf -->
<model name="arm_and_gripper">
  <include>
    <uri>file://arm.sdf</uri>
  </include>
  <include>
    <uri>file://gripper_with_weld.sdf</uri>
  </include>
</model>
~~~

This implies that for scoping, it is *extremely* important for the parser to
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
    <pose relative_to="link">{X_ADa}</pose>
  </frame>
</model>
~~~

~~~xml
<!-- flange_electric.sdf -->
<model name="flange_electric">
  <link name="body"/>

  <frame name="mount">
    <pose relative_to="body">{X_FDf}</pose>
  </frame>

  <frame name="gripper_mount">
    <pose relative_to="body">{X_FCf}</pose>
  </frame>
</model>
~~~

~~~xml
<!-- flange_pneumatic.sdf -->
<model name="flange_pneumatic">
  <link name="body"/>

  <frame name="mount">
    <pose relative_to="body">{X_FDf}</pose>
  </frame>

  <frame name="gripper_mount">
    <pose relative_to="body">{X_FCf}</pose>
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

With the proposed nesting, defining `M` as the overall model's frame, `R1` as
`robot_1`s model frame, and `R2` as `robot_2`s model frame:

```xml
<model name="super_armio_bros">
  <!-- N.B. This could also be defined as a //world element. -->

  <!-- Arm + Electric Flange + Gripper -->
  <model name="robot_1">
    <pose>{X_MR1}</pose>
    <include file="arm.sdf">
      <name>arm</name>
      <uri>file://arm.sdf</uri>
    </include>
    <include>
      <name>flange</name>
      <uri>file://flange_electric.sdf</uri>
      <placement_frame>mount</placement_frame>
      <pose relative_to="arm::flange_mount"/>
    </include>
    <joint name="weld1" type="fixed">
      <parent>arm::flange_mount</parent>
      <child>flange::mount</child>
    </joint>
    <include>
      <uri>file://gripper.sdf</uri>
      <placement_frame>mount</placement_frame>
      <pose relative_to="flange::gripper_mount"/>
    </include>
    <joint name="weld2" type="fixed">
      <parent>flange::gripper_mount</parent>
      <child>gripper::mount</child>
    </joint>
  </model>

  <!-- Arm + Pneumatic Flange + Gripper -->
  <model name="robot_2">
    <pose relative_to="robot_1">{X_R1R2}</pose>
    <include file="arm.sdf">
      <name>arm</name>
      <uri>file://arm.sdf</uri>
    </include>
    <include>
      <name>flange</name>
      <uri>file://flange_pneumatic.sdf</uri>
      <placement_frame>mount</placement_frame>
      <pose relative_to="arm::flange_mount"/>
    </include>
    <joint name="weld1" type="fixed">
      <parent>arm::flange_mount</parent>
      <child>flange::mount</child>
    </joint>
    <include>
      <uri>file://gripper.sdf</uri>
      <placement_frame>mount</placement_frame>
      <pose relative_to="flange::gripper_mount"/>
    </include>
    <joint name="weld2" type="fixed">
      <parent>flange::gripper_mount</parent>
      <child>gripper::mount</child>
    </joint>
  </model>

</model>
```
