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

It was considered to only allow downwards references or a single upwards reference using the `^` characeter. However, there was a lack of a sufficiently
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

The following sections describe the phases for parsing the kinematics of an
SDFormat 1.8 model and world.
Several of the phases in each section are similar to the phases of parsing in
SDFormat 1.7 in the [Pose Frame Semantics Proposal](/tutorials?tut=pose_frame_semantics_proposal#phases-of-parsing-kinematics).
In phases that differ from SDFormat 1.7, *italics* are used to signal the difference.
For new phases, the ***Title:*** is italicized.

##### 1.6.1 Model

There are eight phases for validating the kinematics data in a model,
and different parts of libsdformat handle differing sets of stages,
returning an error code if errors are found during parsing:

- `sdf::readFile` and `sdf::readString` APIs perform parsing Stage 1
- `sdf::Root::Load` performs most parsing stages, but skips some of the more expensive checks
- `ign sdf --check` performs all parsing stages, including more expensive checks

1.  **XML parsing and schema validation:**
    Parse model file from XML into a tree data structure,
    ensuring that there are no XML syntax errors and that the XML
    data complies with the [schema](http://sdformat.org/schemas/root.xsd).
    Schema `.xsd` files are generated from the `.sdf` specification files
    when building `libsdformat` with the
    [xmlschema.rb script](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/tools/xmlschema.rb).

2.  **Name attribute checking:**
    Check that name attributes are not an empty string `""`,
    that they are not reserved (`__.*__` or `world`) and that sibling
    elements of any type have unique names.
    This includes but is not limited to models, actors, links, joints,
    collisions, visuals, sensors, and lights.
    This step is distinct from validation with the schema because the schema
    only confirms the existence of name attributes, not their content.
    The code paths in `libsdformat9` that implement these checks are summarized below:

    2.1 The `sdf::readFile` and `sdf::readString` APIs check for empty names
        via [Param::SetFromString](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/Param.cc#L452-L457)).

    2.2 The `sdf::Root::Load` API that loads all DOM objects recursively also checks
        any DOM objects with name attributes for reserved names using the helper function
        [isReservedName(const string&)](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/Utils.cc#L25-L33),
        returning a `RESERVED_NAME` error code if one is found
        (see [Frame::Load](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/Frame.cc#L124-L130)
        for an example).
        [Model::Load](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/Model.cc#L208-L212)
        also checks for name collisions in direct children of its `//model` element using the helper function
        [Element::HasUniqueChildNames()](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/Element.cc#L677-L688),
        though it only prints a warning to the console without generating an error code.
        Name uniqueness of sibling `//model/link`, `//model/joint`, and `//model/frame` elements
        is also checked when constructing the
        [FrameAttachedTo](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L223-L282) and
        [PoseRelativeToGraph](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L464-L528) objects
        in [Model::Load](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/Model.cc#L323-L340),
        returning a `DUPLICATE_NAME` error code if non-unique names are detected.

    2.3 The `ign sdf --check` command loads all DOM elements and also
        recursively checks for name uniqueness among all sibling elements
        using the [recursiveSiblingUniqueNames](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/parser.cc#L1633-L1655)
        helper function.

3.  **Joint parent/child name checking:**
    For each joint, check that the parent and child ~~link~~ names are different
    and that each match the name of a sibling *frame* to the joint,
    with the following exception:
    if "world" is specified as a parent ~~link~~ name,
    then the joint is attached to a fixed reference frame.
    In `libsdformat9`, these checks are all performed by the helper function
    [checkJointParentChildLinkNames](https://github.com/osrf/sdformat/blob/4fd00c795bafb6f10a7a36356fe3f61a93c961c8/src/parser.cc#L1814-L1911),
    which is invoked by `ign sdf --check`.
    A subset of these checks are performed by
    [Joint::Load](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/Joint.cc#L199-L213)
    (checking that parent and child ~~link~~ names are different and that
    `world` is not specified as the child ~~link~~ name)
    and [Model::Load](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/Model.cc#L316-L324)
    (for non-static models calling [buildFrameAttachedToGraph](https://github.com/osrf/sdformat/blob/4fd00c795bafb6f10a7a36356fe3f61a93c961c8/src/FrameSemantics.cc#L281-L289),
    which checks that each child ~~link~~ specified by a joint exists as a sibling *frame*
    of that joint).

4.  **Check `//model/@canonical_link` attribute value:**
    For models that are not static,
    if the `//model/@canonical_link` attribute exists and is not an empty
    string `""`, check that the value of the `canonical_link` attribute
    matches the name of a link in this model.
    In `libsdformat9`, this check is performed by
    [buildFrameAttachedToGraph](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L200-L217),
    which is called by
    [Model::Load](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/Model.cc#L316-L324)
    for non-static models.

5.  **Check `//model/frame/@attached_to` attribute values:**
    For each `//model/frame`, if the `attached_to` attribute exists and is not
    an empty string `""`, check that the value of the `attached_to` attribute
    matches the name of a sibling link, joint, or frame.
    The `//frame/@attached_to` value must not match `//frame/@name`,
    as this would cause a graph cycle.
    In `libsdformat9`, these checks are performed by
    [buildFrameAttachedToGraph](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L293-L320),
    which is called by
    [Model::Load](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/Model.cc#L316-L324)
    for non-static models.

6.  **Check `//model/frame/@attached_to` graph:**
    Construct an `attached_to` directed graph for the model with each vertex
    representing a frame (see [buildFrameAttachedToGraph](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L168)
    in `libsdformat9`):

    6.1 Add a vertex for the implicit frame of each link in the model
        (see [FrameSemantics.cc:219-233](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L219-L233)).

    6.2 Add a vertex for the implicit model frame. If the model is not static,
        add an edge connecting this vertex to the
        vertex of the model's canonical link
        (see [FrameSemantics.cc:173-178](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L173-L178)
        and [FrameSemantics.cc:235-239](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L235-L239))

    6.3 Add vertices for the implicit frame of each joint ~~with an edge~~
        ~~connecting from the joint to the vertex of its child *frame*~~
        (see [FrameSemantics.cc:242-257](https://github.com/osrf/sdformat/blob/4fd00c795bafb6f10a7a36356fe3f61a93c961c8/src/FrameSemantics.cc#L242-L257).

    *6.4 Add a vertex to the graph for each `//model/frame`*
        (see [FrameSemantics.cc:259-274](https://github.com/osrf/sdformat/blob/4fd00c795bafb6f10a7a36356fe3f61a93c961c8/src/FrameSemantics.cc#L259-L274)).

    *6.5 For each `//model/joint`, add an edge connecting from the joint to the vertex of its child frame*
        *(see [FrameSemantics.cc:276-292](https://github.com/osrf/sdformat/blob/4fd00c795bafb6f10a7a36356fe3f61a93c961c8/src/FrameSemantics.cc#L276-L292)).*

    6.*6* For each `//model/frame`:

    6.*6.1* If `//model/frame/@attached_to` exists and is not empty,
          add an edge from the added vertex to the vertex
          named in the `//model/frame/@attached_to` attribute
          (see [FrameSemantics.cc:288-322](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L288-L322)).

    6.*6.2* Otherwise (ie. if the `//model/frame/@attached_to` attribute
          does not exist or is an empty string `""`),
          add an edge from the added vertex to the model frame vertex,
          (see [FrameSemantics.cc:288-322](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L288-L322)).

    6.*7* Verify that the graph has no cycles and that by following the directed
        edges, every vertex is connected to a link
        (see [validateFrameAttachedToGraph](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L976-L982)
        which is called by [Model::Load](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/Model.cc#L327-L328)).
        To identify the link to which each frame is attached, start from the
        vertex for that frame, and follow the directed edges until a link
        is reached (see [Frame::ResolveAttachedToBody](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/Frame.cc#L216)*,*
        *[Joint::ResolveChildLink](https://github.com/osrf/sdformat/blob/4fd00c795bafb6f10a7a36356fe3f61a93c961c8/src/Joint.cc#L410-L429),*
        *[Joint::ResolveParentLink](https://github.com/osrf/sdformat/blob/4fd00c795bafb6f10a7a36356fe3f61a93c961c8/src/Joint.cc#L432-L451),*
        and [resolveFrameAttachedToBody in FrameSemantics.cc](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L1158) in `libsdformat9`).

7.  **Check `//pose/@relative_to` attribute values:**
    For each `//pose` that is not `//model/pose` (e.g. `//link/pose`,
    `//joint/pose`, `//frame/pose`, `//collision/pose`, `//light/pose`, etc.),
    if the `relative_to` attribute exists and is not an empty string `""`,
    check that the value of the `relative_to` attribute
    matches the name of a link, joint, or frame in this model's scope.
    In `libsdformat9`, these checks are performed by
    [buildPoseRelativeToGraph](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L556-L658),
    which is called by
    [Model::Load](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/Model.cc#L337-L340).

8.  **Check `//pose/@relative_to` graph:**
    Construct a `relative_to` directed graph for the model with each vertex
    representing a frame
    (see [buildPoseRelativeToGraph](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L435)
    in `libsdformat9`):

    8.1 Add a vertex for the implicit model frame `__model__`
        (see [FrameSemantics.cc:453-458](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L453-L458)).

    8.2 Add vertices for each `//model/link`, `//model/joint`, and
        `//model/frame` (see [FrameSemantics.cc:460-474](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L460-L474),
        [FrameSemantics.cc:483-497](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L483-L497), and
        [FrameSemantics.cc:516-531](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L516-L531)).

    8.3 For each `//model/link`:

    8.3.1 If `//link/pose/@relative_to` exists and is not empty,
          add an edge from the link vertex to the vertex named in
          `//link/pose/@relative_to`
          (see [FrameSemantics.cc:554-575](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L554-L575)).

    8.3.2 Otherwise (ie. if `//link/pose` or `//link/pose/@relative_to` do not
          exist or `//link/pose/@relative_to` is an empty string `""`)
          add an edge from the link vertex to the implicit model frame vertex
          (see [FrameSemantics.cc:476-480](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L476-L480)).

    8.4 For each `//model/joint`:

    8.4.1 If `//joint/pose/@relative_to` exists and is not empty,
          add an edge from the joint vertex to the vertex named in
          `//joint/pose/@relative_to`
          (see [FrameSemantics.cc:572](https://github.com/osrf/sdformat/blob/4fd00c795bafb6f10a7a36356fe3f61a93c961c8/src/FrameSemantics.cc#L572)
          and [FrameSemantics.cc:591-600](https://github.com/osrf/sdformat/blob/4fd00c795bafb6f10a7a36356fe3f61a93c961c8/src/FrameSemantics.cc#L591-L600)).

    8.4.2 Otherwise (ie. if `//joint/pose` or `//joint/pose/@relative_to` do not
          exist or `//joint/pose/@relative_to` is an empty string `""`)
          add an edge from the joint vertex to
          the child *frame* vertex named in `//joint/child`
          (see [FrameSemantics.cc:572-577](https://github.com/osrf/sdformat/blob/4fd00c795bafb6f10a7a36356fe3f61a93c961c8/src/FrameSemantics.cc#L572-L577)
          and [FrameSemantics.cc:591-600](https://github.com/osrf/sdformat/blob/4fd00c795bafb6f10a7a36356fe3f61a93c961c8/src/FrameSemantics.cc#L591-L600)).

    8.5 For each `//model/frame`:

    8.5.1 If `//frame/pose/@relative_to` exists and is not empty,
          add an edge from the frame vertex to the vertex named in
          `//frame/pose/@relative_to`
          (see [FrameSemantics.cc:629](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L629)
          and [FrameSemantics.cc:650-659](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L650-L659)).

    8.5.2 Otherwise if `//frame/@attached_to` exists and is not empty
          (ie. if `//frame/@attached_to` exists and is not an empty string `""`
          and one of the following is true: `//frame/pose` does not exist,
          `//frame/pose/@relative_to` does not exist, or
          `//frame/pose/@relative_to` is an empty string `""`)
          add an edge from the frame vertex to the vertex named in
          `//frame/@attached_to`
          (see [FrameSemantics.cc:635](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L635)
          and [FrameSemantics.cc:650-659](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L650-L659)).

    8.5.3 Otherwise (ie. if neither `//frame/@attached_to` nor
          `//frame/pose/@relative_to` are specified)
          add an edge from the frame vertex to the implicit model frame vertex
          (see [FrameSemantics.cc:533-537](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L533-L537)).

    8.6 Verify that the graph has no cycles and that by following the directed
        edges, every vertex is connected to the implicit model frame
        (see [validatePoseRelativeToGraph](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L1146-L1152)
        which is called by [Model::Load](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/Model.cc#L343-L344)).
        Other poses in the model such as `//collision/pose` and `//light/pose`
        do not need to be checked for cycles since they do not create
        implicitly named frames.
        To find the pose of a DOM object relative-to a named frame in the `PoseRelativeToGraph`,
        use the DOM object's `SemanticPose` function
        (such as [Link::SemanticPose](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/include/sdf/Link.hh#L253-L256))
        and the [SemanticPose::Resolve](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/include/sdf/SemanticPose.hh#L65-L73)
        function.

##### 1.6.2 World

This section describes phases for parsing the kinematics of an SDFormat 1.8 world.
Several of these phases are similar to the phases of parsing an SDFormat 1.7
world in the [Pose Frame Semantics Proposal](/tutorials?tut=pose_frame_semantics_proposal#2-world).
In phases that differ from that document, *italics* are used to signal the difference.
For new phases, the ***Title:*** is italicized.

There are seven phases for validating the kinematics data in a world:

1.  **XML parsing and schema validation:**
    Parse world file from XML into a tree data structure,
    ensuring that there are no XML syntax errors and that the XML
    data complies with the [schema](http://sdformat.org/schemas/root.xsd).
    Schema `.xsd` files are generated from the `.sdf` specification files
    when building `libsdformat` with the
    [xmlschema.rb script](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/tools/xmlschema.rb).

2.  **Name attribute checking:**
    Check that name attributes are not an empty string `""`,
    that they are not reserved (`__.*__` or `world`) and that sibling
    elements of any type have unique names.
    This check can be limited to `//world/model/@name`
    and `//world/frame/@name`
    since other names will be checked in the following step.
    This step is distinct from validation with the schema because the schema
    only confirms the existence of name attributes, not their content.
    The code paths in `libsdformat9` that implement these checks are summarized below:

    2.1 The `sdf::readFile` and `sdf::readString` APIs check for empty names
        via [Param::SetFromString](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/Param.cc#L452-L457)).

    2.2 The `sdf::Root::Load` API that loads all DOM objects recursively also checks
        any DOM objects with name attributes for reserved names using the helper function
        [isReservedName(const string&)](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/Utils.cc#L25-L33),
        returning a `RESERVED_NAME` error code if one is found
        (see [Frame::Load](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/Frame.cc#L124-L130)
        for an example).
        [World::Load](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/World.cc#L258-L262)
        also checks for name collisions in direct children of its `//world` element using the helper function
        [Element::HasUniqueChildNames()](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/Element.cc#L677-L688),
        though it only prints a warning to the console without generating an error code.
        Name uniqueness of sibling `//world/model` and `//world/frame` elements
        is also checked when constructing the
        [FrameAttachedTo](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L358-L382) and
        [PoseRelativeToGraph](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L695-L726) objects
        in [World::Load](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/World.cc#L347-L362),
        returning a `DUPLICATE_NAME` error code if non-unique names are detected.

    2.3 The `ign sdf --check` command loads all DOM elements and also
        recursively checks for name uniqueness among all sibling elements
        using the [recursiveSiblingUniqueNames](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/parser.cc#L1633-L1655)
        helper function.

3.  **Model checking:**
    Check each model according to the eight phases of parsing kinematics of an
    sdf model.

4.  **Check `//world/frame/@attached_to` attribute values:**
    For each `//world/frame`, if the `attached_to` attribute exists and is not
    an empty string `""`, check that the value of the `attached_to` attribute
    matches the name of a sibling model or frame.
    The `//frame/@attached_to` value must not match `//frame/@name`,
    as this would cause a graph cycle.
    In `libsdformat9`, these checks are performed by
    [buildFrameAttachedToGraph](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L393-L427),
    which is called by
    [World::Load](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/World.cc#L345-L348).

5.  **Check `//world/frame/@attached_to` graph:**
    Construct an `attached_to` directed graph for the world with each vertex
    representing a frame (see [buildFrameAttachedToGraph](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L328)
    in `libsdformat9`):

    5.1 Add a vertex for the implicit world frame `world`
        (see [FrameSemantics.cc:333-338](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L333-L338)).

    5.2 Add a vertex for each model in the world.
        (see [FrameSemantics.cc:333-338](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L354-L369)).

    5.3 For each `//world/frame`:

    5.3.1 Add a vertex to the graph
          (see [FrameSemantics.cc:333-338](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L371-L386)).

    5.3.2 If `//world/frame/@attached_to` exists and is not empty,
          add an edge from the added vertex to the vertex named in the
          `//world/frame/@attached_to` attribute
          (see [FrameSemantics.cc:393-394](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L393-L394)
          and [FrameSemantics.cc:416-428](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L416-L428)).

    5.3.3 Otherwise (ie. if the `//world/frame/@attached_to` attribute
          does not exist or is an empty string `""`),
          add an edge from the added vertex to the implicit world frame vertex
          (see [FrameSemantics.cc:395-406](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L395-L406)
          and [FrameSemantics.cc:416-428](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L416-L428)).

    5.4 Verify that the graph has no cycles and that by following the directed
        edges, every vertex is connected to a model or the implicit world frame
        (see [validateFrameAttachedToGraph](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L976-L982)
        which is called by [World::Load](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/World.cc#L351-L352)).
        If the directed edges lead from a vertex to the implicit world frame,
        then the `//world/frame` corresponding to that vertex is a fixed
        inertial frame.
        If the directed edges lead to a model, then the `//world/frame`
        corresponding to that vertex is attached to the implicit model frame
        of that model.
        To identify the model or fixed frame to which each frame is attached, start from the
        vertex for that frame, and follow the directed edges until a link
        is reached (see [Frame::ResolveAttachedToBody](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/Frame.cc#L216)
        and [resolveFrameAttachedToBody in FrameSemantics.cc](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L1158) in `libsdformat9`).

6.  **Check `//pose/@relative_to` attribute values:**
    For each `//model/pose` and `//world/frame/pose`,
    if the `relative_to` attribute exists and is not an empty string `""`,
    check that the value of the `relative_to` attribute
    matches the name of a model or frame that is a sibling of the element
    that contains the `//pose`.
    In `libsdformat9`, these checks are performed by
    [buildPoseRelativeToGraph](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L754-L821),
    which is called by
    [World::Load](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/World.cc#L360-L362).

7.  **Check `//pose/@relative_to` graph:**
    Construct a `relative_to` directed graph for the model with each vertex
    representing a frame
    (see [buildPoseRelativeToGraph](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L666)
    in `libsdformat9`):

    7.1 Add a vertex for the implicit world frame.
        (see [FrameSemantics.cc:684-689](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L684-L689)).

    7.2 Add vertices for each `//world/model` and `//world/frame`
        (see [FrameSemantics.cc:691-705](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L691-L705)
        and [FrameSemantics.cc:714-729](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L714-L729)).

    7.3 For each `//world/model`:

    7.3.1 If `//world/model/pose/@relative_to` exists and is not empty,
          add an edge from the model vertex to the vertex named in
          `//world/model/pose/@relative_to`
          (see [FrameSemantics.cc:746-773](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L746-L773)).

    7.3.2 Otherwise (ie. if `//world/model/pose` or
          `//world/model/pose/@relative_to` do not
          exist or `//world/model/pose/@relative_to` is an empty string `""`)
          add an edge from the model vertex to the implicit world frame vertex
          (see [FrameSemantics.cc:707-711](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L707-L711)).

    7.4 For each `//world/frame`:

    7.4.1 If `//frame/pose/@relative_to` exists and is not empty,
          add an edge from the frame vertex to the vertex named in
          `//frame/pose/@relative_to`
          (see [FrameSemantics.cc:792-822](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L792-L822)).

    7.4.2 Otherwise if `//frame/@attached_to` exists and is not empty
          (ie. if `//frame/@attached_to` exists and is not an empty string `""`
          and one of the following is true: `//frame/pose` does not exist,
          `//frame/pose/@relative_to` does not exist, or
          `//frame/pose/@relative_to` is an empty string `""`)
          add an edge from the frame vertex to the vertex named in
          `//frame/@attached_to`
          (see [FrameSemantics.cc:798-822](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L798-L822)).

    7.4.3 Otherwise (ie. if neither `//frame/@attached_to` nor
          `//frame/pose/@relative_to` are specified)
          add an edge from the frame vertex to the implicit world frame vertex
          (see [FrameSemantics.cc:731-735](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L731-L735)).

    7.5 Verify that the graph has no cycles and that by following the directed
        edges, every vertex is connected to the implicit world frame
        (see [validatePoseRelativeToGraph](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/FrameSemantics.cc#L1146-L1152)
        which is called by [World::Load](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/src/World.cc#L365-L366)).
        Other poses in the world such as `//world/light/pose`
        do not need to be checked for cycles since they do not create
        implicitly named frames.
        To find the pose of a DOM object relative-to a named frame in the `PoseRelativeToGraph`,
        use the DOM object's `SemanticPose` function
        (such as [Frame::SemanticPose](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/include/sdf/Frame.hh#L143-L146))
        and the [SemanticPose::Resolve](https://github.com/osrf/sdformat/blob/sdformat9_9.2.0/include/sdf/SemanticPose.hh#L65-L73)
        function.

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
