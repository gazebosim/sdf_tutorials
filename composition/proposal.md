# Model composition: Proposed behavior

* **Authors**:
Eric Cousineau `<eric.cousineau@tri.global>`,
Steven Peters `<scpeters@osrfoundation.org>`,
Addisu Taddese  `<addisu@openrobotics.org>`
* **Status**: Draft
* **SDFormat Version**: 1.8
* **`libsdformat` Version**: 11.0

## Introduction

This proposal suggests changes to the semantics of composition, targeting
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
*   Proposed changes: Each addition / subtraction to SDFormat and `libsdformat`.
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
part of the reason being that nesting directly with `//model/model` was not
implemented in `libsdformat` until 9.3.0. As mentioned in the
[Legacy Behavior](/tutorials?tut=composition&ver=1.5&#libsdformats-implementation-of-include-in-models) documentation, `//include` works by effectively flattening the model; when this happens, certain details may "leak" through.

Normally, nesting of models generally implies that the elements can be
referenced via a form of scope, such as `{scope}::{name}`.
However, `::` is not a special token and thus can be
used to create "false" hierarchy or potential name collisions. Additionally, there is
no way for elements within the same file to refer "up" to another element, e.g. with in a robot assembly, adding a weld between a gripper and an arm when the
two are sibling models.

For posturing an included model, there is no means by which the user can
specify which included frame to posture via `//include/pose`. The target frame
to move currently can only be the
[`__model__` frame](/tutorials?tut=pose_frame_semantics_proposal#2-model-frame-and-canonical-link).
Therefore, if you wanted to weld a gripper to an end effector, but the canonical
link for the gripper is not at the weld point (or it has multiple potential
weld points), you must duplicate this pose information in the top-level.

For including models, it is nice to have access to other model types, e.g.
including a custom model specified as a `*.yaml` or connecting to some other
legacy format. Generally, the interface between models only really needs access
to explicit and implicit frames (for welding joints, attaching sensors, etc.).
The current implementation of `//include`
([since `libsdformat4`](https://github.com/osrf/sdformat/blob/sdformat4_4.0.0/src/parser.cc#L738))
requires that SDFormat know *everything* about the included model, whereas a
user could instead provide an adapter to provide the minimal information
necessary for assembly.

There are existing solutions to handle composition. Generally, those
solutions are some form of text / XML generation (e.g. `xacro`, or Python /
Ruby scripts). These methods can provide for more advanced things, like
parameterization, conditional branching, looping, working up towards Turing
completeness. However, these methods may not have a firm grasp of the semantics
of the data they are manipulating, and thus can undermine encapsulation, and
can add a layer of complexity when errors (syntactic, semantic, or design) are
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
referenced implicitly in nested scopes. This ensures that each model is an
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
    * This looks a bit more like a filesystem (more relevant to these
    semantics). However, there is inertia due to Gazebo's usage of `::`
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

##### 1.4.4 Placement frame: `//model/@placement_frame` and `//include/placement_frame`

It is useful to place an object using a semantic relationship between two
objects, e.g. place the bottom-center of a mug upright on the top-center of
a table. To do this, you can the specify the frame for which the `//model/pose`
or `//include/pose` should change.

This can be achieved by specifying `//model/@placement_frame` for directly
nested models or `//include/placement_frame` for included models. If the
placement frame element is specified for an included model, then
`//include/pose` *must* be specified, as any information in the included
`//model/pose` will no longer be relevant.

As an example, using directly nested models:

~~~xml
<model name="super_model">
  <model name="table" placement_frame="bottom_left_leg">
    <pose>2 4 0 0 0 0</pose>
    <link name="bottom_left_leg"/>
    <frame name="top_center">
      <pose relative_to="bottom_left_leg">1 1 2 0 0 0</pose>
    </frame>
  </model>
  <model name="mug" placement_frame="bottom_center">
    <pose relative_to="table::top_center"/>
    <link name="bottom_center"/>
  </model>
</model>
~~~

or using included models:

~~~xml
<model name="super_model">
  <include>
    <name>table</name>
    <uri>file://table.sdf</uri>
    <placement_frame>bottom_left_leg</placement_frame>
    <pose>2 4 0 0 0 0</pose>
  </include>
  <include>
    <name>mug</name>
    <uri>file://mug.sdf</uri>
    <placement_frame>bottom_center</placement_frame>
    <pose relative_to="table::top_center"/>
  </include>
</model>
~~~

It is worth mentioning that the `//model/@placement_frame` and
`//model/@canonical_link` attributes have different meanings and uses. Given
a `//model/pose`, the `//model/@placement_frame` says which frame in the model
will have that pose. This is a very practical means of setting the location of
models. The `//model/@canonical_link` attribute on the other hand specifies the
link to which the implicit model frame is attached. This does not affect
the model's initial pose, but constrains the implicit model frame to remain
fixed to the canonical link once simulation has started.

**Alternatives Considered**:

It was considered to implement only `//include/placement_frame`, but it was
found to be difficult to implement without making it a proper model attribute
that gets overridden.

##### 1.4.5 Permit files directly in `//include/uri`

Specifying a directory permits usage of `model.config` manifests, which permits
better compatibility for a model when being loaded by software with different
SDFormat specification support. However, it then requires overhead that may not
matter for some applications.

This proposal suggests that `//include/uri` permits referencing files directly.
If a file is relative, then it should be evaluated next to the file itself.

**WARNING**: In general, it is suggested to use `package://` URIs for ROS
models, and `model://` URIs for Gazebo models.

##### 1.4.6 `//static` for for included and directly nested models

If `//model/static` or `//include/static` is specified, the nested models
within that model will *only* be overridden if `//static` is `true`. Otherwise,
if `//static` is `false`, then all of the nested models (within that model)
will remain as specified.

**Alternatives Considered**:

1. The value could be overridden to `true` or `false`, always.

    However, implementations would have to be careful that `@attached_to` is
    properly respected when overriding `//model/static`. Normally, if a model is
    static, its frames will be attached to the world. However, if a model is
    non-static, the attached-to frame should only be within the model itself (to
    observe proper encapsulation).

    Additionally, it may not be possible to force certain models to be
    non-static.
    For example, if a `//static` is set to false, but the model is
    normally a static model with only frames (which is valid), an error should
    be thrown.

    It would also generally be *not* suggested to force a static model to be
    non-static, as the static model may not be designed for non-static use, and
    waste time on debugging.

    This was abandoned because it's overly complex and has sharp, jagged edges.

2. Require `//static` to be stored as a tri-state value (unspecified, true,
false).

    This allows the `//static` elements to be overridden and will affect
    *all* nested models, and can *only* change values from false to true;
    `//static` can only be false if all models included via the file are
    non-static.

    This alternative was abandoned because this is not very simple, and has
    relatively complicated rules. See
    [sdf_tutorials#33](https://github.com/osrf/sdf_tutorials/issues/33) for a
    brief mention.

###### 1.4.6.1 Interaction with `//joint/parent` and `//joint/child` now being frames

In section 1.2.2, it is proposed to admit frames in `//joint/parent` and
`//joint/child` to simplify the encapsulated interface of a model.

This means that a model can include a static version of a nested model, and
possibly use the nested model's frames (which are now effectively attached to
the world) as parent or child frames in a joint.

This is fine, and consistent with existing SDFormat behavior of allowing
kinematic loops (e.g. adding a rope, and attaching both ends to the world).

**Note**: `//joint/child == "world"` is forbidden solely due to encapsulation
issues with poses, as described in the
[Pose Frame Semantics proposal](/tutorials?tut=pose_frame_semantics_proposal#1-2-explicit-vs-implicit-frames).

##### 1.4.7 Up-Converting Flattened Models

As mentioned in the Motivation, in SDFormat <= 1.7 (`libsdformat` <= 10)
`//include` was implemented by "flattening" the model. This means that when
`libsdformat` parses a model which has nested `//include` models, the resultant
XML will be *invalid* for SDFormat 1.8 because the model would be using the
reserved `::` delimiter in an invalid fashion (to define a link, joint, etc.).

This would not be an issue if this flattened XML were a transient artifact
(e.g. temporary serialization for communicating models from a Gazebo server to
a client). However, users could have converted their models with `ign sdf`, and
thus there would be "data at rest" in this format.

To work around this, the conversion from SDFormat 1.7 to 1.8 should have an
"unflattening" phase which takes the flattened XML and *naively* tries to infer
when a new model should be created and when the nested names (e.g.
`M1::my_link`) should be "unnested" (e.g. `my_link` in model `M1`).

To illustrate, the following model from the
[Legacy Behavior](/tutorials?tut=composition&ver=1.5&#libsdformats-implementation-of-include-in-models)
documentation will have been up-converted to the following in SDFormat 1.7 (as
of `libsdformat` 9.2):

~~~xml
<sdf version="1.7">
  <model name="ParentModel">
    <frame name="ChildModel::__model__" attached_to="ChildModel::L1">
      <pose relative_to="__model__">1 0 1 0 0 0</pose>
    </frame>
    <link name="ChildModel::L1">
      <pose relative_to="ChildModel::__model__">0 1 0 0 0 0</pose>
      <visual name="v1">
        <geometry>
          <sphere>
            <radius>0.1</radius>
          </sphere>
        </geometry>
      </visual>
    </link>
    <link name="ChildModel::L2">
      <pose relative_to="ChildModel::__model__">0 0 0 0 0 0</pose>
    </link>
    <joint name="ChildModel::J1" type="revolute">
      <parent>ChildModel::L1</parent>
      <child>ChildModel::L2</child>
    </joint>
  </model>
</sdf>
~~~

should be (naively) up-converted to:

~~~xml
<sdf version="1.8">
  <model name="ParentModel">
    <model name="ChildModel" canonical_link="L1">
      <pose>1 0 1 0 0 0</pose>
      <link name="L1">
        <pose>0 1 0 0 0 0</pose>
        <visual name="v1">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
        </visual>
      </link>
      <link name="L2">
        <pose>0 0 0 0 0 0</pose>
      </link>
      <joint name="J1" type="revolute">
        <parent>L1</parent>
        <child>L2</child>
      </joint>
    </model>
  </model>
</sdf>
~~~

The following basic rules will apply:

* Models will be inferred (implicitly created) by parsing the following
attributes:
  * `//frame/@name`
  * `//joint/@name`
  * `//link/@name`
  * `//model/@name`.
* When a new model is created, all new elements under this model will have the
following attributes "unnested" by stripping the (required) prefix of
`"{model_name}::"`:
  * `//frame/@attached_to`
  * `//frame/@attached_to`
  * `//joint/child`
  * `//joint/parent`
  * `//pose/@relative_to`
  * `//xyz/@expressed_in`
  * Additional Rules:
    * If an attribute is either `"world"` or `"__model__"`, it will be
    unchanged.
    * If an attribute does *not* start with the given prefix, the conversion
    will *fail fast*.

Some notes:

* Semantic validation will be handled by the parsing process itself, *not* by
the naive up-conversion process.
* This up-conversion is *only* intended to support models that *could* have
been emitted by `libsdformat10` by using *valid* `//include` statements. It is possible to write valid models in SDFormat 1.7 that are invalid in SDFormat
1.8; no effort will be made to reconcile those models (see examples below).
* Since `libsdformat9.3` and above supports direct nesting but `//include`
still worked via flattening, unflattening will occur regardless of whether or
not there are directly nested models in the model, and the unflattening may
handle "partially" flattened models.
* Nested models with an explicitly specified `__model__` frame (e.g.
`ChildModel::__model__`) will have this frame removed, and this will be
converted to the appropriate `//model/pose`. `//model/@canonical_link` will
always be specified, whether or not `@attached_to` corresponds to the default
canonical link.

  * Any poses within this model will be expected to either refer to this model
  frame, or any frame contained by this model, s.t. no numerical computation
  needs to take place.
* If nested `__model__` frames are not present, no attempt will be made to
implicitly "offset" the consituent elements' poses
into the newly created `//model/pose`.

**Implementation**

This will be implemented by modifying the implicit conversion schema
(as defined and consumed by [`Converter.cc`](https://github.com/osrf/sdformat/blob/113bf26308f7354f446cc4dcd4746196d493bfde/src/Converter.cc))
for the [currently blank SDFormat 1.7 -> 1.8 file](https://github.com/osrf/sdformat/blob/113bf26308f7354f446cc4dcd4746196d493bfde/sdf/1.8/1_7.convert) to
have an element named `//unnest`, which will signify that the above mentioned changes should take place.

**Example of a failing conversions**

This could have been valid in SDFormat 1.7, but is not valid in SDFormat 1.8,
even with up-conversion:

~~~xml
<sdf version="1.7">
  <model name="anything">
    <link name="M1::B"/>
    <joint name="M1::J">
      <parent>M1::B</parent>
      <child>M2::B</child>  <!-- INVALID: This reference does not start with `M1::`. -->
    </joint>
    <link name="M2::B"/>
  </model>
</sdf>
~~~

This model could only have been produced by hand-crafting a flattened model
(most likely after conversion).

#### 1.5 Minimal `libsdformat` Interface Types for Non-SDFormat Models

As mentioned above, the encapsulation goal of this proposal should allow for
downstream libraries to permit specifying non-SDFormat model via `//include`
tags *without* `libsdformat` having to try and convert the model to SDFormat.

In order to do so, it is proposed that the following API hook be permitted to
be registered in `libsdformat`. This is described in the following pseudocode,
along with a specification of the proposed contract for custom parsers:

~~~c++
// This can be used in both //model elements as well as /world.
struct sdf::NestedInclude {
  /// Provides the URI as specified in `//include/uri`. This may or may not end
  /// with a file extension (it will not end an extension if it refers to a
  /// model package).
  std::string uri;

  /// Provides the *resolved* absolute file path from the URI.
  /// It is recommended to use this in `CustomModelParser` when check
  /// predicates on filenames -- however, the predicates should generally only
  /// check the file extension.
  std::string resolved_file_name;

  // N.B. Should be unnecesssary if downstream consumer has composition. Not
  // the case for Drake :(
  /// Name of the model in absolute hierarchy.
  /// Example: `top_model::middle_model::my_new_model`
  std::string absolute_model_name;

  /// Name relative to immediate parent as specified in `//include/@name`.
  /// Example: `my_new_model`
  std::string local_model_name;

  /// As defined by `//include/static`.
  bool is_static{false};

  /// This is a "virtual" XML element that will contain all custom (*unparsed*)
  /// elements and attributes within `//include`.
  sdf::ElementPtr virtual_custom_elements;
};

class sdf:::InterfaceFrame {
  /// \param[in] name The *local* name.
  /// \param[in] attached_to Name of attached-to frame. Must be "__model__" if
  ///   attached to model.
  /// \param[in] X_AF The pose of the frame relative to attached frame.
  public: InterfaceFrame(
      std::string name, std::string attached_to, math::Pose3d X_AF);
  /// Accessors.
  public: std::string GetName() const;
  public: std::string GetAttachedTo() const;
  public: math::Pose3d GetPoseInAttachedToFrame() const;
};

class sdf::InterfaceLink {
  /// \param[in] name The *local* name.
  /// \param[in] pose The pose of the link relative to model frame.
  public: InterfaceLink(std::string name, math::Pose3d X_ML);
  /// Accessors.
  public: std::string GetName() const;
  public: math::Pose3d GetPoseInModelFrame() const;
};

class sdf::NestedModelFramePoseGraph {
  /// \param[in] Minimum API to posture the model frame.
  public: math::Pose3d ResolveNestedModelFramePoseInWorldFrame() const;
  /// \param[in] relative_to Can be "world", or any frame within the nested
  ///   model's frame graph. (It cannot reach outside of this model).
  public: math::Pose3d ResolveNestedFramePose(
      std::string frame_name, std::string relative_to = "world");
};

// Repostures custom models for the given nested custom model.
// Simplest query is `GetModelPoseInWorldFrame()`.
using RepostureFunction =
    std::function<void(sdf::NestedModelFramePoseGraph graph)>;

class sdf::InterfaceModel {
  /// \param[in] name The *local* name (no nesting, e.g. "::").
  ///   If this name contains "::", an error will be raised.
  /// \param[in] reposture_function Called after pose graphs are constructed to
  ///   reposture objects.
  /// \param[in] canonical_link_name The canonical link's name. (It must be
  ///   registered).
  /// \param[in] model_frame_pose_in_canonical_link_frame Model frame pose
  ///   relative to canonical link's frame. Defaults to identity.
  /// \param[in] model_frame_pose_in_parent_model_frame Model frame pose
  ///   relative to the including model's frame. Defaults to identity.
  ///   \note This will not be used if //include/pose is specified.
  public: InterfaceModel(
      std::string name,
      sdf::RepostureFunction reposture_function,
      std::string canonical_link_name,
      math::Pose3d model_frame_pose_in_canonical_link_frame = {},
      math::Pose3d model_frame_pose_in_parent_model_frame = {});
  /// Accessors.
  public: std::string GetName() const;
  public: sdf::RepostureFunction GetRepostureFunction() const;
  public: std::string GetCanonicalLinkName() const;
  public: math::Pose3d GetModelFramePoseInCanonicalLinkFrame() const;
  public: math::Pose3d GetModelFramePoseInParentModelFrame() const;
  /// Provided so that hierarchy can still be leveraged from SDFormat.
  public: void AddNestedModel(sdf::InterfaceModelPtr nested_model);
  /// Gets registered nested models.
  public: std::vector<sdf::InterfaceModelConstPtr> GetNestedModels() const;
  /// Provided so that the including SDFormat model can still interface with
  /// the declared frames.
  public: void AddFrame(sdf::InterfaceFrame frame);
  /// Gets registered frames.
  public: std::vector<sdf::InterfaceFrame> GetFrames() const;
  /// Provided so that the including SDFormat model can still interface with
  /// the declared links.
  public: void AddLink(sdf::InterfaceLink link);
  /// Gets registered links.
  public: std::vector<sdf::InterfaceLink> GetLinks() const;
};

/// Defines a custom model parser.
///
/// Every custom model parser should define it's own way of (quickly) 
/// determining if it should parse a model. This should generally be done by
/// looking at the file extension of `include.resolved_file_name`, and
/// returning nullptr if it doesn't match a given criteria.
///
/// Custom model parsers are visited in the *reverse* of how they are defined.
/// The latest parser gains precedence.
///
/// Custom model parsers are *never* checked if resolved file extension ends
/// with `*.sdf` or `*.world`.
/// If libsdformat encounters a `*.urdf` file, it will first check custom
/// parser. If no custom parser is found, it will then convert the URDF XML to
/// SDFormat XML, and parse it as an SDFormat file.
///
/// \param[in] include The parsed //include information from which this model
///   should be parsed.
/// \param[out] errors Errors encountered during custom parsing.
///   If any errors are reported, this must return nullptr.
/// \returns An optional ModelInterface.
///   * If not nullptr, the returned model interface is incorporated into the
///     existing model and its frames are exposed through the frame graph.
///   * If nullptr and no errors are reported, then libsdformat should
///     continue testing out other custom parsers registered under the same
///     extension (e.g. a parser for `.yaml`, which may cover many different
///     schemas).
///
/// If an exception is raised by this callback, libsdformat will *not* try to
///intercept the exception.
using sdf::CustomModelParser = std::function<
    sdf::InterfaceModelPtr (sdf::NestedInclude include, Errors& errors)>;

/// Registers a custom model parser.
/// \param[in] model_parser Callback as described above.
void sdf::Root::registerCustomModelParser(
    sdf::CustomModelParser model_parser);
~~~

When `libsdformat` calls a custom model parser and that model parser succeeds,
then the parsing code will:

1. First ensure no `errors` were specified.
2. Incorporate the resulting `sdf::InterfaceModelPtr interface`:
  a. Assert that it is not `nullptr`
  b. For each `interface.GetNestedModels()`:
    i. Recursively incorporate the nested models.
  c. Register model frame and canonical link in frame graph.
  d. Register each individual frame in frame graph.

That should then allow the included models to have their frames used in the
top-level model's `//pose` definitions.

**Alternatives Considered**:

* For handling file predicates based on name and/or content:
  * Explicitly use a `sdf::FileNamePredicate` when calling
  `registerCustomModelParser`. This was not done because there is sufficient
  functionality in the API for individual parsers to do this.
  * Rather than use `sdf::FileNamePredicate`, it was considered to use something
  like `sdf::FileContentsPredicate`, to allow for a MIME type-like check to
  happen. However, since this is explicitly for models that are being included by
  `//include/uri`, it is expected that the URI supplied to SDFormat will resolve
  to a file on disk, thus we do not need to worry about contents "over the wire"
  (e.g. models passed from a Gazebo client to a Gazebo server,
  `/robot_description` in ROS, etc.).

* For the composition API:
  * It was considered to expose as much of the topology as possible, both links
  and frames, and possibly joints. However, that would complicate the
  implementation:
    * This `libsdformat` API would somehow have to infect existing API to allow
      custom-included models to populate existing graphs explicitly.
    * This infection would require additional encapsulation of `libsdformat`
      details (e.g. XML pointer for elements). While not necessarily bad in
      principle, this may be an impractical rearchitecture for the next
      release.

##### 1.5.1 Modifications to existing `libsdformat` API

There should be no large changes necessary to existing public `libsdformat`
API.

**Alternatives Considered**:

* There was consideration to have `sdf::SemanticPose` be able to declare
  indicate `::ResolveAttachedToFrame()`; however, this was not chosen because
  it requires being able to construct a `sdf::SemanticPose` in isolation and
  ultimately has more information than is necessary for this interface.
  Additionally, there is complication with what scope the semantic pose should
  supply the frame in.

##### 1.5.2 Motivating Example

In order to inform this API, a (non-working) prototype usage of the (phantom)
API was hashed out in
[drake#13128](https://github.com/robotlocomotion/drake/pull/13128).

**TODO(eric.cousineau)**: Update this once the initial API is fleshed out and
Drake has a working prototype usage.

#### 1.6 Proposed Parsing Stages

**WARNING**: This parsing stages have not yet incorporated all of the proposals above.
These changes will be added in incrementally. Once they are all added, this text will
be removed.

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

    6.3 Add a vertex for the implicit frame of each joint ~~with an edge~~
        ~~connecting from the joint to the vertex of its child *frame*~~
        (see [FrameSemantics.cc:242-257](https://github.com/osrf/sdformat/blob/4fd00c795bafb6f10a7a36356fe3f61a93c961c8/src/FrameSemantics.cc#L242-L257)).

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

    *6.8 Verify that the parent and child frames of each joint resolve to*
        *different values. This check can be skipped in the special case that*
        *"world" is the joint's parent frame since that frame is not in a*
        *model's `FrameAttachedToGraph` (checked in `libsdformat11` by `ign sdf --check`, see*
        *[Joint::ResolveParentLink](https://github.com/osrf/sdformat/blob/44cab95014e61849f508ec92a613100301512aaf/src/Joint.cc#L407-L418)*
        *and [parser.cc:1895-1930](https://github.com/osrf/sdformat/blob/44cab95014e61849f508ec92a613100301512aaf/src/parser.cc#L1895-L1930)).*

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

#### Short-form modifications for Interface API

*TODO(eric.cousineau): Factor into above steps.*

* Load the SDFormat model DOM.
* For each include:
  * If non-SDFormat, directly load, record the `sdf::InterfaceModel`.
  * If SDFormat-compatible, recursively load the DOM into nested models.
    * For each nested model, directly load any nested non-SDFormat models.
* Resolve frame graph.
  * Reposture non-SDFormat models.

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
