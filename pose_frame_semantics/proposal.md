# Pose Frame Semantics Proposal

* **Authors**:
Steven Peter `<scpeters@osrfoundation.org>`,
Addisu Taddese  `<addisu@openrobotics.org>`,
Eric Cousineau `<eric.cousineau@tri.global>`
* **Status**: Final
* **SDFormat Version**: 1.7
* **`libsdformat` Version**: 9.0

## Introudction

This proposal suggests a series of changes intended to support semantics for
more expressivity of kinematics and coordinate frames in SDFormat 1.7.
SDFormat 1.5 added `<frame>` elements to several elements, and the frame
attribute string to `<pose>` elements, as described in the
[documentation on existing behavior for pose frame semantics](/tutorials?tut=pose_frame_semantics).
Semantics for the frame element and attribute were not fully defined, so they
have not yet been used.
The changes proposed here are intended to fully define the frame element to
improve usability.

## Document summary

The proposal includes the following sections:

* [Motivation](#motivation): An explanation of the background and rationale behind the proposal
* [Proposed changes](#proposed-changes): Each addition to or subtraction from the existing SDFormat
version’s design, definitions, semantics and syntax, organized under
subsections of related concepts
* [Examples](#examples): Long form code samples of the proposed changes
* [Parsing phases](#phases-of-parsing-kinematics): Updated phases of parsing kinematics necessary for
SDFormat 1.7 models and worlds

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

This proposes to improve the expressivity of model specification in SDFormat by
adding the ability to define arbitrary coordinate frames within a model and
choose the frame relative to which each frame is defined.

This allows frames to be used to compose information and minimize redundancy,
such as specifying a link's pose relative to its parent / inboard joint.
It could also be used to abstract other information for inverse kinematics,
visual servoing, or sensor calibration by defining a camera pose using a frame
instead of a base link and pose offset.

## Proposed changes

The following sections are the proposed changes regarding the semantics of
SDFormat 1.7’s frame element and supporting topics.
Top level sections may encompass subsections of proposed changes that fall
under the same purview.
Each section or subsection describes a proposed change, the details
surrounding it, how it differs from existing functionality, and why the change
is necessary.
Some sections include examples and alternatives considered.

### 1 Terminology

#### 1.1 Frames and poses

Each pose must be defined **relative to** (or be measured in) a certain frame.
This is achieved by the attribute `//pose/@relative_to`, [described below](#6-details-of-pose-relative_to-attribute).

Arbitrary frames are defined with the `//frame` tag.
A frame must have a name (`//frame/@name`),
be **attached to** another frame or link (`//frame/@attached_to`),
and have a defined pose (`//frame/pose`).
Further details of the change include:

* A pose defined **relative to** a given frame does not imply it is
 **attached to** that frame.
* A pose's **relative to** frame only defines its *initial configuration*;
any movement due to degrees of freedom will only result in a new pose as
defined by its **attached to** frame.
    * This is done in order to support a "Model-Absolute" paradigm for model
    building; see the [Addendum on Model Building](#addendum-model-building-contrast-model-absolute-vs-element-relative-coordinates) for further discussion.

Defining these semantics, which were missing from SDFormat 1.5,
minimizes redundancy in poses and offsets and makeslo relationships between physical elements easier to interpret

#### 1.2 Explicit vs. implicit frames

Explicit frame elements (`//frame`) must only appear in `//model`
(`//model/frame`) and `//world` (`//world/frame`) elements.
Implicit frames,  defined by non-frame elements, must be introduced for convenience.
The following frame types are implicitly introduced:

* Link frames: each link has a frame named `//link/@name` attached to the
  link at its origin defined by `//link/pose`.
* Joint frames: each joint has a frame named `//joint/@name` attached to the
  child link at the joint's origin defined by `//joint/pose`.
  As a direct consequence, `world` is no longer permitted to be specified
  as a child link of a joint, since that would break encapsulation of the
  model.
* Model frame: each model has a frame that is attached to one of its links
  (see [section 2.1](#2-1-implicit-frame-defined-by-model-pose-attached-to-canonical-link)
  for the definition of a model's canonical link).
  Depending on the context, a model frame can be referenced with `__model__`
  or `//model/@name` (see [section 2.2](#2-2-referencing-the-implicit-model-frame-via-__model__-or-model-name)).
  The model frame is the default frame to which explicit model frames
  defined by `//model/frame` are attached.
* World frame: each world has a fixed inertial reference frame named `world` that is
  the default frame to which explicit world frames defined by `//world/frame`
  are attached.

SDFormat 1.5 permitted explicit `//frame` elements in many places.
SDFormat 1.7 limits `//frame` elements to `//model` and `//world` since
this provides equivalent functionality with much less complexity required in
the SDFormat parser.

##### 1.2.1 Alternatives considered

Introducing implicit frames for other elements such as `//link/visual`,
`//link/collision`, and `//link/sensor` was considered. However, it was
determined that introducing these implicit frames adds unnecessary complexity
to the SDFormat parser. It would also pollute the frame graph making it less
efficient to traverse.

#### 1.3 Empty `//pose` and `//frame` elements imply identity pose

Empty `//pose` elements must be interpreted as equivalent to the identity pose.
In the following examples, the `//pose` elements in each pair are equivalent to each other.

~~~
<pose />
<pose>0 0 0 0 0 0</pose>
~~~

~~~
<pose relative_to='frame_name' />
<pose relative_to='frame_name'>0 0 0 0 0 0</pose>
~~~

Likewise, empty `//frame` elements must be interpreted as having an identity
pose relative to `//frame/@attached_to`.
The four frame elements below are equivalent.

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

In SDFormat 1.5, an identity pose is specified by omitting the tag or with an
explicit `<pose>0 0 0 0 0 0</pose>`.
This change is a convenience to reduce verbosity, because there are many
expected cases where a frame is defined relative to another frame with no
additional pose offset.

### 2 Model frame and canonical link

#### 2.1 Implicit frame defined by `//model/pose` attached to canonical link

Each non-static model must have at least one link designated as the canonical link.
The implicit frame of the model is attached to this link.
This implicit frame of a static model is attached to the world.

The implicit frame is defined by the `//model/pose` element, typically
called the "model frame".
It must be user-configurable, but with a default value.

The name of the canonical link can be specified in the `//model/@canonical_link` attribute.
If it is not specified, then the first `//link` element in the model is chosen as the canonical link.

The following two models are equivalent:

~~~
<!-- //model/@canonical_link -->
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

In SDFormat 1.4, the model frame is the frame relative to which all
`//link/pose` elements are interpreted.
The SDFormat 1.4 specification does not clearly state to which link the model
frame is attached, but Gazebo has a convention of choosing the first `//link`
element listed as a child of a `//model` as the `@attached_to` link and
referring to this as the model's Canonical Link
(see [Model.cc from gazebo 10.1.0](https://bitbucket.org/osrf/gazebo/src/gazebo10_10.1.0/gazebo/physics/Model.cc#lines-130:132)).

In SDFormat 1.5, a model without links is considered valid, but its implicit
frame is not well-defined since it is not clear where the frame is attached.
It is necessary to specify what a canonical link is because the model frame
must be attached to this link.

##### 2.1.1 Alternatives considered

~~~
<!-- //link/@canonical -->
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

#### 2.2 Referencing the implicit model frame via `__model__` or model name

SDFormat 1.7 must provide a means to reference the implicit model frame
depending on the context.

From child elements of a given model, the "internal implicit model frame" can
be referenced using the reserved name `__model__`.
From outside of a given model, the "external implicit model frame" can be
referenced using the model's specified name.

Nested models will have their own individual model frames. (See pending Nesting
proposal for nuances.)

Previous versions of SDFormat did not have the `@attached_to` and `@relative_to`
attributes, so there was no way to refer to frames by name.
The ability to reference `__model__` makes implementation more straightforward.

##### 2.2.1 Alternatives considered

The first alternative was to make the model frame be explicitly named as the
model name specified by the file (not overridden by `//include`). No link,
joint, or frame can be specified using this name.

The caveat with this is that it be confusing if/when more complex references
are supported via nesting.

The second alternative was to prevent the implicit model frame from being
referred to explicitly.

For this solution, implementation could become complicated / ambiguous when
handling default frames. For example, `""` could be used as the token for the
model / world frame. Additionally, `@relative_to` defaults to `""`, but in some
contexts this may imply the model frame, the parent element frame, the child link frame, etc.

It also complicates migration via `Converter.cc` when handling things like
replacing `//joint/axis/use_parent_frame` with
`//joint/axis/xyz/@expressed_in`.

#### 2.3 Referencing the implicit world frame via `world`

The "internal implicit frame" for a `//world` element must be `world` (rather
than `__model__` or `__world__`). This may not be referred to within `//model`
elements, *except* for specifying `//joint/parent`.

For the same reasons as the implicit model frame, the ability to reference
`world` makes implementation more straightforward.

##### 2.3.1 Alternatives considered

Two possible alternatives were (a) have both `__world__` for the world frame and
`world` for the world link or (b) use `__world__` for both.

(a) was decided against because it seemed redundant given that `world` could be
referenced both as a frame and a link, which is consistent with implicit frames
for links. (b) was decided against because it would create additional churn to
support both `world` and `__world__` up to a point, and then switch over to
`__world__`.

### 3 Name conflicts and scoping rules for explicit and implicit frames

As frames are referenced in several attributes by name, it is necessary to
avoid naming conflicts between frames defined in `//world/frame`,
`//world/model`, `//model/frame`, `//model/link`, and `//model/joint`.
This motivates the scoping and naming rules proposed in the following sections.

#### 3.1 Scoping rules for referencing frames by name

To ensure that multiple copies of the same model can co-exist as siblings
in a world, separate scopes must be defined for referencing frames by name:

* Model scope: each model has its own scope in which explicit `//model/frame`
  and implicit `//model/link` and `//model/joint` frames can be referenced.
* World scope: the world has a separate scope in which explicit `//world/frame`
  and implicit `//world/model` frames can be referenced.

For example, the following world has three scopes, one each for the world,
`model_1`, and `model_2`.
The world scope contains the explicit `//world/frame` named `explicit_frame`
and the implicit model frames `model_1` and `model_2`.
The `model_1` and `model_2` scopes each contain frames named `explicit_frame`
and `link`, but there is no name conflict because they are in separate scopes.

~~~
<world name="frame_scope">
  <frame name="explicit_frame"/>
  <model name="model_1">
    <frame name="explicit_frame"/>
    <link name="link"/>
  </model>
  <model name="model_2">
    <frame name="explicit_frame"/>  <!-- VALID: name is unique in this model. -->
    <link name="link"/>             <!-- VALID: name is unique in this model. -->
  </model>
</world>
~~~

In the following example, there are three frames in the model scope:
the implicit link frames `base` and `attachment` and the implicit
joint frame `attachment`.
Referring to a frame named `attachment` is ambiguous in this case,
which inspires the element naming rule in the following section
that disallows name conflicts like this.

~~~
<model name="model">
  <link name="base"/>
  <link name="attachment"/>
  <joint name="attachment" type="fixed">  <!-- INVALID: joint name should not match name of sibling link. -->
    <parent>base</parent>
    <child>attachment</child>
  </joint>
</model>
~~~

In SDFormat 1.4, the only objects referenced by name are the links
named in `//joint/parent` and `//joint/child`, and the names are always
scoped to the current model.
In SDFormat 1.5, the `::` delimiter is used to indicate that the target
link is within a nested model, but the scope is still limited to
objects contained in the current model.
With the addition of `//world/frame` and `//world/model/pose/@relative_to`,
it is necessary to consider the world scope separately from each
model's scope to avoid name conflicts and ensure encapsulation.

##### 3.1.1 Alternatives considered

One alternative was to not use any scoping at all, such that any frame could
be referenced by name from any other part of the world.
This would make naming conflicts much more common as you could not include
two copies of the same model in a world without giving unique names to the
links, joints, and explicit frames.

Another approach instead of scoping is to treat the xml hierarchy like a
filesystem and use familiar operators such as `/` and `..` to specify frame
references directly in the hierarchy.
This adds significant complexity and difficulty and deviates from the current
practice of specifying parent and child links for a joint by name.

The current proposal is the most conservative and should allow for more
expansive (but possibly more complex) scoping rules to be incorporated
while maintaining compatibility.

#### 3.2 Unique names for all sibling elements

<!-- TODO(eric): These naming rules should stay in this proposal, but should
then transition to a nesting / scoping proposal once they land. -->

All named sibling elements must have unique names.
Uniqueness is forced so that referencing implicit frames is not ambiguous, e.g.
you cannot have a link and joint share an implicit frame name.

One method of ensuring name uniqueness across element types is by adopting the
practice of including the element type in model names.
For example, numbering models as `link1` / `link2` or using element types as a
suffix, like `front_right_wheel_joint` / `front_right_steering_joint`.
Furthermore, the frame semantics proposed in this document use the names of
sibling elements `//model/frame`, `//model/link` and `//model/joint` to refer
to frames.

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

Some existing SDFormat models may not comply with this requirement.
* A validation tool will need to be created to identify models that violate
this requirement.
* The specification version is incremented to 1.7 so that checks can be added
when converting to the newer, stricter version.

SDFormat 1.5 is more permissive and does not explicitly disallow identical
sibling names.
This change is necessary because frames are referenced in several attributes by name.
It is necessary to avoid naming conflicts between frames defined in
`//model/frame`, `//model/link` and `//model/joint`.

##### 3.2.1 Alternatives considered

It was considered to specify the frame type in the `//frame/@attached_to`
and `//pose/@relative_to` attributes in order to avoid this additional naming
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

#### 3.3 Reserved names

Entities in a simulation must not use `world` as a name. It has a special
interpretation when specified as a parent or child link of a joint.

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

Names starting and ending with double underscores (eg. `__wheel__`) must be
reserved for use by library implementors and the specification. For example,
such names might be useful during parsing for setting sentinel or default names
for elements with missing names.
If explicitly stated, they can be referred to
(e.g. `__model__` / `world` for implicit model / world frames, respectively).

~~~
<model name="__model__"/><!-- INVALID: name starts and ends with __, and is reserved. -->
~~~

~~~
<model name="model">
  <!-- VALID: Both frames are equivalent. -->
  <frame name="frame1"/>
  <frame name="frame2" attached_to="__model__"/>
</model>
~~~

~~~
<model name="model">
  <link name="__link__"/><!-- INVALID: name starts and ends with __. -->
</model>
~~~

In SDFormat 1.5, when a joint specifies “world” as its parent or child link,
its behavior is inconsistent and depends on the existence of a sibling link named “world”.
If such a sibling link exists, that link will be used as the parent / child,
but if no sibling link exists, then a static link fixed to the world frame is used instead.
These changes reduce the inconsistency by disallowing sibling links named “world”.

### 4 Details of `//model/frame`

The `//model/frame` must have two attributes, `@name` and `@attached_to`, as well
as a child `//pose` element that specifies the initial pose of the frame.
Further details of the attributes of `//model/frame` are given in the following subsections.

SDFormat 1.5 introduced `//model/frame` and its attributes,
but left the semantics undefined.
It is necessary to define these semantics so the element and attributes can be utilized.

#### 4.1 The `//model/frame/@name` attribute

The `//model/frame/@name` attribute must specify the name of a `//frame`.
It is a required attribute, and can be used by other frames in the `@attached_to`
and `//pose/@relative_to` attributes to refer to this frame.
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

#### 4.2 The `//model/frame/@attached_to` attribute

The `//model/frame/@attached_to` attribute must specify the link to which the
`//frame` is attached.
It is an optional attribute.
If it is specified, it must contain the name of an explicit or implicit frame
in the current scope.
Cycles in the `@attached_to` graph are not allowed.
If a `//frame` is specified, recursively following the `@attached_to` attributes
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

  <frame name="F0" attached_to="F0"/>  <!-- INVALID: cycle in attached_to graph does not lead to link. -->

  <frame name="F1" attached_to="F2"/>
  <frame name="F2" attached_to="F1"/>  <!-- INVALID: cycle in attached_to graph does not lead to link. -->
</model>
~~~

### 5 Details of `//world/frame`

The `//world/frame` must have two attributes, `@name` and `@attached_to`,
and a child `//pose` element that specifies the initial pose of the frame.
Further details of the attributes of `//world/frame` are given in the following subsections.

SDFormat 1.5 introduced `//world/frame` and its attributes, but left the semantics undefined.
It is necessary to define these semantics so the element and attributes can be utilized.

#### 5.1 The `//world/frame/@name` attribute

The `//world/frame/@name` attribute must specify the name of the frame.

To avoid ambiguity, sibling frames — explicit frames specified by `//world/frame` and
implicit frames specified by `//world/model`— must have unique names.

#### 5.2 The `//world/frame/@attached_to` attribute

The `//world/frame/@attached_to` attribute must specify another frame to which
this frame is attached.

A `//world/frame` can be attached to an implicit frame
(defined by `//world` or `//world/model`) or to an explicit frame defined by
`//world/frame`.
If the `//world/frame/@attached_to` attribute is not
specified or is left empty, the frame will be attached to the world frame. If
the attribute is specified, it must refer to a sibling `//world/frame` or
`//world/model`.

When a a `//world/frame` is attached to a `//world/model`, it is indirectly
attached to the canonical link of the model.

Similar to `//model/frame`, cycles in the `@attached_to` graph are not allowed.
If a `//world/frame` is specified, recursively following the `@attached_to`
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
  <frame name="F0" attached_to="F0"/>  <!-- INVALID: cycle in attached_to graph does not lead to the implicit world frame. -->

  <frame name="F1" attached_to="F2"/>
  <frame name="F2" attached_to="F1"/>  <!-- INVALID: cycle in attached_to graph does not lead to the implicit world frame. -->
</world>
~~~

### 6 Details of `//pose/@relative_to` attribute

The `//pose/@relative_to` attribute must indicate the frame relative to which
the initial pose of the frame is expressed.
This must be applied equally to the pose of explicit frames (`//frame/pose`),
implicit frames (`//model/pose`, `//link/pose`, and `//joint/pose`),
and objects without named frames
(`//collision/pose`, `//light/pose`, `//sensor/pose`, `//visual/pose`).

* If the `//pose/@relative_to` attribute is not an empty string `""`, its value
must match the name of an explicit or implicit frame in the current scope.
* If the `//pose/@relative_to` attribute does not exist or is empty,
the default behavior for all elements other than `//frame/pose` is the
behavior from SDFormat 1.4
(see the "Parent frames in sdf 1.4" section of the
[pose frame semantics documentation](/tutorials?tut=pose_frame_semantics)).
  * This corresponds to `//link/pose` relative to the model frame by default
and `//joint/pose` relative to the child link's implicit frame by default.
* If the `//frame/pose/@relative_to` attribute does not exist or is empty,
it defaults to the value of the `//frame/@attached_to` attribute.

Cycles in the `@relative_to` attribute graph are not allowed and must be
checked separately from the `@attached_to` attribute graph.
Following the `@relative_to` attributes of the specified frames in the model
scope must lead to a frame expressed relative to the model frame.
In the world scope, following the `@relative_to` attributes must lead to
the implicit world frame.

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
  <link name="L4">
    <pose relative_to="A">{X_AL4}</pose>    <!-- INVALID: no frame in model scope named A. -->
  </link>

  <link name="cycle0">
    <pose relative_to="cycle0">{X_C0C0}</pose>  <!-- INVALID: cycle in relative_to graph does not lead to model frame. -->
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

  <frame name="F1" attached_to="L">         <!-- Frame directly attached_to link L. -->
    <pose>{X_LF1}</pose>                    <!-- Pose relative_to the attached_to frame (L -> M) by default. -->
  </frame>
  <frame name="F2" attached_to="L">         <!-- Frame directly attached_to link L. -->
    <pose relative_to="">{X_LF2}</pose>     <!-- Pose relative_to the attached_to frame (L -> M) by default. -->
  </frame>
  <frame name="F3">                         <!-- Frame indirectly attached_to canonical link L via model frame. -->
    <pose relative_to="L">{X_LF3}</pose>    <!-- Pose relative_to link frame L -> M. -->
  </frame>
  <frame name="F4">
    <pose relative_to="A">{X_AF4}</pose>    <!-- INVALID: no frame in model scope named A. -->
  </frame>

  <frame name="cycle0">
    <pose relative_to="cycle0">{X_C0C0}</pose>  <!-- INVALID: cycle in relative_to graph does not lead to model frame. -->
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
<world name="scope_relative_to">              <!-- Implicit world frame. Referred to as (W) -->
  <frame name="W0">                           <!-- Frame attached_to implicit world frame by default. -->
    <pose>{X_WW0}</pose>                      <!-- Pose relative_to the attached_to frame (W) by default. -->
  </frame>
  <frame name="W1">                           <!-- Frame attached_to implicit world frame by default. -->
    <pose relative_to="W0">{X_W0W1}</pose>    <!-- Pose relative_to explicit frame W0. -->
  </frame>

  <model name="M1">
    <pose>{X_WM1}</pose>                      <!-- Model pose relative_to world frame by default. -->

    <frame name="F">                          <!-- Frame indirectly attached_to canonical link link L via model frame. -->
      <pose>{X_MF}</pose>                     <!-- Pose relative_to the attached_to frame (M) by default. -->
    </frame>
    <link name="L">
      <collision name="C">
        <pose relative_to="F">{X_FC}</pose>   <!-- Pose relative to explicit frame F (F -> M) in this model's scope. -->
      </collision>
      <visual name="V">
        <pose relative_to="F">{X_FV}</pose>   <!-- Pose relative to explicit frame F (F -> M) in this model's scope. -->
      </visual>

      <light name="L">                        <!-- Name matches that of containing link, which is permitted. -->
        <pose relative_to="F">{X_FL}</pose>   <!-- Pose relative to explicit frame F (F -> M) in this model's scope. -->
      </light>
    </link>

    <frame name="F1" attached_to="L">         <!-- Frame directly attached_to link L. -->
      <pose relative_to="C">{X_CL}</pose>     <!-- INVALID: no frame named C in this scope (collisions don't have implicit frames). -->
    </frame>
    <frame name="F2" attached_to="L">         <!-- Frame directly attached_to link L. -->
      <pose relative_to="W0">{X_W0L}</pose>   <!-- INVALID: no frame named W0 in this scope (can't access world scope from within model). -->
    </frame>
  </model>

  <frame name="W2" attached_to="M1"/>         <!-- Frame indirectly attached_to canonical link L of model M1. -->
  <frame name="W3">                           <!-- Frame attached_to world frame by default. -->
    <pose relative_to="M1">{X_M1W3}</pose>    <!-- Pose relative_to implicit model frame M1. -->
  </frame>

  <model name="M2">
    <pose relative_to="W3">{X_W3M2}</pose>    <!-- Pose relative_to explicit frame W3. -->
    <link name="L"/>
  </model>
</world>
~~~

~~~
<world name="model_pose_relative_to">
  <model name="noframe0">
    <pose relative_to="noframe0">{X_N0N0}</pose>  <!-- INVALID: no frame named noframe0 in this scope. -->
    ...
  </model>

  <model name="cycle0">
    <pose relative_to="__model__">{X_C0C0}</pose> <!-- INVALID: cycle in relative_to graph does not lead to world frame. -->
    ...
  </model>
</world>
~~~

~~~
<world name="world_frame_cycles">
  <frame name="cycle0">
    <pose relative_to="cycle0">{X_C0C0}</pose>  <!-- INVALID: cycle in relative_to graph does not lead to world frame. -->
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
It is not a cycle, however, since the `@attached_to` and `@relative_to` attributes
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

SDFormat 1.5 added the `//pose/@frame` attribute, but the semantics were left undefined.
Defining the semantics of `//pose/@frame`, now `//pose/@relative_to`,
is necessary to allow the use of the attribute.

### 7 Replace `//joint/axis/use_parent_model_frame` with `//joint/axis/xyz/@expressed_in`

The `//joint/axis/use_parent_model_frame` tag must be removed in SDFormat 1.7.,
and `//joint/axis/xyz/@expressed_in` must be added to modify the orientation
of this vector.

An empty string or default value implies the joint's initial orientation.
Any valid frame can be referred to from here.
This also applies to `//joint/axis2`.

As an example, an SDFormat 1.6 joint like this:

~~~
<model name="example">
  ...
  <joint name="joint" type="revolute">
    <pose>{X_CJ}</pose>
    <parent>{parent}</parent>
    <child>{child}</child>
    <axis>
      <xyz>{xyz_axis_M}</xyz>
      <use_parent_model_frame>true</use_parent_model_frame>
    </axis>
  </joint>
</model>
~~~

Becomes the following in SDFormat 1.7:

~~~
<model name="example">
  ...
  <joint name="joint" type="revolute">
    <pose>{X_CJ}</pose>
    <parent>{parent}</parent>
    <child>{child}</child>
    <axis>
      <xyz expressed_in="__model__">{xyz_axis_M}</xyz>
    </axis>
  </joint>
</model>
~~~

SDFormat 1.5 introduced this tag to maintain backwards compatibility with
SDFormat 1.4 when specifying the unit vector along the axis of motion of a joint.

The removal is necessary now because it has become clear that the usefulness of
this tag is outweighed by the confusion it creates,
as the resulting frame semantics of `//joint/axis/xyz` are inconsistent with the
way other tags in SDFormat operate.
The addition of `//joint/axis/xyz/@expressed_in` accommodates the migration from
`use_parent_model` and improves expressiveness.

#### 7.1 Alternatives considered

Migration for `//jonit/axis/xyz` is absolutely necessary if
`//use_parent_model_frame` is removed. SDFormat's current conversion code (in
`src/Converter.cc`) is only for changing the basic structure of a document, and
this change would require more involved changes.

## Examples

The following sections provide more in-depth examples of the major concepts
proposed in the above sections.
The examples highlight SDFormat 1.7’s powerful expressiveness for constructing
models using relative coordinate frames.

### 1 The `//pose/@relative_to` attribute

Consider the following figure from the
[specifying pose documentation](/tutorials?tut=specify_pose).
It shows a parent link `P`, child link `C`, and joint `J` with joint frames
`Jp` and `Jc` on the parent and child respectively.

<!-- Figure Credit: Alejandro Castro -->

[[file:../spec_model_kinematics/joint_frames.svg|600px]]

An SDFormat representation of this model is given below.
The pose of the parent link `P` is specified relative to the implicit
model frame, while the pose of the other
elements is specified relative to other named frames.
This allows poses to be defined recursively, and also allows explicitly named
frames `Jp` and `Jc` to be attached to the parent and child, respectively.
For reference, equivalent expressions of `Jc` are defined as `Jc1` and `Jc2`.

    <model name="M">

      <link name="P">
        <pose relative_to="__model__">{X_MP}</pose>
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

      <frame name="Jc2" attached_to="J" /> <!-- Jc2 == Jc1, since //pose/@relative_to defaults to J. -->

    </model>

#### 1.1 Parity with URDF using `//pose/@relative_to`

The following image from the URDF documentation corresponds to the example URDF from the
"Parent frames in URDF" section of the
[Pose Frame Semantics: Legacy Behavior documentation](/tutorials?tut=pose_frame_semantics).

<img src="http://wiki.ros.org/urdf/XML/model?action=AttachFile&do=get&target=link.png"
     alt="urdf coordinate frames"
     height="500"/>

The same URDF model can be expressed with identical kinematics with SDFormat
by using link and joint names in the pose `@relative_to` attribute.

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

The difference between the URDF and SDFormat expressions is shown in the patch below:

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
A validator could be created to identify SDFormat files that can be directly
converted to URDF with minimal modifications based on these principles.

##### 1.1.1 Alternatives considered

An even simpler approach to getting parity with URDF would be to add an
attribute `//joint/@attached_to_child` that specifies whether the implicit
joint frame is attached to the child link (true) or the parent link (false).
If `//joint/pose/@relative_to` is unset, this attribute would determine
the default value of `//joint/pose/@relative_to`.
For backwards compatibility, the attribute would default to true.
In this example, setting that attribute to false would eliminate the need
to specify the `//joint/pose/@relative_to` attribute and the duplication
of the parent link name.
As seen below, the `//link/pose/@relative_to` attributes still need to be set:

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
with the other proposed functionality.

### 2 Parity with URDF using `//model/frame`

One application of the `//frame` tag is to organize the model so that the pose
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

### 3 Using `//pose/@relative_to` for co-located elements within a link

The pose information of elements attached to links is often duplicated.
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

By creating explicit frames at the model scope, the `//pose/@relative_to`
attribute can be used to specify the collision, visual, and light poses
without duplication.

    <model name="model_with_explicit_frames">
      <frame name="led1" attached_to="link_with_LEDs">
          <pose>0.1 0 0 0 0 0</pose>
      </frame>
      <frame name="led2" attached_to="link_with_LEDs">
          <pose>-0.1 0 0 0 0 0</pose>
      </frame>

      <link name="link_with_LEDs">
        <light name="led1_light" type="point">
          <pose relative_to="led1" />
        </light>
        <collision name="led1_collision">
          <pose relative_to="led1" />
          <geometry>
            <box>
              <size>0.01 0.01 0.001</size>
            </box>
          </geometry>
        </collision>
        <visual name="led1_visual">
          <pose relative_to="led1" />
          <geometry>
            <box>
              <size>0.01 0.01 0.001</size>
            </box>
          </geometry>
        </visual>

        <light name="led2_light" type="point">
          <pose relative_to="led2" />
        </light>
        <collision name="led2_collision">
          <pose relative_to="led2" />
          <geometry>
            <box>
              <size>0.01 0.01 0.001</size>
            </box>
          </geometry>
        </collision>
        <visual name="led2_visual">
          <pose relative_to="led2" />
          <geometry>
            <box>
              <size>0.01 0.01 0.001</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>

#### 3.1 Alternatives considered

Instead of permitting elements inside a link from using the
`//pose/@relative_to` attribute at the model scope, one could allow
implicit frames for elements inside a link (like `//link/collision`,
`//link/visual`, etc.) and/or explicit link frames `//link/frame` and allow any
poses of any element to be `@relative_to` explicit or implicit frames
defined by sibling elements.

    <model name="model_with_explicit_link_frames">
      <frame name="F"/>                   <!-- Explicit frame F in model scope. -->
      <frame name="M"/>                   <!-- Explicit frame F in model scope. -->
      <link name="L">
        <frame name="F">                  <!-- Explicit frame F in link L scope. -->
          <pose>{X_LF}</pose>
        </frame>
        <light name="led1" type="point">
          <pose relative_to="F" />        <!-- Valid: Pose relative_to frame F in link scope. -->
        </light>
        <light name="led2" type="point">
          <pose relative_to="M" />        <!-- INVALID: frame M not in link scope. -->
        </light>
      </link>
    </model>

While there may be use cases that benefit from embedding explicit frames
inside their `@attached_to` link, doing so adds unnecessary complexity.
Frames would require additional scopes to be resolved, and the ability
to reference frames across links would be limited.
Furthermore, increasing the number of implicit frames increases the size
of the frame graph and adds complexity to the parsing task.
This approach is not recommended as its utility is outweighed by
its complexity.

## Phases of parsing kinematics

The following sections describe the phases for parsing the kinematics of an
SDFormat 1.7 model and world.
Several of the phases in each section are similar to the phases of parsing in
SDFormat 1.4 in the [Legacy behavior documentation](/tutorials?tut=pose_frame_semantics).
In phases that differ from SDFormat 1.4, *italics* are used to signal the difference.
For new phases, the ***Title:*** is italicized.

### 1 Model

There are *seven* phases for validating the kinematics data in a model.
In libsdformat, the `sdf::readFile` and `sdf::readString` API's perform parsing
stage 1, and `sdf::Root::Load` is proposed to perform all parsing stages.
Each API returns an error code if errors are found during parsing.

1.  **XML parsing and schema validation:**
    Parse model file from XML into a tree data structure,
    ensuring that there are no XML syntax errors and that the XML
    data complies with the [schema](http://sdformat.org/schemas/root.xsd).
    Schema `.xsd` files are generated from the `.sdf` specification files
    when building `libsdformat` with the
    [xmlschema.rb script](https://bitbucket.org/osrf/sdformat/src/sdformat6_6.2.0/tools/xmlschema.rb).

2.  **Name attribute checking:**
    Check that name attributes are not an empty string `""`,
    *that they are not reserved (*`__.*__` *or* `world`*)* and that sibling
    elements of *any* type have unique names.
    This includes but is not limited to models, actors, links, joints,
    collisions, visuals, sensors, and lights.
    This step is distinct from validation with the schema because the schema
    only confirms the existence of name attributes, not their content.

3.  **Joint parent/child name checking:**
    For each joint, check that the parent and child link names are different
    and that each match the name of a sibling link to the joint,
    with the following exception:
    if "world" is specified as a *parent* link name,
    then the joint is attached to a fixed reference frame.

4.  ***Check `//model/@canonical_link` attribute value:***
    If the `//model/@canonical_link` attribute exists and is not an empty
    string `""`, check that the value of the `canonical_link` attribute
    matches the name of a link in this model.

5.  ***Check `//model/frame/@attached_to` attribute values:***
    For each `//model/frame`, if the `attached_to` attribute exists and is not
    an empty string `""`, check that the value of the `attached_to` attribute
    matches the name of a sibling link, joint, or frame.
    The `//frame/@attached_to` value must not match `//frame/@name`,
    as this would cause a graph cycle.

6.  ***Check `//model/frame/@attached_to` graph:***
    Construct an `attached_to` directed graph for the model with each vertex
    representing a frame:

    6.1 Add a vertex for the implicit frame of each link in the model.

    6.2 Add a vertex for the implicit model frame. If the model is not static,
        add an edge connecting this vertex to the
        vertex of the model's canonical link.

    6.3 Add vertices for the implicit frame of each joint with an edge
        connecting from the joint to the vertex of its child link.

    6.4 For each `//model/frame`:

    6.4.1 Add a vertex to the graph.

    6.4.2 If `//model/frame/@attached_to` exists and is not empty,
          add an edge from the added vertex to the vertex
          named in the `//model/frame/@attached_to` attribute.

    6.4.3 Otherwise (ie. if the `//model/frame/@attached_to` attribute
          does not exist or is an empty string `""`),
          add an edge from the added vertex to the model frame vertex.

    6.5 Verify that the graph has no cycles and that by following the directed
        edges, every vertex is connected to a link.
        To identify the link to which each frame is attached, start from the
        vertex for that frame, and follow the directed edges until a link
        is reached.

7.  ***Check `//pose/@relative_to` attribute values:***
    For each `//pose` that is not `//model/pose` (ie. `//link/pose`,
    `//joint/pose`, `//frame/pose`, `//collision/pose`, `//light/pose`, etc.),
    if the `relative_to` attribute exists and is not an empty string `""`,
    check that the value of the `relative_to` attribute
    matches the name of a link, joint, or frame in this model's scope.

8.  ***Check `//pose/@relative_to` graph:***
    Construct a `relative_to` directed graph for the model with each vertex
    representing a frame:

    8.1 Add a vertex for the implicit model frame `__model__`.

    8.2 Add vertices for each `//model/link`, `//model/joint`, and
        `//model/frame`.

    8.3 For each `//model/link`:

    8.3.1 If `//link/pose/@relative_to` exists and is not empty,
          add an edge from the link vertex to the vertex named in
          `//link/pose/@relative_to`.

    8.3.2 Otherwise (ie. if `//link/pose` or `//link/pose/@relative_to` do not
          exist or `//link/pose/@relative_to` is an empty string `""`)
          add an edge from the link vertex to the implicit model frame vertex.

    8.4 For each `//model/joint`:

    8.4.1 If `//joint/pose/@relative_to` exists and is not empty,
          add an edge from the joint vertex to the vertex named in
          `//joint/pose/@relative_to`.

    8.4.2 Otherwise (ie. if `//joint/pose` or `//joint/pose/@relative_to` do not
          exist or `//joint/pose/@relative_to` is an empty string `""`)
          add an edge from the joint vertex to
          the child link vertex named in `//joint/child`.

    8.5 For each `//model/frame`:

    8.5.1 If `//frame/pose/@relative_to` exists and is not empty,
          add an edge from the frame vertex to the vertex named in
          `//frame/pose/@relative_to`.

    8.5.2 Otherwise if `//frame/@attached_to` exists and is not empty
          (ie. if `//frame/@attached_to` exists and is not an empty string `""`
          and one of the following is true: `//frame/pose` does not exist,
          `//frame/pose/@relative_to` does not exist, or
          `//frame/pose/@relative_to` is an empty string `""`)
          add an edge from the frame vertex to the vertex named in
          `//frame/@attached_to`.

    8.5.3 Otherwise (ie. if neither `//frame/@attached_to` nor
          `//frame/pose/@relative_to` are specified)
          add an edge from the frame vertex to the implicit model frame vertex.

    8.6 Verify that the graph has no cycles and that by following the directed
        edges, every vertex is connected to the implicit model frame.
        Other poses in the model such as `//collision/pose` and `//light/pose`
        do not need to be checked for cycles since they do not create
        implicitly named frames.

### 2 World

This section describes phases for parsing the kinematics of an SDFormat 1.7 world.
Several of these phases are similar to the phases of parsing an SDFormat 1.4
world in the [Legacy behavior documentation](/tutorials?tut=pose_frame_semantics).
In phases that differ from that document, *italics* are used to signal the difference.
For new phases, the ***Title:*** is italicized.

There are *seven* phases for validating the kinematics data in a world:

1.  **XML parsing and schema validation:**
    Parse world file from XML into a tree data structure,
    ensuring that there are no XML syntax errors and that the XML
    data complies with the [schema](http://sdformat.org/schemas/root.xsd).
    Schema `.xsd` files are generated from the `.sdf` specification files
    when building `libsdformat` with the
    [xmlschema.rb script](https://bitbucket.org/osrf/sdformat/src/sdformat6_6.2.0/tools/xmlschema.rb).

2.  **Name attribute checking:**
    Check that name attributes are not an empty string `""`,
    *that they are not reserved (*`__.*__` *or* `world`*)* and that sibling
    elements of *any* type have unique names.
    This check can be limited to `//world/model/@name`
    *and `//world/frame/@name`*
    since other names will be checked in the following step.
    This step is distinct from validation with the schema because the schema
    only confirms the existence of name attributes, not their content.

3.  **Model checking:**
    Check each model according to the *seven* phases of parsing kinematics of an
    sdf model.

4.  ***Check `//world/frame/@attached_to` attribute values:***
    For each `//world/frame`, if the `attached_to` attribute exists and is not
    an empty string `""`, check that the value of the `attached_to` attribute
    matches the name of a sibling model or frame.
    The `//frame/@attached_to` value must not match `//frame/@name`,
    as this would cause a graph cycle.

5.  ***Check `//world/frame/@attached_to` graph:***
    Construct an `attached_to` directed graph for the world with each vertex
    representing a frame:

    5.1 Add a vertex for the implicit world frame `world`.

    5.2 Add a vertex for each model in the world.

    5.3 For each `//world/frame`:

    5.3.1 Add a vertex to the graph.

    5.3.2 If `//world/frame/@attached_to` exists and is not empty,
          add an edge from the added vertex to the vertex named in the
          `//world/frame/@attached_to` attribute.

    5.3.3 Otherwise (ie. if the `//world/frame/@attached_to` attribute
          does not exist or is an empty string `""`),
          add an edge from the added vertex to the implicit world frame vertex.

    5.4 Verify that the graph has no cycles and that by following the directed
        edges, every vertex is connected to a model or the implicit world frame.
        If the directed edges lead from a vertex to the implicit world frame,
        then the `//world/frame` corresponding to that vertex is a fixed
        inertial frame.
        If the directed edges lead to a model, then the `//world/frame`
        corresponding to that vertex is attached to the implicit model frame
        of that model.

6.  ***Check `//pose/@relative_to` attribute values:***
    For each `//model/pose` and `//world/frame/pose`
    if the `relative_to` attribute exists and is not an empty string `""`,
    check that the value of the `relative_to` attribute
    matches the name of a model or frame that is a sibling of the element
    that contains the `//pose`.

7.  ***Check `//pose/@relative_to` graph:***
    Construct a `relative_to` directed graph for the model with each vertex
    representing a frame:

    7.1 Add a vertex for the implicit world frame.

    7.2 Add vertices for each `//world/model` and `//world/frame`.

    7.3 For each `//world/model`:

    7.3.1 If `//world/model/pose/@relative_to` exists and is not empty,
          add an edge from the model vertex to the vertex named in
          `//world/model/pose/@relative_to`.

    7.3.2 Otherwise (ie. if `//world/model/pose` or
          `//world/model/pose/@relative_to` do not
          exist or `//world/model/pose/@relative_to` is an empty string `""`)
          add an edge from the model vertex to the implicit world frame vertex.

    7.4 For each `//world/frame`:

    7.4.1 If `//frame/pose/@relative_to` exists and is not empty,
          add an edge from the frame vertex to the vertex named in
          `//frame/pose/@relative_to`.

    7.4.2 Otherwise if `//frame/@attached_to` exists and is not empty
          (ie. if `//frame/@attached_to` exists and is not an empty string `""`
          and one of the following is true: `//frame/pose` does not exist,
          `//frame/pose/@relative_to` does not exist, or
          `//frame/pose/@relative_to` is an empty string `""`)
          add an edge from the frame vertex to the vertex named in
          `//frame/@attached_to`.

    7.4.3 Otherwise (ie. if neither `//frame/@attached_to` nor
          `//frame/pose/@relative_to` are specified)
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
