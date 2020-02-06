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
These should really be names and frames (implicit and explicit), and
transitively, the links they refer to.

### 1 Nesting and Encapsulation

#### 1.1 Standalone Components Individual Files

To enable "bottom-up" assemblies (or piece part design) to maximize modularity,
individual files should generally be viewed as standalone: all references in
the file should only refer to items "under" that file (e.g. links, joints, or
frames defined in the file, or links, joints, or frames defined in included
files). More explicitly:

* Individual files should *not* be able to have a deferred reference to
something that isn't defined in that file (like Python modules).
* In conjunction with the [pose frame semantics proposal](/tutorials?tut=pose_frame_semantics_proposal),
all initially specified `//pose` elements within a file should be rigidly
related to one another, and should not be able to have their poses mutated by
an external site (to simplify pose resolution).

#### 1.2 Interface Elements

Assemblies require interfaces, and those interfaces should be able to conceal internal details while still providing useful abstractions, such as welding
points when swapping out components.

As such, you should consider levels of abstraction and interface. In this case,
frames will be in the main interface point.

This permits a user to specify a mounting point as an interface, move that
within the model, and not have to update other downstream components.

##### 1.2.1 Frame Naming Suggestion

If you need to use intermediate within frames that are not intended to be used
for public consumption, please prefix them with a single `_`, like Python.

##### 1.2.2 Scope of Interface Elements

To avoid the complication of inter-element ambiguity, or multiple levels of
scope resolution, all interface elements within an immediate `//model` should be
referencable by only *one* level of nesting. If there are two levels of nesting for a name (e.g. `a::b::frame`), then `a` and `b` will be models. `b` will
never be a link or a visual or anything else.

##### 1.2.3 `//joint/parent` and `//joint/child` become frames, not just links

Assuming that assembly happens either by posturing (e.g. `//pose/@relative_to`),
attaching (`//frame/@attached_to`), or joints (`//joint`), then it would be
ideal to have all of these items refer to frames. `//pose` and `//frame`
already refer to frames, so admitting joints to refer to frames would also
simplify things.

This allows easier swapping out of components.

**WARNING**: This would motivate preserving frames through saving SDFormat
files via Gazebo / libsdformat, esp. if they become the "API".

#### 1.3 Name Scoping and Cross-Referencing

#### 1.3.1 Basics

Syntax:

* There is no "implicit name" resolution; if a name does not exist in a
requested scope, it is an error.
* Cross-referencing is only allowed from sibling elements *in the same file*.
* There are only two types of references allowed, current scope and absolute
scope. Relative scopes are *not* permitted.
* Scopes are syntax-based, i.e. defined by file or element composition. Welding
joints, affixing frames, etc., does not change the scope of an element.
* Scopes via composition are defined by the *instantiated* (or overriden) model
name, not the model name specified by the included file.

Semantics:

* `//joint/parent` and `//joint/child` can cross model boundaries
* `//frame/@attached_to` can cross
* `//frame/@attached_to` *can* refer to links outside of a given model, as long
as it
* `//model/@canonical_link` *cannot* cross model boundaries
* The default `//pose/@relative_to` will refer to the *closest* enclosing
`//model`, not the file.
 
#### 1.3.2 Model Frame Cross-Reference

For a model named `{name}`, the only way to refer to the model frame is by
specifying `{name}::__model__`. Referring to `{name}` is invalid.

This implies that for a name like `a::b`, `a` is a model, `b` is a frame. For a
name like `a::b::c`, `a` and `b` are models, and `c` is the frame.

#### 1.4 Posturing Frame for `//include/pose`

It is useful to place an object using a semantic relationship between two
objects, e.g. place the bottom-center of a mug upright on the top-center of a
table.

This can be achieved by specifying `//include/pose/@posture_frame`, e.g.:

~~~
<include>
    <uri>file://table.sdf</uri>
    <pose posture_frame="bottom_left_log"/>
</include>
<include>
    <uri>file://mug.sdf</uri>
    <pose posture_frame="bottom_center" relative_to="table/top_center"/>
</include>
~~~

##### Do not permit overriding canonical link

`<include/>` *should not* be able to override canonical link.

#### 1.5 Minimal Interface Types for Non-SDFormat Models

... Suggested API goes here.

#### 1.6 Extras

Need to sort these out...

##### Usability: Permit direct files in `//include/uri`

Specifying a directory permits usage of `model.config` manifests, which permits
better compatibilty for a model when being loaded by software with different
SDFormat specification support. However, it then requires overhead that may not
matter for some applications.

This proposal suggests that `//include/uri` permits referencing files directly.

### Examples

TODO: These are currently old.

#### Weld Arm and Gripper

If composing and welding an arm and a gripper:

~~~xml
<!-- arm.sdf -->
<model name="arm">
    <link name="body"/>
</model>
~~~

~~~xml
<!-- gripper.sdf -->
<model name="gripper">
    <link name="body"/>
</model>
~~~

~~~xml
<model name="arm_and_gripper">
    <include file="arm.sdf">
        <uri>arm.sdf</uri>
    </include>
    <include>
        <uri>gripper.sdf</uri>
        <pose frame="arm">{X_AG}</pose>
    </include>
    <joint name="weld" type="fixed">
        <parent>arm::body</parent>
        <child>gripper::body</child>
    </joint>
</model>
~~~

#### Negative Example

You cannot achieve the above by defining the weld in the gripper itself:

~~~xml
<!-- arm.sdf: Same as above. -->
~~~

~~~xml
<!-- gripper_with_weld.sdf -->
<model name="gripper">
    <pose>{X_AG}</pose>
    <link name="gripper">
    <joint name="weld" type="fixed">
        <parent>arm::body</parent> <!-- INVALID: Does not exist in this file -->
        <child>gripper::body</child>
    </joint>
</model>
~~~

~~~xml
<model name="arm_and_gripper">
    <include>
        <uri>arm.sdf</uri>
    </include>
    <include>
        <uri>gripper_with_weld.sdf</uri>
    </include>
</model>
~~~

#### Simple Cross-Referencing

Consider the following example model, all defined in a single file:

~~~xml
<model name="parent">
    <link name="link1"/>
    <link name="link2"/>

    <model name="child1">
        <link name="link1">
            <pose frame="::link1"/>  <!-- VALID: Refers to parent's link1 -->
            <pose frame="link1"/>  <!-- INVALID: Circular -->
        </link>
        <frame name="some_frame">
            <pose frame="link2"/>  <!-- INVALID: link2 does not exist in /child1 scope -->
        </frame>
    </model>

    <model name="child2">
        <link name="link1">
            <pose frame="::child1::link1"/>  <!-- VALID: Refers to child1's link1 -->
        </link>
    </model>
</model>
~~~

This implies that for scoping, it is *extremely* important that the parser to
know that it's working with a single model file.

#### Robot Arm with Gripper

Frames:

* `L` - arm/link
* `F` - flange origin
* `G` - gripper physical origin
* `Gm` - gripper model origin
    * For sake of artificial complexity (specification edge cases), will
    not coincide with `G`.

Files:

~~~xml
<!-- arm.sdf -->
<model name="arm">
    <link name="link"/>
    <frame name="flange_fixture">
      <pose frame="link">{X_LF}</pose>
    </frame>
</model>
~~~

~~~xml
<!-- flange_electric.sdf -->
<model name="flange">
    <link name="body"/>
    <frame name="gripper_origin">
      <pose frame="body">{X_FG_electric}</pose>
    </frame>
</model>
~~~

~~~xml
<!-- flange_pneumatic.sdf -->
<model name="flange">
    <link name="body"/>
    <frame name="gripper_origin">
      <pose frame="body">{X_FG_pneumatic}</pose>
    </frame>
</model>
~~~

~~~xml
<!-- gripper.sdf -->
<model name="gripper">
    <link name="gripper"/>
    <frame name="origin">
        <pose>{X_GmG}</pose>
    </frame>
</model>
~~~

Proposed welding semantics, with somma dat nesting:

```xml
<model name="super_armio_bros">
<!-- N.B. This could also be defined as a //world element. -->

    <!-- Arm + Electric Flange + Gripper -->
    <model name="robot_1">
        <include file="arm.sdf">
            <name>arm</name>
            <pose>{X_MR1}</pose>
        </include>
        <include>
            <uri>file://flange_electric.sdf</uri>
            <pose frame="arm/flange_origin"/>
        </include>
        <include file="gripper" pose_model_frame="origin">
            <pose frame="flange/gripper_origin"/>
        </include>
    </model>

    <!-- Arm + Pneumatic Flange + Gripper
         For fun, attached to end of above arm's gripper. -->
    <model name="robot_2">
        <include file="arm.sdf">
            <name>arm</name>
            <pose frame="/robot_1/gripper">{X_G1R2}</pose>
        </include>
        <include file="flange_electric">
            <pose frame="arm/flange_fixture"/>
        </include>
        <include file="gripper" pose_model_frame="origin">
            <pose frame="flange/gripper_fixture"/>
        </include>
    </model>

    <joint name="cute_weld">
        <parent>robot_1/gripper</parent>
        <child>robot_2/link</child>  <!-- Or canonical frame? -->
        <!-- <child>robot_2</child> - Alternative? -->
    </joint>

</model>
```
