# Model Composition: Proposal

## Motivation

At present, SDFormat has an `//include` tag. However, it may have issues with
encapsulation / "viral" format requirements and namespacing. These are
described below.

These are incorporated into a new suggested parsing order to acommodate these
motivations.

### Proposed Update: Phases of Parsing Composition of of SDFormat 1.7 Model

*TODO(eric): Write this.*

For the motivation of each of these stages, please see below.

## Encapsulation

Any set of models are effectively an "API". They has publicly referencable
elements (links, joints, frames, and possibly sub-models).

At present, there is no specification on how encapsulated an included file
should be, and at what stage of construction inclusion is permitted. Both of
these drive reusability of existing models, and what modifications are necessary
for composition.

Some example circumstances:

* Can the included file refer to joints that it does not define?
    * Example: Adding a gripper to an IIWA. There may be a weld pose based on the pneumatic or electric flange.
    * What if including a model in one context completely changes the topology
    from another context, unintentionally?
* How well does abstraction work?
    * Can links be renamed, and can aliased be left, possiblye with pose offsets?

Defining encapsulation could also pave the way for SDFormat to be a *non-viral*
format, e.g. a library developer can merge multiple formats (potentially
non-XML) together via SDFormat in code, without having to convert their models
to some sort of SDFormat IR (which may not provide sufficient feature
coverage), or have to reimplement / fork `libsdformat`.

### Guiding Motivation: Make Your Model an "API"

*TODO(eric): I want to present this, but can't think of a better location.*

As such, you should consider levels of abstraction; use frames frequently,
especially for mounting points. If that mounting point physically moves at some
point, you update your model, and any downstream references can be left
untouched.

*TODO(eric): For the abstraction argument, //joint/parent and //joint/child
should really be able to refer to a frame - then a frame can be used completely
for abstraction!*

If you need to use intermediate frames that are not intended to be used for
public consumption, please prefix them with a single `_`, like Python.

### Model File Completeness

A program should be able to load any *single file* on its own, with its declared dependencies being present (like Python modules).

All initially specified `//pose` elements will be rigidly related to one
another. You cannot refer to elements that do not exist inside that file.

The goal here here is to keep things simple from a parsing perspective,
especially for using model-absolute coordinate specification.

### Positive Example

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
    <include file="arm.sdf"/>
    <include file="gripper.sdf">
        <pose frame="arm">{X_AG}</pose>
    </include>
    <joint name="weld" type="fixed">
        <parent>arm/body</parent>
        <child>gripper/body</child>
    </joint>
</model>
~~~

### Negative Example

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
        <parent>arm/body</parent> <!-- INVALID: Does not exist in this file -->
        <child>gripper/body</child>
    </joint>
</model>
~~~

~~~xml
<model name="arm_and_gripper">
    <include file="arm.sdf"/>
    <include file="gripper_with_weld.sdf"/>
</model>
~~~

<!-- TODO(eric): Will this mess up Anzu workflows? Should there be some sort of
     specification welding? Perhaps along the lines of kinematic models? -->

## Namespacing: Syntax and Semantics

There is no explicit specification about namespacing, relative references or
absolute references, etc.

`<insert text from pro-pose-al>`

### Element Nesting

Within a single `//model`, there should only be *one* level of nesting. Thus,
there is no complication of inter-element ambiguity, or multiple levels of
scope resolution.

*TODO(eric): Update this if we permit //link/frame... but the more I write, the
more I strongly dislike adding complexity solely for the sake of sugar...*

For example, an atlernative formulation to the abstract frame placement, placing
`X_PJp` relative to the link element.

~~~xml
<model name="abstract_joint_frames">
  <link name="parent"/>
  <frame name="parent_j1" attached_to="parent">
    <pose>{X_PJp}</pose>
  </frame>
  <joint name="joint1">
    <pose frame="parent/j1"/>
    <parent>parent</parent>
    <child>child</child>
  </joint>
  <link name="child">
    <pose frame="joint1">{X_JcC}</pose>
    <!-- Or: <pose frame="parent_j1">{X_JcC}</pose> -->
  </link>
</model>
~~~

*TODO(eric): [See discussion](https://bitbucket.org/osrf/sdf_tutorials/pull-requests/14/pose-frame-semantics-suggested-semantics/activity#comment-100143077).
Determine explicit semantics about references for `//link/frame`.*

### Model Nesting: Cross-Referencing

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
* `//frame[@attached_to]` can cross
* `//frame[@attached_to]` *can* refer to links outside of a given model, as long
as it
* `//model[@canonical_link]` *cannot* cross model boundaries
* The default `//pose[@relative_to]` will refer to the *closest* enclosing
`//model`, not the file.

### Example: Simple Cross-Referencing

Consider the following example model, all defined in a single file:

~~~xml
<model name="parent">
    <link name="link1"/>
    <link name="link2"/>

    <model name="child1">
        <link name="link1">
            <pose frame="/link1"/>  <!-- VALID: Refers to parent's link1 -->
            <pose frame="link1"/>  <!-- INVALID: Circular -->
        </link>
        <frame name="some_frame">
            <pose frame="link2"/>  <!-- INVALID: link2 does not exist in /child1 scope -->
        </frame>
    </model>

    <model name="child2">
        <link name="link1">
            <pose frame="/child1/link1"/>  <!-- VALID: Refers to child1's link1 -->
        </link>
    </model>
</model>
~~~

This implies that for scoping, it is *extremely* important that the parser to
know that it's working with a single model file.

### Example: Robot Arm with Gripper

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
        <include file="flange_electric">
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

### Do not permit overriding canonical link

`<include/>` *should not* be able to override canonical link.

*TODO(eric): I completely forgot the motivation for this... This now seems a
wee bit dumb...*

## Open Questions

### Purely Kinematic Models?

This document may implicitly permit a model that is purely kinematic, or rather,
defined purely as frames, relative to the model frame.

Opinions:

* Eric: I am in favor of this, as it is useful for gripper or camera offsets
that may admit other welding afterwards.
    * For gripper offsets, it is true that these may be actual physical bodies;
    I am fine with them being as such, as long as people don't put crappy /
    useless inertial values there. If they do, then it's more or less abuse.
    * However: Due to the current state of using absolute coordinates, and thus
    only permitting frame welding via joints, there is no mechanism to support
    purely kinematic welding. We should really fix this.
    Perhaps if we define some mechanism to place a model's initial pose, and
    permit external specification of frame fixturing?

Motivating Example: Scene-fixed camera calibration results - frames only

    <!-- Scene cameras: `M` will be affixed to some world-affixed frame -->
    <model name="camera_calibration_scene_fixed">
      <frame name="camera_001_rgb">
        <pose>{X_MC1}</pose>
      </frame>
      <frame name="camera_001_depth" attached_to="camera_001_rgb">
        <pose>{X_C1D}</pose>
      </frame>
      <!-- ... -->
    </model>

    <!-- Wrist cameras: `M` will be affixed to wrist -->
    <model name="camera_calibration_scene_fixed">
      <frame name="camera_011_rgb">
        <pose>{X_MC11}</pose>
      </frame>
      <frame name="camera_011_depth" attached_to="camera_011_rgb">
        <pose>{X_C11D}</pose>
      </frame>
    </model>

*TODO(eric): As an alternative, make some sort of `//frame_group` tag?*

### Permit inlined `//include`?

In order to permit different levels of abstraction (e.g. for frame groups, or
for adding certain collision elements), should it be possible to `//include` an
element, but dump it into the current scope?

Kind of like `from my_module import *` in Python... which is recommended
against...

However, it *is* super useful to be able add a set of frames and welding
semantics... But that would break encapsulation? Should that just be deferred
to text processing???

## 
