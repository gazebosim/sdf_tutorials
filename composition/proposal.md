# Model composition: Proposed behavior

## Document summary

The proposal includes the following sections:

*   Motivation: background and rationale.
*   Proposed changes: Each additon / subtraction to SDFormat.
*   Examples: Long form code samples.

## Motivation

When constructing models of robots, composition / nesting is extremely useful
when making nontrivial assemblies, and especially when handling different
versions of worlds or robots (e.g. adding multiple instances of a robot, or
swapping out grippers).

As described in the [legacy behavior tutorial](/tutorials?tut=spec_world),
SDFormat 1.6 has the `//include` tag which accommodates these use cases, but it
does not provide guidance on modularity / encapsulation, and by the same token
does not strictly define how relationships among assemblies should be defined.

<!--
TODO: At what scope could parsing stages be added, where custom model formats
could be loaded? May be very difficult...
-->

## Proposed changes

### Usability: Permit direct files in `//include/uri`

Specifying a directory permits usage of `model.config` manifests, which permits
better compatibilty for a model when being loaded by software with different
SDFormat specification support. However, it then requires overhead that may not
matter for some applications.

This proposal suggests that `//include/uri` permits referencing files directly.

### Bottom-Up Encapsulation for Files

To enable "bottom-up" assemblies (or piece part design) to maximize modularity,
individual files should generally be viewed as standalone: all references in
the file should only refer to items "under" that file (e.g. links, joints, or
frames defined in the file, or links, joints, or frames defined in included
files). This entails that:

* Individual files should *not* be able to have a deferred reference to
something that isn't defined in that file (like Python modules).
* In conjunction with the [pose frame semantics proposal](/tutorials?tut=pose_frame_semantics_proposal),
all initially specified `//pose` elements within a file should be rigidly
related to one another, and should not be able to have their poses mutated by
an external site (to simplify pose resolution).

#### Do not permit overriding canonical link

`<include/>` *should not* be able to override canonical link.

### Encourage using models as an "API": abstraction and conventions

Assemblies require interfaces, and those interfaces should be able to conceal internal details while still providing useful abstractions, such as welding
points when swapping out components.

As such, you should consider levels of abstraction; use frames frequently,
especially for mounting points. If that mounting point physically moves at some
point, you update your model, and any downstream references can be left
untouched.

If you need to use intermediate frames that are not intended to be used for
public consumption, please prefix them with a single `_`, like Python.

#### Permit `//joint/parent` and `//joint/child` to refer to frames

This allows joints (like welds) to be abstracted, and thus can simplify
swapping out components.

**WARNING**: This would motivate preserving frames through saving SDFormat
files via Gazebo / libsdformat, esp. if they become the "API".

### Posturing frame for `//include/pose`.

It is useful to place an object using a semantic relationship between two
objects, e.g. place the bottom-center of a mug upright on the top-center of a
table.

This can be achieved by specifying `//include/posture_frame`, e.g.:

~~~
<include>
    <uri>file://table.sdf</uri>
    <posture_frame>bottom_left_log</posture_frame>
    <pose/>
</include>
<include>
    <uri>file://mug.sdf</uri>
    <posture_frame>bottom_center</posture_frame>
    <pose relative_to="table/top_center"/>
</include>
~~~

# **TODO**: Need to work on following text!

### Namespacing

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
* `//frame/@attached_to` can cross
* `//frame/@attached_to` *can* refer to links outside of a given model, as long
as it
* `//model/@canonical_link` *cannot* cross model boundaries
* The default `//pose/@relative_to` will refer to the *closest* enclosing
`//model`, not the file.
 
## Examples


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
    <include file="arm.sdf">
        <uri>arm.sdf</uri>
    </include>
    <include>
        <uri>gripper.sdf</uri>
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
    <include>
        <uri>arm.sdf</uri>
    </include>
    <include>
        <uri>gripper_with_weld.sdf</uri>
    </include>
</model>
~~~

### Simple Cross-Referencing

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

### Robot Arm with Gripper

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
