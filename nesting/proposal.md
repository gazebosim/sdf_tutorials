# WIP Model Nesting: Proposal

TODO(eric): Ensure that we have overarching goals for compatiblity but new
functionality, with possible changes...

At present, SDFormat adds an `<include/>` tag. However, it has the following
design issues:

1.   Encapsulation: There is no specification on how encapsulated an included
file should be, and at what stage of construction inclusion is permitted.
    * Can the included file refer to joints that it does not define?
        * Example: Adding a gripper to an IIWA. There may be a weld pose based on the pneumatic or electric flange.
    * It is useful to encode frame offsets.
2.   There is no explicit specification about namespacing, relative references or absolute references, etc.
    * This is being addressed in the [Pose Frame Semantics Proposal PR](https://bitbucket.org/osrf/sdf_tutorials/pull-requests/7/pose-frame-semantics-proposal-for-new/diff)
3.   SDFormat requires that included models be converted to SDFormat (e.g. including an URDF). This may prevent software packages from doing more complex composition where there may not be an exact mapping to SDFormat, or where this conversion is excessive overhead.

## Encapsulation

For this aspect, the goal of this proposal is meant to permit adding models
either inside of a file, or after other models have been processed.

* This is achievable with current implementation of Gazebo, and with
partially-conformant implementation of Drake. However, with new proposals
for `//pose[@frame]` semantics, and the desire to stick with model-absolute
coordinates, this may become more nuanced when referring to frames in a
model.
* This permits SDFormat to be a *non-viral* format, e.g. a library developer
does not have to try and implement a mechanism to convert to some holisitc
SDFormat IR, or their own incantation thereof.

### Model File Completeness

A program should be able to load any *single file* on its own, with its declared dependencies being present (like Python modules).

All initially specified `//pose` elements will be rigidly related to one
another. You cannot refer to elements that do not exist inside that file.

The goal here here is to keep things simple from a parsing perspective,
especially for using model-absolute coordinate specification.

### Positive Example

If composing and welding an arm and a gripper:

    ~~~
    <-- arm.sdf -->
    <model name="arm">
        <link name="body"/>
    </model>
    ~~~

    ~~~
    <-- gripper.sdf -->
    <model name="gripper">
        <link name="body"/>
    </model>
    ~~~

    ~~~
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

    ~~~
    <-- arm.sdf: Same as above. -->
    ~~~

    ~~~
    <-- gripper_with_weld.sdf -->
    <model name="gripper">
        <pose>{X_AG}</pose>
        <link name="gripper">
        <joint name="weld" type="fixed">
            <parent>arm/body</parent> <!-- INVALID: Does not exist in this file -->
            <child>gripper/body</child>
        </joint>
    </model>
    ~~~

    ~~~
    <model name="arm_and_gripper">
        <include file="arm.sdf"/>
        <include file="gripper_with_weld.sdf"/>
    </model>
    ~~~

<!-- TODO(eric): Will this mess up Anzu workflows? Should there be some sort of
     specification welding? Perhaps along the lines of kinematic models? -->

## Naming Semantics

`<insert text from pro-pose-al>`

### Element Nesting

Within a single `//model`, there should only be *one* level of nesting. Thus,
there is no complication of inter-element ambiguity, or multiple levels of
scope resolution.

*TODO(eric): Update this if we permit //link/frame... but the more I write, the
more I strongly dislike adding complexity solely for the sake of sugar...*

For example, an atlernative formulation to the abstract frame placement, placing
`X_PJp` relative to the link element.

    <model name="abstract_joint_frames">
      <link name="parent"/>
      <frame name="parent_j1" affixed_to="parent">
        <pose>{X_PJp}</pose>
      </frame>
      <joint name="joint1">
        <pose frame="parent/j1"/>
        <parent>parent</parent>
        <child>child</child>
      </joint>
      <link name="child">
        <pose frame="joint1">{X_JcC}</pose>
        <!-- Or: <pose frame="parent/j1">{X_JcC}</pose> -->
      </link>
    </model>

**TODO**: See discussion: https://bitbucket.org/osrf/sdf_tutorials/pull-requests/14/pose-frame-semantics-suggested-semantics/activity#comment-100143077

*   Determine explicit semantics about references for `//link/frame`

### Model Nesting: Cross-`//model` referencing

For a single file that has nested models, each model can reference each other's
frames, links, or joints:

*TODO(eric): Make example of this. This will inform what the complications
might be for //link/frame.*

## Example: Robot Arm with Gripper

```xml
<!-- Frames
    * L - arm/link
    * F - flange origin
    * G - gripper physical origin.
        * Gm - gripper model origin. Will not be coincident with G.
-->

<!-- arm.sdf -->
<model name="arm">
    <link name="link"/>
    <frame name="flange_fixture">
      <pose frame="link">{X_LF}</pose>
    </frame>
</model>

<!-- flange_electric.sdf -->
<model name="flange">
    <link name="body"/>
    <frame name="gripper_origin">
      <pose frame="body">{X_FG_electric}</pose>
    </frame>
</model>

<!-- flange_pneumatic.sdf -->
<model name="flange">
    <link name="body"/>
    <frame name="gripper_origin">
      <pose frame="body">{X_FG_pneumatic}</pose>
    </frame>
</model>

<!-- gripper.sdf -->
<model name="gripper">
    <link name="gripper"/>
    <frame name="origin">
        <pose>{X_GmG}</pose>
    </frame>
</model>
```

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
            <!-- N.B. B/c both models live in same file, cross referencing is
                 fine... ??? -->
            <!-- Should this be forced to use a relative path?
                 e.g. ../robot_1/gripper? -->
            <pose frame="robot_1/gripper">{X_G1R2}</pose>
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

### Open Question: Overriding canonical link

`<include/>` *should not* be able to override canonical link.

### Open Question: Purely Kinematic Models

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
      <frame name="camera_001_depth" affixed_to="camera_001_rgb">
        <pose>{X_C1D}</pose>
      </frame>
      <!-- ... -->
    </model>

    <!-- Wrist cameras: `M` will be affixed to wrist -->
    <model name="camera_calibration_scene_fixed">
      <frame name="camera_011_rgb">
        <pose>{X_MC11}</pose>
      </frame>
      <frame name="camera_011_depth" affixed_to="camera_011_rgb">
        <pose>{X_C11D}</pose>
      </frame>
    </model>

*TODO(eric): As an alternative, make some sort of `//frame_group` tag?*
