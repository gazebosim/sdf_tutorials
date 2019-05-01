# WIP Model Nesting: Proposal

TODO(eric): Ensure that we have overarching goals for compatiblity but new
functionality, with possible changes...

At present, SDFormat adds an `<include/>` tag. However, it has the following
design issues:

1.   Encapsulation, Lifetime: There is no specification on how encapsulated an included file should be, and at what stage of construction inclusion is permitted.
    * Can the included file refer to joints that it does not define?
        * Example: Adding a gripper to an IIWA. There may be a weld pose based on the pneumatic or electric flange.
    * It is useful to encode frame offsets.
2.   There is no explicit specification about namespacing, relative references or absolute references, etc.
    * This is being addressed in the [Pose Frame Semantics Proposal PR](https://bitbucket.org/osrf/sdf_tutorials/pull-requests/7/pose-frame-semantics-proposal-for-new/diff)
3.   SDFormat requires that included models be converted to SDFormat (e.g. including an URDF). This may prevent software packages from doing more complex composition where there may not be an exact mapping to SDFormat, or where this conversion is excessive overhead.

## Lifetime

For this aspect, the goal of this proposal is meant to permit the following:

* Adding models *after other models have been processed*
    * This is achievable with current implementation of Gazebo, and with
    partially-conformant implementation of Drake. However, with new proposals
    for `//pose[@frame]` semantics, and the desire to stick with model-absolute
    coordinates, this may become more nuanced when referring to frames in a
    model.
    * This permits SDFormat to be a *non-viral* format, e.g. a library developer
    does not have to try and implement a mechanism to convert to some holisitc
    SDFormat IR, or their own incantation thereof.

## Encapsulation

In order to simplify, `<insert text from pro-pose-al>`

## Naming Semantics

`<insert text from pro-pose-al>`

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
            <name>amr</name>
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

### Open Questions

*   `<include/>` should or should not be able to override canonical frame?
    * Care should be taken to enusre this does not alter the affixed-to
    semantics of model-defined frames.
