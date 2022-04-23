# Merging for Composition: Proposed behavior

* **Authors**:
Eric Cousineau `<eric.cousineau@tri.global>`,
Addisu Taddese  `<addisu@openrobotics.org>`
Steve Peters `<scpeters@openrobotics.org>`
* **Status**: Draft
* **SDFormat Version**: 1.9
* **`libsdformat` Version**: 12.0

## Introduction

This proposal suggests a new behavior for the `//model/include` tag that copies
and merges the contents of a model file into the current model without adding
additional nested model hierarchy or naming scope. The new merging behavior is
used when the `//include/@merge` attribute is `true`.

In SDFormat 1.8 and earlier, composing an SDFormat model file from content
stored in separate files using `//model/include` requires each included model
to be fully encapsulated and creates a nested model hierarchy that mirrors
the file structure used to store the models.
This approach is guaranteed to avoid name collisions but constrains the
model structure to match the file structure and limits the flexibility of model
composition.

The proposed behavior decouples the model structures available via composition
from the file structure used to store the underlying model components.
This is useful both for creating new models and for decomposing existing models
into separate components without visibile changes to downstream consumers,
while maintaining the encapsulation provided by SDFormat 1.8.
The cost of this feature is that users must take care to avoid name collisions
between the entities of the models to be merged.

## Document summary

This proposal includes the following sections:

* Motivation: background and rationale.
* Proposed changes: The addition to the SDFormat specification and the
`libsdformat` implementation.
* Examples: Models and workflows using this features.

## Motivation

The `//world/include` tag was first introduced in SDFormat 1.4 to support
insertion of models into a world. This was the first way to compose an SDFormat
document using content from separate SDFormat files.
The `//model/model` tag was added in SDFormat 1.5 to allow a hierarchical
nesting of models and was accompanied by the `//model/include` tag to allow
nested models to be composed using content from separate SDFormat model files,
though the behavior was not entirely consistent
(see [documentation](/tutorials?tut=composition&ver=1.5)).
The behavior of nested models specified using `//model/model` and
`//model/include` was made consistent through improvements to the specification
in SDFormat 1.8
(see [composition proposal](/tutorials?tut=composition_merge_proposal)).
The SDFormat 1.8 specification allows a parent model to reference frames or
entities in nested child models but not the reverse.
This asymmetry enforces hierarchical encapsulation of models and ensures that
each model is fully defined by its own contents and those of its nested models.
Separate name scopes are defined for each model in the hierarchy to avoid name
collisions.

As mentioned in the introduction above, if a user wished to split a model into
separate components and then combine them via composition, those changes may now
become visible to downstream consumers.

For example, say a user had combinatorics of arm + flange + gripper
combinations, and wished to defer the combinatorics to SDFormat's `//include`
composition (instead of a text processing method like `xacro`) such that each
component could be used in composition or isolation. If the user made this
change, it could change the name of relevant interface elements, e.g. links,
joints, and frames. For example, what used to be `composite_arm::gripper_mount`
may now need to become something like `composite_arm::flange::gripper_mount`.

However, with the proposed feature, the user could choose to preserve the
naming at the sites of usage, e.g. `composite_arm::gripper_mount` from above.

## Proposed changes

### `//model/include/@merge`

This adds a new attribute, `@merge`, to `//model/include` tags that when set to
`true` changes the include behavior to insert the contents of the included
model directly into the parent model without introducing a new scope into the
model hierarchy. Some model elements are not merged: `//model/static`,
`//model/self_collide`, `//model/enable_wind`, and `//model/allow_auto_disable`.

<-- TODO: explain how poses work and the proxy _merged__<model_name>__model__ frame -->

## Examples

### Small example:

Given the parent model:

~~~
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="world_model">
    <model name="robot">
      <include merge="true">
        <uri>test_model</uri>
        <pose>100 0 0 0 0 0</pose>
      </include>
    </model>
  </world>
</sdf>
~~~

and the included model:

~~~
<sdf version="1.9">
  <model name="test_model">
    <link name="L1" />
    <frame name="F1" />
  </model>
</sdf>
~~~

The result would be:

~~~
<sdf version='1.9'>
  <world name='world_model'>
    <model name='robot'>
      <frame name='_merged__test_model__model__' attached_to='L1'>
        <pose relative_to='__model__'>100 0 0 0 -0 0</pose>
      </frame>
      <link name='L1'>
        <pose relative_to='_merged__test_model__model__'/>
      </link>
      <frame name='F1' attached_to='_merged__test_model__model__'/>
    </model>
  </world>
~~~

### Decomposing an existing model into separate model files

## Appendix
