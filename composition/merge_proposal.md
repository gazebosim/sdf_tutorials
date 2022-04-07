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
nested models to be composed using content from separate SDFormat model files.
The behavior of the `//include` tags was more fully specified in SDFormat 1.8
(see [proposal](/tutorials?tut=composition_merge_proposal)).
The SDFormat 1.8 specification enforces strict encapsulation of models within
a nested model hierarchy, such that an included model must not reference any
external frames or entities. Separate name scopes are defined for each model
to avoid name collisions.

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

TBD

## Examples

TBD

## Appendix
