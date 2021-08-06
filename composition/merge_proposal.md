# Merging for Composition: Proposed behavior

* **Authors**:
Eric Cousineau `<eric.cousineau@tri.global>`,
Addisu Taddese  `<addisu@openrobotics.org>`
* **Status**: Draft
* **SDFormat Version**: 1.9
* **`libsdformat` Version**: 12.0

## Introduction

This proposal suggests functional and semantic changes to how `//include` can
function with the usage of the `//include/@merge` attribute.

The `//include` tag was first introduced in SDFormat 1.4, and had its behavior
more fully specified in SDFormat 1.8 (see
[proposal](/tutorials?tut=composition_merge_proposal)). The new behavior
introduced in SDFormat 1.8 defines stricter encapsulation. However, changing a
model from inline elements to use composition will create user-visible changes
in how the elements are named and scoped.

This proposal provides a means to use composition to migrate / create models
using composition in a way that is less visible to downstream consumers, while
maintaining the encapsulation the SDFormat 1.8 proposal provides.

## Document summary

This proposal includes the following sections:

* Motivation: background and rationale.
* Proposed changes: The addition to the SDFormat specification and the
`libsdformat` implementation.
* Examples: Models and workflows using this features.

## Motivation

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
