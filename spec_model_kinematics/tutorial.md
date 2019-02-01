# Specifying model kinematics in SDFormat

This tutorial describes how to use SDFormat to model the kinematics of
articulated multibody systems with the `<model>`, `<link>`, `<joint>`,
and `<pose>` tags, which can be briefly summarized as:

* `<link>`: a rigid body (like a "link" in a chain)
* `<joint>`: an articulation constraint between links
* `<pose>`: the relative position and orientation between two coordinate frames
* `<model>`: a container for links and joints that defines a complete robot or physical object

## `<model>`

The model tag serves as a container for a group of links and joints.
It is required to specify a model name in the `name` attribute
    <model name="empty" />


`<link>`, `<joint>`,


The SDFormat `<model>` tag
It describes the

[previous tutorial](http://sdformat.org/tutorials?tut=specify_pose&cat=specification)

[official documentation of the model element](http://sdformat.org/spec?ver=1.6&elem=model)

<[model](http://sdformat.org/spec?ver=1.6&elem=model)>,
<[link](http://sdformat.org/spec?ver=1.6&elem=link)>,
<[joint](http://sdformat.org/spec?ver=1.6&elem=joint)>,
