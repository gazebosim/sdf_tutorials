# Specifying model kinematics in SDFormat

This tutorial describes how to use SDFormat to model the kinematics of
articulated multibody systems with the `<model>`, `<link>`, `<joint>`,
and `<pose>` tags, which can be briefly summarized as:

* `<link>`: a rigid body (like a "link" in a chain)
* `<joint>`: an articulation constraint between links
* `<pose>`: the relative position and orientation between two coordinate frames
* `<model>`: a container for links and joints that defines a complete robot or physical object

SDFormat links, joints, and models each have their own coordinate frames that
can be offset using the `<pose>` tag.
See the
[previous tutorial](http://sdformat.org/tutorials?tut=specify_pose&cat=specification)
about specifying poses for more detail on the `<pose>` tag.

## `<model>`

The `<model>` tag serves as a named container for a group of links and joints.
Its full documentation can be found
[here](http://sdformat.org/spec?ver=1.4&elem=model).

It is required to name the model using the `name` attribute.
For example, a model with no links or joints could be written as:

    <model name="empty" />

coordinate frame
`<link>`, `<joint>`,


The SDFormat `<model>` tag
It describes the


[official documentation of the model element](http://sdformat.org/spec?ver=1.6&elem=model)

<[model](http://sdformat.org/spec?ver=1.6&elem=model)>,
<[link](http://sdformat.org/spec?ver=1.6&elem=link)>,
<[joint](http://sdformat.org/spec?ver=1.6&elem=joint)>,
