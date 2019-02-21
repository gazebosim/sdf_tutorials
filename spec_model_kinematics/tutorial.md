# Specifying model kinematics in SDFormat

This tutorial describes how to use SDFormat to model the kinematics of
articulated multibody systems with the `<model>`, `<link>`, `<joint>`,
and `<pose>` tags, which can be briefly summarized as:

* `<link>`: a rigid body (like a "link" in a chain)
* `<joint>`: a kinematic relationship between links
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
For example, an empty model with no links or joints:

    <model name="empty" />

A model can define a pose that will offset the model frame relative
to its parent when it is inserted into a world:

    <model name="model_pose_Z">
      <pose>0 0 1 0 0 0</pose>
    </model>

## `<link>`

The `<link>` tag represents a named rigid body and must be the child of a `<model>`.
Its full documentation can be found
[here](http://sdformat.org/spec?ver=1.4&elem=link).

It is also required for the link to be named using the `name` attribute.
For example, a model with one link:

    <model name="one_link">
      <link name="link"/>
    </model>

A model may contain multiple links with unique names:

    <model name="two_links">
      <link name="link1"/>
      <link name="link2"/>
    </model>

A model with multiple links having the same name is invalid:

    <model name="invalid_two_links_same_name">
      <link name="link"/>
      <link name="link"/>
    </model>

The link `<pose>` tag will be interpreted as a coordinate transform applied
relative to its model frame.
When inserted into a world, the link pose relative to the world is identical
for the following two models:

    <model name="model_and_link_pose">
      <pose>1 0 0 0 0 0</pose>
      <link name="link">
        <pose>0 1 0 0 0 0</pose>
      </link>
    </model>

    <model name="equivalent_link_pose">
      <link name="link">
        <pose>1 1 0 0 0 0</pose>
      </link>
    </model>

## `<joint>`

The `<joint>` tag represents a kinematic relationship between rigid body links
that constrains the degrees of freedom between those links.
It must be a child of a model.
Its full documentation can be found
[here](http://sdformat.org/spec?ver=1.4&elem=joint).

The supported joint types are listed below along with the number of degrees
of freedom remaining between the two links.

* `ball`: 3 rotational degrees of freedom
* `continuous`: 1 rotational degree of freedom with joint limits
* `fixed`: 0 degrees of freedom
* `prismatic`: 1 translational degree of freedom
* `revolute`: 1 rotational degree of freedom with joint limits
* `screw`: 1 coupled rotational/translational degree of freedom
* `universal`: 2 rotational degrees of freedom

