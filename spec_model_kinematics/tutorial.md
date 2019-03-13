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
* `continuous`: 1 rotational degree of freedom with no joint limits
* `fixed`: 0 degrees of freedom
* `prismatic`: 1 translational degree of freedom
* `revolute`: 1 rotational degree of freedom with joint limits
* `screw`: 1 coupled rotational/translational degree of freedom
* `universal`: 2 rotational degrees of freedom

The parent and child links of a joint are specified by the `<parent>` and
`<child>` tag respectively. These tags refer to other links inside the model by
their names. For example, the following model contains two links and a fixed
joint between the links:

    <model name="two_links_fixed">
      <link name="link1"/>
      <link name="link2"/>
      <joint name="joint1" type="fixed">
        <parent>link1</parent>
        <child>link2</child>
      </joint>
    </model>

A fixed joint connects two links such that all six degrees of freedom between
the links are constrained. In essence, the two links become one rigid body.

Additionally, a `<joint>` tag may contain a `<pose>` tag that specifies the
coordinate transform of the joint relative to the child link frame. This
transform is considered to be the transform between the two links when the
joint is at its initial position. It also determines the relationship between
a given joint position and the resulting transform between the links. While
a `<pose>` tag is not necessary for a fixed joint, it is used regularly
in other joint types.

In the following two examples, a revolute joint is used to demonstrate the use
of the `<pose>` tag. In these examples, the joint pose is set so that at
a joint position of 0, the two links are orthogonal to each other. In the first
example, the joint position is set to `0 0 -0.1`. Since the pose is specified
relative to the child link (link2), the position of the joint in the world
frame is `0.1 0 0`.

    <model name="two_links_orthogonal_1">
      <link name="linkA">
        <pose>0 0 0 0 0 0</pose>
      </link>
      <link name="linkB">
        <pose>0.1 0 0.1 0 0 0</pose>
      </link>
      <joint name="jointA" type="revolute">
        <pose>0 0 -0.1 0 0 0</pose>
        <parent>linkA</parent>
        <child>linkB</child>
        <axis>
          <xyz>0 1 0</xyz>
        </axis>
      </joint>
      <joint name="joint_world" type="fixed">
        <parent>world</parent>
        <child>linkA</child>
      </joint>
    </model>

In the second example, the joint position is set to `0 -0.1 0 `. The position
of the joint in the world frame is `0 0 0.1`.

    <model name="two_links_orthogonal_2">
      <link name="link1">
        <pose>0 0 0 0 0 0</pose>
      </link>
      <link name="link2">
        <pose>0.1 0 0.1 0 0 0</pose>
      </link>
      <joint name="joint1" type="revolute">
        <pose>-0.1 0 0.0 0 0 0</pose>
        <parent>link1</parent>
        <child>link2</child>
        <axis>
          <xyz>0 1 0</xyz>
        </axis>
      </joint>
      <joint name="joint_world" type="fixed">
        <parent>world</parent>
        <child>link1</child>
      </joint>
    </model>

In both examples, the `<axis>` tag is used to specify the axis of rotation of
the revolute joint. Once again, this axis is specified relative to the child
link frame.

The initial configuration of the articulated bodies, i.e, at a joint value of
0 radians, is shown in the following diagram. Note that the pose of linkB and
link2 is the same in both models.

[[file:revolute_joint_1a.svg|600px]]

However, a joint value of 0.78 radians (45 degrees) results in two very
different poses for linkB and link2.

[[file:revolute_joint_1b.svg|600px]]
