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

A model can define a pose that offsets the model frame relative
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

The link `<pose>` tag is a coordinate transform applied relative to its model
frame.
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

There are several different types of joints that can be specified in the
`type` attribute.
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
      <joint name="joint" type="fixed">
        <parent>link1</parent>
        <child>link2</child>
      </joint>
    </model>

The joint `<pose>` tag is a coordinate transform applied relative to the
child link frame to define the joint frame.
This transform is considered to be the transform between the two links when the
joint is at its initial position.
While a `<pose>` tag is not necessary for a fixed joint, it is used regularly
in other joint types.
See the following example for an illustration of the joint pose.

Some joint types allow degrees of freedom along a specified axis.
For example, the rotational axis of a revolute joint is specified by a unit
Vector3 in the `<xyz>` tag under the `<axis>` element, which is interpreted
in the joint frame.
The joint pose and axis direction for the following SDF model are illustrated
in the following figure, with model frame `M`, Parent link frame `P`,
Child link frame `C`, and joint frame `J`.

    <model name="two_links_revolute">
      <link name="Parent">
        <pose>{xyz_MP} {rpy_MP}</pose>
      </link>
      <link name="Child">
        <pose>{xyz_MC} {rpy_MC}</pose>
      </link>
      <joint name="joint" type="revolute">
        <pose>{xyz_CJ} {rpy_CJ}</pose>
        <parent>Parent</parent>
        <child>Child</child>
        <axis>
          <xyz>{xyz_axis_J}</xyz>
        </axis>
      </joint>
    </model>

[[file:sdf_joint_frames.svg|600px]]

Note that coordinate frames are defined differently in URDF.
The kinematic topology of a URDF file must be a tree with no closed
kinematic loops, and frames are defined recursively along each chain of
links and joints.
As discussed in the
[previous tutorial](http://sdformat.org/tutorials?tut=specify_pose&cat=specification),
the `<origin>` tag is the URDF analog of the SDFormat `<pose>`.
The joint origin defines the transform from the parent link frame to the
joint frame, and the child link frame is co-located with the joint frame.
This is illustrated in the [URDF documentation](http://wiki.ros.org/urdf/XML/joint)
with the following image and a corresponding URDF snippet:

    <robot name="two_links_revolute">
      <link name="Parent"/>
      <link name="Child"/>
      <joint name="joint" type="revolute">
        <origin xyz="{xyz_PJ}" rpy="{rpy_PJ}" />
        <parent>Parent</parent>
        <child>Child</child>
        <axis xyz="{xyz_axis_J}" />
      </joint>
    </robot>

<img src="http://wiki.ros.org/urdf/XML/joint?action=AttachFile&do=get&target=joint.png"
     alt="urdf coordinate frames"
     height="500"/>

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
