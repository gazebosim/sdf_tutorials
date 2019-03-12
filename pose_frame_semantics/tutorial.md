# Pose Frame Semantics

In version 1.4 and earlier of the SDF spec, the `<pose>` element represents
a relative coordinate transformation between a frame and its parent.
Link frames were always defined relative to a model frame and joint frames
relative to their child link frames.

In version 1.5 of the SDF spec, the `frame` attribute string was added to
`<pose>` elements to allow poses to be defined relative to a named frame
instead of the default frames from version 1.4.
For example, this would allow an SDF model to define its kinematics like a
URDF, with joint frames defined relative to the parent link frame and
a joint's child link frames relative to the joint frame.

The 1.5 spec also adds `<frame>` elements which can define named coordinate
frames in addition to the existing link and joint frames in the model.

This document is a work in progress to define the semantics of the pose frame
attribute.

## Legacy behavior

### Element naming rules in sdf 1.4

In sdf 1.4, sibling elements of the same type must have unique names.
For example, the following models are invalid because links, joints, and
collisions with the same parent do not have unique names.

    <sdf version="1.4">
      <model name="model">
        <link name="link"/>
        <link name="link"/>
      </model>
    </sdf>

    <sdf version="1.4">
      <model name="model">
        <link name="link1"/>
        <link name="link2"/>
        <link name="link3"/>
        <joint name="joint" type="fixed">
          <parent>link1</parent>
          <child>link2</child>
        </joint>
        <joint name="joint" type="fixed">
          <parent>link2</parent>
          <child>link3</child>
        </joint>
      </model>
    </sdf>

    <sdf version="1.4">
      <model name="model">
        <link name="link">
          <collision name="collision">
            ...
          </collision>
          <collision name="collision">
            ...
          </collision>
        </link>
      </model>
    </sdf>

The following model contains collision elements with the same name, but
the models are valid because the elements are not siblings, but rather
children of different links.

    <sdf version="1.4">
      <model name="model">
        <link name="link1">
          <collision name="collision">
            ...
          </collision>
        </link>
        <link name="link2">
          <collision name="collision">
            ...
          </collision>
        </link>
      </model>
    </sdf>

Sibling elements of different types are not mandated to have unique names,
so the following is valid, though it is uncommon in practice.

    <sdf version="1.4">
      <model name="model">
        <link name="base"/>
        <link name="attachment"/>
        <joint name="attachment" type="fixed">
          <parent>base</parent>
          <child>attachment</child>
        </joint>
      </model>
    </sdf>

### Parent frames in sdf 1.4

With the exception of joint frames, all `<pose>` tags in sdf 1.4 define
a relative transform between the current element's frame and the frame of its
parent xml element.

For example, in the following model, the link frame is defined relative to
the model frame, and the inertial, collision, visual, and sensor frames
are all defined relative to the model frame.

    <sdf version="1.4">
      <model name="model">
        <pose>...</pose>
        <link name="link">
          <pose>...</pose>
          <inertial>
            <pose>...</pose>
          </inertial>
          <collision name="collision">
            <pose>...</pose>
          </collision>
          <visual name="visual">
            <pose>...</pose>
          </visual>
          <sensor name="sensor">
            <pose>...</pose>
          </sensor>
        </link>
      </model>
    </sdf>

The one exception is the joint frame, which is defined relative to the child
link frame. In the following example, the joint frame is defined relative
to the link frame of `link2`.

    <sdf version="1.4">
      <model name="model">
        <pose>...</pose>
        <link name="link1">
          <pose>...</pose>
        </link>
        <link name="link2">
          <pose>...</pose>
        </link>
        <joint name="joint" type="fixed">
          <pose>...</pose>
          <parent>link1</parent>
          <child>link2</child>
        </joint>
      </model>
    </sdf>

Given this longstanding behavior, this pattern should be followed for
sdf 1.5+ when the pose frame attribute is empty.
This would imply that the following models are equivalent to the previous
models discussed in this section:

    <sdf version="1.5">
      <model name="model">
        <pose frame=''>...</pose>
        <link name="link">
          <pose frame=''>...</pose>
          <inertial>
            <pose frame=''>...</pose>
          </inertial>
          <collision name="collision">
            <pose frame=''>...</pose>
          </collision>
          <visual name="visual">
            <pose frame=''>...</pose>
          </visual>
          <sensor name="sensor">
            <pose frame=''>...</pose>
          </sensor>
        </link>
      </model>
    </sdf>

and

    <sdf version="1.5">
      <model name="model">
        <pose frame=''>...</pose>
        <link name="link1">
          <pose frame=''>...</pose>
        </link>
        <link name="link2">
          <pose frame=''>...</pose>
        </link>
        <joint name="joint" type="fixed">
          <pose frame=''>...</pose>
          <parent>link1</parent>
          <child>link2</child>
        </joint>
      </model>
    </sdf>


### Parent frames in URDF

For comparison, the behavior of parent frames in Unified Robot Description
Format ([URDF](http://wiki.ros.org/urdf/XML/model)) is given in this section.
A URDF files contains links, joints, collisions, visuals, and inertials
like SDFormat, but with several significant differences.

The first difference is that coordinate transformations are expressed using
the attributes of the `<origin/>` tag instead of the value of `<pose/>`,
which is a superficial difference as the numerical contents of each tag have a
[similar definition](http://http://sdformat.org/tutorials?tut=specify_pose).

    <pose>x y z roll pitch yaw</pose>

is equivalent to

    <origin rpy='roll pitch yaw' xyz='x y z'/>

Similar to SDFormat, URDF inertial, collision, and visual elements are defined
relative to their parent link frame with their own `<origin/>` tag, but links
and robots (equivalent to models) do not have `<origin/>` tags:

    <sdf version="1.4">
      <model name="model">
        <link name="link">
          <inertial>
            <pose>...</pose>
          </inertial>
          <collision name="collision">
            <pose>...</pose>
          </collision>
          <visual name="visual">
            <pose>...</pose>
          </visual>
        </link>
      </model>
    </sdf>

is equivalent to

    <robot name="model">
      <link name="link">
        <inertial>
          <origin rpy='...' xyz='...'/>
        </inertial>
        <collision name="collision">
          <origin rpy='...' xyz='...'/>
        </collision>
        <visual name="visual">
          <origin rpy='...' xyz='...'/>
        </visual>
      </link>
    </model>

The most significant difference between URDF and SDFormat coordinate frames
is related to links and joints.
While SDFormat allows kinematic loops with the topology of a directed graph,
URDF kinematics must have the topology of a directed tree, with each link
being the child of at most one joint.
URDF coordinate frames are defined recursively based on this tree structure,
with each joint's `<origin/>` tag defining the coordinate transformation from
the parent link frame to the child link frame.

    <sdf version="1.4">
      <model name="model">
        <link name="link1">
          <pose>...</pose>
        </link>
        <link name="link2">
          <pose>...</pose>
        </link>
        <joint name="joint" type="fixed">
          <pose>...</pose>
          <parent>link1</parent>
          <child>link2</child>
        </joint>
      </model>
    </sdf>

is decidedly not equivalent to

    <robot name="model">
      <link name="link1"/>
      <link name="link2"/>
      <joint name="joint" type="fixed">
        <origin rpy='...' xyz='...'/>
        <parent>link1</parent>
        <child>link2</child>
      </joint>
    </robot>

The following image is used in
[URDF documentation](http://wiki.ros.org/urdf/XML/model)
to illustrate how coordinate frames are defined:

<img src="http://wiki.ros.org/urdf/XML/model?action=AttachFile&do=get&target=link.png"
     alt="urdf coordinate frames"
     height="500"/>

This model in this image could be represented by the following URDF:

    <robot name="model">

      <link name="link1"/>

      <joint name="joint1" type="...">
        <origin rpy='...' xyz='...'/>
        <parent>link1</parent>
        <child>link2</child>
      </joint>
      <link name="link2"/>

      <joint name="joint2" type="...">
        <origin rpy='...' xyz='...'/>
        <parent>link1</parent>
        <child>link3</child>
      </joint>
      <link name="link3"/>

      <joint name="joint3" type="...">
        <origin rpy='...' xyz='...'/>
        <parent>link3</parent>
        <child>link4</child>
      </joint>
      <link name="link4"/>
    </robot>

### Specifying parent and child link names for joints in sdf 1.4

Joints specify the parent and child links by name in the `<parent>` and
`<child>` elements.
The specified links must exist as siblings of the joint with one exception:
if one but not both of the parent and child links is specified as `world`
and a sibling link named `world` does not exist, the `world` link will be
interpreted as a fixed inertial frame.

The following model contains joint with a valid specification of sibling links.

    <sdf version="1.4">
      <model name="model">
        <link name="link1"/>
        <link name="link2"/>
        <joint name="joint" type="fixed">
          <parent>link1</parent>
          <child>link2</child>
        </joint>
      </model>
    </sdf>

The following models contain joints with a valid specification of one sibling
link connected to a fixed inertial frame as parent or child.

    <sdf version="1.4">
      <model name="model">
        <link name="link"/>
        <joint name="joint" type="fixed">
          <parent>world</parent>
          <child>link</child>
        </joint>
      </model>
    </sdf>

and

    <sdf version="1.4">
      <model name="model">
        <link name="link"/>
        <joint name="joint" type="fixed">
          <parent>link</parent>
          <child>world</child>
        </joint>
      </model>
    </sdf>

The following model contains a link named `world`, so the joint connects `link`
to its sibling rather than connecting to a fixed inertial frame.

    <sdf version="1.4">
      <model name="model">
        <link name="link"/>
        <link name="world"/>
        <joint name="joint" type="fixed">
          <parent>world</parent>
          <child>link</child>
        </joint>
      </model>
    </sdf>

The following model contains an invalid joint specification because the parent
link does not exist.

    <sdf version="1.4">
      <model name="model">
        <link name="link"/>
        <joint name="joint" type="fixed">
          <parent>fake_link</parent>
          <child>link</child>
        </joint>
      </model>
    </sdf>

The following world also contains an invalid joint specification because, while
`link1` does exist in the world, it is not a sibling of the joint.

    <sdf version="1.4">
      <world name="world_with_invalid_joint">
        <model name="model1">
          <link name="link1"/>
        </model>
        <model name="model2">
          <link name="link2"/>
          <joint name="joint" type="fixed">
            <parent>link1</parent>
            <child>link2</child>
          </joint>
        </model>
      </world>
    </sdf>

This section has discussed naming conventions for `<joint>` elements as
children of a `<model>`.
For completeness, it should be noted that the SDF specification allows for a
`<joint>` to be a direct child of a `<world>`
(see [world.sdf:58](https://bitbucket.org/osrf/sdformat/src/21d2cbe52bb/sdf/1.4/world.sdf#world.sdf-58)),
but the naming conventions for this case are not established, as this use case
is not supported by Gazebo or any other known software.


### Specifying parent and child link names for joints in sdf 1.5 with nested models

Support for nested models was added in sdf 1.5, which allows a `<model>`
element to contain child `<model>`s.
For example, the following model contains two links nested inside child models.

    <sdf version="1.5">
      <model name="model">
        <model name="model1">
          <link name="link"/>
        </model>
        <model name="model2">
          <link name="link"/>
        </model>
      </model>
    </sdf>

The coordinate frame of each child model is defined relative to its parent
element, following the convention from sdf 1.4.

A joint can specify the names of parent and child links from sibling models
by specifying the sibling model name with the delimiter `::` followed
by the link name.

This model contains a valid joint specification with parent and child
links from sibling models:

    <sdf version="1.5">
      <model name="model">
        <model name="model1">
          <link name="link"/>
        </model>
        <model name="model2">
          <link name="link"/>
        </model>
        <joint name="joint" type="fixed">
          <parent>model1::link</parent>
          <child>model2::link</child>
        </joint>
      </model>
    </sdf>

This model contains a valid joint specification with a child sibling link
and the parent link from a sibling model.

    <sdf version="1.5">
      <model name="model">
        <model name="nested_model">
          <link name="link"/>
        </model>
        <link name="link"/>
        <joint name="joint" type="fixed">
          <parent>nested_model::link</parent>
          <child>link</child>
        </joint>
      </model>
    </sdf>

TODO: figure out if a child model is allowed to have the same name as its
parent.

**Please note:** The future nesting behavior and the `::` delimiter are under
discussion and subject to change.

## Proposed behavior

This section includes proposals for changes to legacy behavior and
semantics for referencing pose frames.

### Element naming rules: unique names for all sibling elements

While it was not explicitly disallowed in previous versions of the spec, it
can be very confusing when sibling elements of any type have identical names.
In practice, many models include the element type in the name, whether numbered
as `link1`/`link2` or used as a suffix `front_right_wheel_joint`
/ `front_right_steering_joint`, which helps to further ensure name uniqueness
across element types.

### Pose frame semantics

Requiring unique names for sibling elements will simplify the process of
referencing frames by name, as it will be sufficient to refer to a name of an
element within the xml hierarchy without specifying the type.

Use `/` instead of `::` as separator.

Use `..` to refer to parent element.

