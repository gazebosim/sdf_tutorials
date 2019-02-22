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
link connected to a fixed inertial frame as parent and child.

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
to its sibling, not to a fixed inertial frame.

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

### Support for nested models in sdf 1.5

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

## Proposed behavior

This section will include proposals for changes to legacy behavior and
semantics for referencing pose frames.

