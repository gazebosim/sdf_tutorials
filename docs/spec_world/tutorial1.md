# Creating Worlds in SDFormat

Conceptually, a world in SDFormat is a environment in which models can be
instantiated and simulated by a physics engine. A world is created using the
`<world>` tag. It can contain various elements, but this documentation only covers the
`<model>` element as we describe how to create a simulation world
composed of various models. The full specification of `<world>` can be found
[here](http://sdformat.org/spec?ver=1.4&elem=world).

> **Note**: See [Appendix](#appendix) about a required name attribute of `<world>`

One of the most fundamental properties of a world is that it contains the world
coordinate frame, which is defined to be the canonical inertial frame of
reference for all dynamic bodies in the world. When a model is inserted into
a world as a direct child of `<world>`, its pose is expressed relative to this
frame. Refer to the [Specifying Pose](/tutorials?tut=specify_pose) and [Model
Kinematics](/tutorials?tut=spec_model_kinematics) documentation to learn more
about setting poses of models.

Two methods are available in SDFormat for inserting a model into a world.

## Models defined inline

The first method involves defining the models directly inside the `<world>`
tag. Example:

```xml
<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="simple_world">
    <model name="ground">
      <link name="body">
        ...
      </link>
    </model>
    <model name="box">
      <pose>0 0 1 0 0 0</pose>
      <link name="body">
        ...
      </link>
    </model>
    <model name="sphere">
      <pose>10 0 2 0 0 0</pose>
      <link name="body">
        ...
      </link>
    </model>
  </world>
</sdf>
```
This is the simplest approach since it only requires a single file to describe
the world. However, it has some drawbacks.

1. If multiple instances of the same model but at different poses are desired,
   the entire text of the `<model>` tag has to be duplicated.
1. Models defined in a world file cannot be used in other worlds or other SDF
   files.

## Models defined in other files

To mitigate these issues SDFormat v1.4 introduced the `<include>` tag inside
`<world>`. With this approach, models can be defined in separate files and
later get inserted into a world by using the `<include>` tag. Example:

```xml
<!--ground/ground.sdf-->
<?xml version="1.0" ?>
<sdf version="1.4">
  <model name="ground">
    <link name="body">
      ...
    </link>
  </model>
</sdf>
```

```xml
<!--ground/model.config-->
<?xml version="1.0" ?>
<model>
  <name>ground</name>
  <sdf version="1.4">ground.sdf</sdf>
</model>
```

```xml
<!--box/box.sdf-->
<?xml version="1.0" ?>
<sdf version="1.4">
  <model name="box">
    <link name="body">
      ...
    </link>
  </model>
</sdf>
```

```xml
<!--box/model.config-->
<?xml version="1.0" ?>
<model>
  <name>box</name>
  <sdf version="1.4">box.sdf</sdf>
</model>
```

```xml
<!--sphere/sphere.sdf-->
<?xml version="1.0" ?>
<sdf version="1.4">
  <model name="sphere">
    <pose>1 2 3 0 0 0</pose>
    <link name="body">
      ...
    </link>
  </model>
</sdf>
```

```xml
<!--sphere/model.config-->
<?xml version="1.0" ?>
<model>
  <name>sphere</name>
  <sdf version="1.4">sphere.sdf</sdf>
</model>
```

```xml
<!--simple_world.sdf-->
<?xml version="1.0" ?>
<sdf version="1.4">
<world name="simple_world">
    <include>
      <uri>ground</uri>      
    </include>
    <include>
      <uri>box</uri>      
    </include>
    <include>
      <uri>sphere</uri>      
      <pose>10 0 2 0 0 0</pose>
    </include>
  </world>
</sdf>
```

As can be seen in the example, the models `ground`, `box`, and `sphere` are
defined in the files `ground/ground.sdf`, `box/box.sdf`, and
`sphere/sphere.sdf` respectively along with their `model.config` files. In
`simple_world.sdf` the `<include>` tag is used to include the models in the
world. The pose of each model can be overridden by the `<pose>` child tag of
`<include>`. This is demonstrated in the example where the pose of the sphere
in the original definition of the model was `1 2 3 0 0 0` but gets overridden
to `10 0 2 0 0 0` when inserted into the world. Since the name of a model has
to be unique, `<include>` also provides a mechanism for overriding the name of
the included model. Thus, it is possible to create two instances of the same
model with different names as shown in the following example.

```xml
<?xml version="1.0" ?>
<sdf version="1.5">
<world name="simple_world_two_boxes">
    <include>
      <uri>box</uri>      
      <name>box1</name>
    </include>
    <include>
      <uri>box</uri>      
      <name>box2</name>
      <pose>4 0 1 0 0 0</pose>
    </include>
  </world>
</sdf>
```

> **Note**: A functionally limited version of the `<include>` tag is available
> in SDFormat v1.4. This version allows specifying the `<uri>` of the
> externally defined model but does not allow overriding the name or pose of
> the inserted model.

## Creating a box and a sphere on ground plane

A fully working example of a box and a sphere placed on a ground plane is
provided below. Note that the `<static>` tag is used in the ground model to
indicate that the model does not behave as a dynamic object and should be
considered only for its collision and visual properties. More about the
`<static>` tag can be found in the [Inertial
Properties](/tutorials?tut=spec_inertial) documentation (coming soon).

```xml
<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="simple_world">
    <model name="ground">
      <static>true</static>
      <link name="ground_link">
        <collision name="collision1">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual1">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>

    <model name="box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="body">
        <collision name="collision1">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual1">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>

    <model name="sphere">
      <pose>10 0 1 0 0 0</pose>
      <link name="body">
        <collision name="collision1">
          <geometry>
            <sphere>
              <radius>1</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="visual1">
          <geometry>
            <sphere>
              <radius>1</radius>
            </sphere>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Appendix

The `<world>` tag has a required `name` attribute. It can be used to
differentiate between multiple worlds running in parallel. However, this is not
a very common use case and will not be discussed in this article.

The world element is not referred to elsewhere in an SDF file by the name
specified in its `name` attribute. Instead, the special name `world` is used.
For example, when referring to the world as a link in the `<joint>` element,
the special name `world` refers to the world as long as there is no sibling link
that has the name `world`.

```xml
<sdf version="1.4">
  <world name="world_with_joint">
    <model name="fixed_box">
      <link name="body"/>
      <joint name="j_fixed" type="fixed">
        <parent>world</parent> <!-- The name `world` is used instead of world_with_joint -->
        <child>body</child>
      </joint>
    </model>
  </world>
</sdf>
```
