# Proposal for parameter passing

* **Authors**:
Jenn Nguyen `<jenn@openrobotics.org>`,
Addisu Taddese `<addisu@openrobotics.org>`
* **Status**: Draft
* **SDFormat Version**: 1.7
* **`libsdformat` Version**: 10

## Introduction

**Purpose statement**

This proposal suggests extending the [composition behavior in SDFormat 1.7](
  http://sdformat.org/tutorials?tut=composition#including-a-model)
(i.e., using the `<include>` tag) to pass additional arguments to SDFormat files
which will allow a user to send custom data into a model file and reduce the
need for model file duplication.

**Background**

Currently, through the `<include>` (i.e., `//include`) tag, model parameters
such as `//model/name` and `//model/pose` may be overwritten but there are
cases when other parameters are desired to be updated as well. For example, if
several robots have the same base but different sensor configurations then
several model files need to be created to compose all the robot variations.
This is an issue because these model files are most likely copies of each other
with certain parameters changed.

**Conclusion**

The aim of adding additional parameter support is to avoid this duplication,
where a model file contains all configurations with default parameters and a
user may update and/or override certain parameters as desired in a model or
world file when using the `//include` tag.

## Document summary

* *Proposed changes*: details the proposed design and the actions available to
users
* *Example*: provides an example of how this proposed approach can be utilized


## Motivation

The motivation behind this proposal is to avoid duplication when constructing
models. For example, in [SubT's robots](
  https://github.com/osrf/subt/wiki/Robots), X1 has 8-10 different sensor
configurations but all have the same base. If we take a look at all the
different configurations in [Fuel](https://app.ignitionrobotics.org/), we see
that they are essentially copies of each other but with different parameters for
these sensors.

## Proposed changes

Parameters will be specified by the user in either model or world files through
the use of model
[composition (or nested models)](http://sdformat.org/tutorials?tut=composition)
which will contain `//include` tags and
[custom elements](http://sdformat.org/tutorials?tut=custom_elements_attributes_proposal&cat=pose_semantics_docs).
Discussed in this section is the proposed design for specifying parameters and
the actions available to the user. The intent is to provide a means of
manipulating almost all parameters in SDF models through model/world files
without requiring modifications to already existing/created model files.
The new attributes `element_id` and `action` (detailed below) will be reserved
from being used in other elements in SDFormat.

### 1. Add custom element `//include/experimental:params`

In a world file, the user can construct a model using model composition and
include components of the model using the `//include` tag. If a user wants to
modify parameters of the included model, then the user will use the experimental
custom element `//include/experimental:params` that will describe the
adjustments to elements. In the custom element `//include/experimental:params`,
`experimental` is the namespace prefix and `params` is the custom element. This
approach was chosen so that downstream users can choose to ignore the namespaced
custom element and its contents. The goal is to implement passing additional
arguments in `//include` through an experimental custom element in
SDFormat 1.7 / `libsdformat` 10 then after being vetted it will be made official
in a future SDFormat / `libsdformat` release where the custom element will be
changed to `//include/params`.The included model (not the constructed model in
the model/world file) will be referred to as the original model/file.
In the model/world file and under `//include`, the elements listed under
`//include/experimental:params` will indicate elements from the original model
and will specify new values and/or elements to be updated, added, and/or
removed.

**Details**

To specify an element from the original file, the tag (e.g., `//link`,
`//sensor`, `//visual`) needs to be provided as well as the `element_id` attribute
where the `element_id` is the name of all the parent elements leading to the
specified element separated by double colons (`::`). For example, given:

```xml
<link name="chassis">
  <sensor name="camera">
    <camera name="cam">
      ...
    </camera>
  </sensor>
</link>
```

If the user wants to specify the `//camera` element, then
`element_id="chassis::camera::cam"` where "chassis" is the name of the `//link`,
"camera" is the name of the `//link/sensor`, and "cam" is the name of the
`//link/sensor/camera`. This is how the correct element will be identified and
will be called the element identifier. Although it is possible to identify the
element without requiring the user to specify the tag, enforcing the user to
provide the tag allows less ambiguity to the user as well as provides a means of
error checking. Then the user will need to specify a corresponding action with
that element to dictate the alteration they would like to make using a new
attribute `action`. The attribute takes in a string and the available actions
are:

* `modify`: indicates that the values in the original model are to be modified
to the new listed values
* `replace`: replaces the elements and/or values from the original model to the
new provided elements and/or values
* `add`: adds new elements to the original model
* `remove`: removes (or disables) the elements from the original model

The `action` attribute must be provided either in the element identifier or in
the direct children of the identifier.

##### Alternatives considered

Alternatively, instead of using the proposed element identifier, the user could
use the attribute `id` for each customizable element in the original file but
the intent of using the named path (i.e., the name of the parent elements and
itself separated by double colons) was to provide a way for the user to modify
the original model without requiring large amounts of updates to the original
file.

## Examples

### Example 1

Let's look at an example, here is an original model `base_robot`:

```xml
<model name="base_robot">
  ...
  <link name="chassis">
    <visual name="lidar_visual">
      ...
    </visual>
    <sensor name="lidar">
      ...
    </sensor>
    <sensor name="camera">
      <pose>0.5 0.02 0 0 0</pose>
      ...
    </sensor>
  </link>

  <link name="top">
    <visual name="camera_visual">
      ...
      <geometry>
        <box>
          <size>0.03 0.03 0.05</size>
        </box>
      </geometry>
      <material>
        <ambient>0.5 0.5 0 1</ambient>
        <diffuse>0 1 0 1</diffuse>
        <specular>0.5 0.5 0.5 1</specular>
      </material>
    </visual>
    <sensor name="camera">
      <pose>0.2 1 0.5 0 0 0</pose>
      <update_rate>20</update_rate>
      ...
    </sensor>
  </link>
```

Now the user would like to use the original model `base_robot` in a world file
but with modifications. The modifications include removing the lidar from the
chassis link, adding a plugin to the camera sensor in the chassis link, adding a
camera visual to the chassis link, update camera information in the top link,
and replace the top link's camera visual geometry.

```xml
<sdf version="1.7"
     xmlns:experimental="http://sdformat.org/schemas/experimental">
  <model name="custom_robot">
    <include>
      <uri>model://base_robot</uri>

      <experimental:params>
        <visual element_id="chassis::lidar_visual" action="remove"/>
        <sensor element_id="chassis::lidar" action="remove"/>

        <sensor element_id="chassis::camera" action="add">
          <plugin name="camera_depth_sensor" filename="libcamera_depth_sensor.so"/>
        </sensor>

        <visual element_id="chassis::camera_visual" action="add">
          <pose>0.5 0.02 0 0 0</pose>
          <geometry>
              <box>
                  <size>0.02 0.02 0.02</size>
              </box>
          </geometry>
        </visual>

        <sensor element_id="top::camera" action="modify">
          <pose>0 1 0 0 0 0</pose>
          <update_rate>60</update_rate>
        </sensor>

        <visual element_id="top::camera_visual">
          <geometry action="replace">
            <sphere>
              <radius>0.05</radius>
            </sphere>
          </geometry>
          <material action="modify">
            <ambient>0 1 0 1</ambient>
          </material>
        </visual>

      </experimental:params>
    </include>
  </model>
</sdf>
```

In `//include/experimental:params`, the action can be either listed in the
element identifier or listed individually in the children element(s).
For example,

```xml
<sensor element_id="top::camera" action="modify">
  <pose>0 1 0 0 0 0</pose>
  <update_rate>60</update_rate>
</sensor>
```

The action is defined at the top level where the `element_id` is, which means
that all children listed in this element will be modified. If the user wants to
do several different actions to the children elements then the user can specify
the action for each child element individually. For instance,

```xml
<visual element_id="top::camera_visual">
  <geometry action="replace">
    <sphere>
      <radius>0.05</radius>
    </sphere>
  </geometry>
  <material action="modify">
    <ambient>0 1 0 1</ambient>
  </material>
</visual>
```

If the original model has several corresponding elements, it will be up to the
user to update all appropriately. For example, if a lidar has a `//visual` and
`//sensor` element and the user would like to remove the lidar information, then
both will need to be listed for removal:

```xml
<visual element_id="chassis::lidar_visual" action="remove"/>
<sensor element_id="chassis::lidar" action="remove"/>
```

When a user wants to `add` an element that does not exist in the original model,
then the `add` action can be used similarly as follows:

```xml
<visual element_id="chassis::camera_visual" action="add">
  <pose>0.5 0.02 0 0 0</pose>
  <geometry>
      <box>
          <size>0.02 0.02 0.02</size>
      </box>
  </geometry>
</visual>
```

In this example, the `//visual` element named "camera_visual" will be added as a
child of `//link[@element_id=’chassis’]`. A check will be performed to ensure
that the element does not already exist. If it does exist, then a warning/error
will be printed and the element will be skipped. The process of skipping
elements will also occur when the user uses the `modify`, `replace`, and/or
`remove` actions and the element is not found in the original model.

**Expected output**

Below shows what the example's output should be after parsing
`//experimental:params`:

```xml
<model name="base_robot">
  ...
  <link name="chassis">
    <sensor name="camera">
      <pose>0.5 0.02 0 0 0</pose>
      <plugin name="camera_depth_sensor" filename="libcamera_depth_sensor.so"/>
      ...
    </sensor>
    <visual name="camera_visual">
      <pose>0.5 0.02 0 0 0</pose>
      <geometry>
        <box>
          <size>0.02 0.02 0.02</size>
        </box>
      </geometry>
    </visual>
  </link>

  <link name="top">
    <visual name="camera_visual">
      ...
      <geometry>
        <sphere>
          <radius>0.05</radius>
        </sphere>
      </geometry>
      <material>
        <ambient>0 1 0 1</ambient>
        <diffuse>0 1 0 1</diffuse>
        <specular>0.5 0.5 0.5 1</specular>
      </material>
    </visual>
    <sensor name="camera">
      <pose>0 1 0 0 0 0</pose>
      <update_rate>60</update_rate>
      ...
    </sensor>
  </link>
```

## Example 2

If the original model instead contained:

```xml
...
<link name="top">
  <visual name="camera_visual">
    ...
    <geometry>
      <sphere>
        <radius>0.1</radius>
      </sphere>
    </geometry>
  </visual>
</link>
...
```

And the user would like to modify the radius of the sphere in the `//visual`
element. Then under `//experimental:params` the user would specify:

```xml
<visual name="top::camera_visual">
  <geometry action="modify">
    <sphere>
       <radius>0.05</radius>
    </sphere>
  </geometry>
</visual>
```

## Ease of parameter modification

This section discusses how this proposed functionality provides easier parameter
modification instead of using only model composition.

First, adding a `//plugin` to already existing elements (which can not be done
using model composition) such as a `//link/sensor` or `//link/joint` can quickly
be done using the `add` action.

Also, adding elements including a `//sensor` to an existing element can easily
be done using the `add` action. If done through model composition, then a dummy
link and joint would need to be created to attach the `//sensor` to the desired
element.

Lastly, this provides a way to update values such as
`//sensor/camera/update_rate` or `//sensor/camera/topic` using the action
`modify` or `replace`. This can not be easily done with composition, if another
value is desired for a particular model then a duplicate model containing the
other value must be created.
