# Parameter passing tutorial

This documentation explains the implementation proposed in the [Parameter passing proposal](http://sdformat.org/tutorials?tut=param_passing_proposal).

**Prerequisites**:

* [Including a model in SDFormat](http://sdformat.org/tutorials?tut=composition&ver=1.7#including-a-model)
* [XPath syntax](http://sdformat.org/tutorials?tut=convention_tutorial&cat=specification&)

## Introduction

Parameter passing extends on using the `//include` tag to pass additional arguments
to SDFormat files, which allows a user to send custom data into a model file and
reduce the need for file duplication.

**Limitations**

Manipulating custom elements has limited support but will be addressed in the future
(see [custom elements](http://sdformat.org/tutorials?tut=param_passing_tutorial#custom-elements)
for details).
Also, since plugins do not require unique names, referencing the correct plugin
from the original file may not be possible. Only the first plugin with the provided
name can be referenced for modification.


## Updating included model with `//include/experimental:params`

A model can be constructed using model composition and include components of the
model using the `//include` tag.
To modify parameters of the included model, the experimental custom element
`//include/experimental:params` can be used to describe the adjustments to elements.
In the custom element `//include/experimental:params`, `experimental` is the namespace
prefix and `params` is the custom element.
This approach was chosen so that downstream users can choose to ignore the namespaced
custom element and its contents.
The goal is to implement passing additional arugments in `//include` through an
experimental custom element in SDFormat 1.7 / `libsdformat` 10 then after being
vetted it will be made official in a future SDFormat / `libsdformat` release
where the custom element will be changed to `//include/params`.
The included model (not the constructed model in the model/world file) will
be referred to as the original model/file.
In the model/world file and under `//include`, the elements listed under
`//include/experimental:params` will reference elements from the original model
and will specify new values and/or elements to be updated, added, and/or removed.

## Specify element using `element_id`

To refer to an element from the original file, the tag (e.g., `//link`, `//sensor`, `//visual`)
needs to be provided as well as the `element_id` attribute where `element_id` is
the name of all the parent elements leading to the specified element separated by
double colons (`::`) except for the `//model` name.
For example, given an original model:

```xml
<!-- original model -->
<model name="robot">
  ...
  <link name="chassis">
    <visual name="camera_visual">
      ...
    </visual>
    <sensor name="camera_sensor">
      <camera name="camera">
        ...
        <image>
          <width>320</width>
          <height>240</height>
        </image>
      </camera>
    </sensor>
  </link>
  ...
</model>
```

To specify the `//camera` element, then `element_id="chassis::camera_sensor::camera"`
where "chassis" is the name of the `//link`, "camera_sensor" is the name of `//link/sensor`,
and "camera" is the name of the `//link/sensor/camera`.
This is how the correct element will be identified and will be called the element identifier.

A corresponding `action` needs to be specified with the identified element to
dictate the desired alteration using the `action` attribute.
The `action` attribute must be provided in the element identifier or in the direct
children of the identifier.
Depending on the `action` used, if the `action` is specified in the element identifier
(i.e., where `element_id` is specified) then any children elements listed will
follow the same action. If the `action` is specified in the direct children, then
each child will follow the stated `action`. Let's look at an example.

Using the original model example above, the following snippet would live in the
model/world file (that uses the `//include/experimental:params` tag):

```xml
...
<include>
  <uri>/path/to/original/model</uri>
  ...
  <experimental:params>
    <!-- action stated in element identifier -->
    <visual element_id="chassis::camera_visual" action="remove"/>

    <!-- actions stated in direct children of element identifier -->
    <sensor element_id="chassis::camera_sensor">
      <camera name="camera" action="modify">
        <image>
          <width>1280</width>
        </image>
      </camera>
      <plugin name="camera_plugin" filename="/path/to/plugin" action="add"/>
    </sensor>
  </experimental:params>
</include>
...
```

The next section details each available action and provides several examples.
For a full combined example, please see [example 1 from the proposal](http://sdformat.org/tutorials?tut=param_passing_proposal#example-1).

## Summary of available actions:

* `add`: adds new elements to the original model
* `modify`: modifies values and/or attributes of elements.
This only updates existing elements and does not add or remove them.
* `remove`: removes the elements from the original model
* `replace`: replaces the elements from the original model to the new provided elements

Parameter passing works off the original model using a top-down approach so
subsequent actions can manipulate newly added/updated elements but need to refer
to any new changes (see [Replace examples](http://sdformat.org/tutorials?tut=param_passing_tutorial#replace-examples) > **Example 1**).

---

### Add action

The `add` action adds new elements to the original model.

When the `add` action is declared where `@element_id` is declared,
then the `@name` attribute is required for adding elements
such as a `//link`, `//visual`, `//sensor`, etc.
When the action is declared in one of the immediate children elements of
`@element_id`, then the `@name` attribute is not required unless it is required
by [SDFormat specification](http://sdformat.org/spec).

When `element_id=""` then it assumed that the element to be added is a direct
child of the original model.

#### Add examples

**Example 1**

Given the original model has a `//link[@name="chassis"]`, the below example
adds the `//visual[@name="camera_visual"]` element as a direct child of
`//link[@name="chassis"]`:

```xml
<!-- In //experimental:params -->
<visual element_id="chassis" name="camera_visual" action="add">
  <pose>1 0 0 0 0 0</pose>
  <geometry>
    <box>
      <size>0.05 0.05 0.05</size>
    </box>
  </geometry>
</visual>
```

The following example is equivalent (but it is recommended to `add` the element
following the structure above when the element requires the `@name` attribute):

```xml
<!-- //experimental:params -->
<link element_id="chassis">
  <visual name="camera_visual" action="add">
    <pose>1 0 0 0 0 0</pose>
    <geometry>
      <box>
        <size>0.05 0.05 0.05</size>
      </box>
    </geometry>
  </visual>
</link>
```

In the above example, the `add` action is specified in a direct child of where
`@element_id` is declared.

**Example 2**

For adding elements that do not have the `@name` attribute
(e.g., `//transparency`, `//pose`, `//inertial`, or `//axis`),
specify the `@action` in the direct children of the `@element_id` element.

```xml
<!-- //experimental:params -->
<link element_id="chassis">
  <inertial action="add">
    <mass>1</mass>
    <inertia>
      ...
    </inertia>
  </inertial>
</link>
```

```xml
<!-- //experimental:params -->
<visual element_id="top::camera_visual">
  <transparency action="add">0.5</transparency>
  <pose relative_to="something" action="add">0 0 1 0 0 0</pose>
</visual>
```

**Example 3**

To add a new element as a direct child of the included/original model, use `element_id=""`:

```xml
<!-- //experimental:params -->
<frame element_id="" name="some_frame" action="add">
  <pose>0 1 0 0 0 0</pose>
</frame>
```

This adds the `//frame[@name="some_frame"]` to the included `//model`
(i.e., the output would be `//model/frame[@name="some_frame"]`).

---

### Modify action

The `modify` action provides the ability to modify values and/or attributes of elements.
This action only updates existing elements and does not add or remove them.
It can update or add attributes to existing elements but can not remove them.

To `modify` elements and/or attributes, the full element path to that element is needed with the new value(s).
For example, if the original model has a `//visual[@name="visual"]/geometry/cylinder[radius=0.5]`
and modifying only the `//radius` is desired, then under `//experimental:params`:

```xml
<visual element_id="path::to::visual" action="modify">
  <geometry>
    <cylinder>
      <radius custom:attr="foo">1.0</radius>
    </cylinder>
  </geometry>
</visual>
```

This modifies the original `//radius` from 0.5 to be 1.0 (`//length` is left unchanged)
and adds a new attribute `@custom:attr="foo"` to the element.
If the original `//visual` has other children elements,
these elements are left unchanged since they are not provided under `//experimental:params`.

To modify an attribute, the attribute must be specified in the desired element
and if the element has a value (not children elements) then the value must be provided as well
even if it is desired to leave the value unchanged (see `modify` example 2).
If the element does have child elements then only providing the attribute is needed (see example 1).
If no attributes are provided, the original attributes will be left unchanged.
Although, it is not possible to remove an attribute, a work around is to assign
the attribute an empty value (e.g., `//pose[@relative_to=""]`).
The `@name` attribute can only be modified when the `modify` action is defined in the `@element_id` element.

#### Modify examples

**Example 1**

To `modify` a name attribute only, in this case the original model has `//visual[@name="visual"]`:

```xml
<!-- //experimental:params -->
<visual element_id="path::to::visual" name="new_name" action="modify"/>
```

this only modifies the `@name` of the `//visual` to be `new_name`.
There are no other listed attributes or child elements so nothing else is changed.

To `modify` child elements and not the `@name` attribute provide
the full path to the element to be modified with the updated value (see [above example](http://sdformat.org/tutorials?tut=param_passing_tutorial#modify-action)).

**Example 2**

Looking at an `//inertial` example, if the original model has:

```xml
<!-- included model -->
...
<link name="chassis">
  <pose relative_to="some_frame">1 0 0 0 0 0</pose>
  <inertial>
    <mass>1.14395</mass>
    <inertia>
      <ixx>0.126164</ixx>
      <ixy>0</ixy>
      <ixz>0</ixz>
      <iyy>0.416519</iyy>
      <iyz>0</iyz>
      <izz>0.481014</izz>
    </inertia>
  </inertial>
  ...
</link>
...
```

To `modify` the `@relative_to`, `//mass`, and some `//inertia` elements:

```xml
<!-- //experimental:params -->
<link element_id="path::to::chassis" action="modify">
  <pose relative_to="new_frame">1 0 0 0 0 0</pose>
  <inertial>
    <mass>2.637</mass>
    <inertia>
      <ixx>0.02467</ixx>
      <iyy>0.04411</iyy>
      <izz>0.02467</izz>
    </inertia>
  </inertial>
</link>
```

Note that `//pose` values are the same values as the original.
Currently, there is no way to specify if element values should remain the same
so they must be provided (this can only be done with attributes by not providing them).

The expected updated model would be:

```xml
<!-- updated included model -->
...
<link name="chassis">
  <pose relative_to="new_frame">1 0 0 0 0 0</pose>
  <inertial>
    <mass>2.637</mass>
    <inertia>
      <ixx>0.02467</ixx>
      <ixy>0</ixy>
      <ixz>0</ixz>
      <iyy>0.04411</iyy>
      <iyz>0</iyz>
      <izz>0.02467</izz>
    </inertia>
  </inertial>
  ...
</link>
...
```

---

### Remove action

The `remove` action removes elements from the original model.

When `remove` is declared where `@element_id` is specified, the entire element is removed.
Any listed children elements are ignored.
When `remove` is declared in the direct children of `@element_id` there are 2 cases to consider.
If the direct child (where `remove` is defined):

1. Has no children elements, the entire element is removed
(see [Remove examples](http://sdformat.org/tutorials?tut=param_passing_tutorial#remove-examples) > **Example 1**)
1. Has children, the individual children are removed but not the element where `remove` is defined
(see [Remove examples](http://sdformat.org/tutorials?tut=param_passing_tutorial#remove-examples) > **Example 2**)

When `remove` is declared in the direct children and that element has a `@name` attribute,
the `@name` should be provided. If `@name` isn't specified
(when `@name` exists in the original model) and there are multiple elements
with the same tag in the original model, the first element found with the same tag is assumed.
It is better to specify the element directly using `@element_id` when possible to
avoid potential confusion.

See below `remove` examples for more details.

#### Remove examples

**Example 1**

Given the original model has a `//link[@name="chassis"]/visual[@name="camera_visual"]`,
to `remove` the `//visual[@name="camera_visual"]` element:

```xml
<!-- //experimental:params -->
<visual element_id="chassis::camera_visual" action="remove"/>
```

This removes the entire `//visual` element (including all its children) from the original model.
If there are attributes or children declared where `@action` is defined,
they are ignored. For example,

```xml
<!-- //experimental:params -->
<visual element_id="chassis::camera_visual" action="remove" some="attribute">
  <material>
    ...
  </material>
</visual>
```

The attribute `some="attribute"` and `//material` elements are ignored since
the entire `//visual` element is being removed.

The following example is equivalent (but it is recommended to `remove` the element
following the structure above if removing the entire element is desired):

```xml
<!-- //experimental:params -->
<link element_id="chassis">
  <visual name="camera_visual" action="remove"/>
</link>
```

In this situation, stating `@name` is not required but is encouraged
since there could exist multiple `//visual` elements in the original.
When there are multiple, the first found `//visual` element found is assumed to be removed.

**Example 2**

For removing elements that do not have the `@name` attribute
(e.g., `//transparency`, `//material`, or `//inertial`),
specify the `@action` in the direct children of the `@element_id` element.

```xml
<!-- //experimental:params -->
<visual element_id="chassis::camera_visual">
  <material action="remove"/>
</visual>
```

In the above example, since `//material` does not have listed children elements,
the entire `//material` element is removed.
To remove only individual children of `//material`, list them as follows:

```xml
<!-- //experimental:params -->
<visual element_id="chassis::camera_visual">
  <material action="remove">
    <diffuse/>
    <ambient/>
  </material>
</visual>
```

This removes, `//diffuse` and `//ambient` from `//material` but not `//material`.

Note that it is not possible to remove further sub elements.
For example, if the original model has:

```xml
<!-- included model -->
<joint name="joint" type="revolute">
  ...
  <axis>
    ...
    <dynamics>
      ...
      <damping>0.2</damping>
    </dynamics>
  </axis>
</joint>
```

`//damping` can not be removed using the `remove` action, only `//dynamics` can be.
Use the `replace` action to replace `//dynamics` with the desired instead
(see `replace` action example 2).

---

### Replace action

The `replace` action replaces elements from the original model to the new provided elements.

Elements can only be replaced with the same type of elements
(e.g., a `//link` can not be replaced by a `//visual`).
When using the `replace` action where `@element_id` is declared,
then the element requires a `@name` attribute (even if the old name is being used)
since we are replacing the original element with the newly provided.
Similar to `remove`, if `replace` is stated in the direct children of the
`@element_id` element then the `@name` attribute is used to find the child of `@element_id`.
If none is provided, the first matching element (with the same tag) is assumed to be replaced.
Replacing `@name` with a new `@name` can only be done when the `replace` action
is defined in the `@element_id` element.

#### Replace examples

**Example 1**

Given the original model has a `//link[@name="chassis"]/visual[@name="camera_visual"]`,
to `replace` the `//visual[@name="camera_visual"]` element:

```xml
<!-- //experimental:params -->
<visual element_id="chassis::camera_visual" name="camera_visual" action="replace">
  ...
</visual>
```

This replaces the original element with the provided element above
but notice the `@name` attribute will remain the same.

The following example is equivalent but is not recommended:

```xml
<!-- //experimental:params -->
<link element_id="chassis">
  <visual name="camera_visual" action="replace">
    ...
  </visual>
</link>
```

In this situation, stating `@name` is not required but is encouraged
since there could exist multiple `//visual` elements in the original.
When there are multiple, the first found `//visual` element found is assumed to be replaced.
Also, in this case, the `@name` can not be changed.
To `replace` the element with a new name:

```xml
<!-- //experimental:params -->
<visual element_id="chassis::camera_visual" name="new_name" action="replace">
  ...
</visual>
```

It is possible to further alter this `//visual` element but will need to refer
to the new element `@name`:

```xml
<!-- //experimental:params -->
<visual element_id="chassis::new_name" action="modify">
  <!-- modifications -->
</visual>
```

**Example 2**

For replacing elements that do not have the `@name` attribute
(e.g., `//material`, `//inertial`, or `//axis`),
specify the `@action` in the direct children of the `@element_id` element.

For example, if the original model has:

```xml
<!-- included model -->
<joint name="joint" type="revolute">
  ...
  <axis>
    ...
    <dynamics>
      ...
      <damping>0.2</damping>
    </dynamics>
  </axis>
</joint>
```

to `replace` `//axis`:

```xml
<!-- //experimental:params -->
<joint element_id="path::to::joint">
  ...
  <axis action="replace">
    ...
    <dynamics>
      ...
      <damping>0</damping>
    </dynamics>
  </axis>
</joint>
```

## Custom elements

Currently, the only way to `add` a custom element is through a known `SDFormat`
tag. For example,

```xml
<!-- //experimental:params -->
<link element_id="" name="link1" action="add">
  <foo:custom_elem>foo</foo:custom_elem>
</link>
```

The alterations that can be done are `modify` or `remove` when the `action`
is stated in the child of `@element_id`. For instance,

```xml
<!-- //experimental:params -->
<link element_id="link1">
  <foo:custom_elem action="modify">bar</foo:custom_elem>
</link>
```

The expected output of the original model will be:

```xml
<link name="link1">
  <foo:custom_elem>bar</foo:custom_elem>
</link>
```
