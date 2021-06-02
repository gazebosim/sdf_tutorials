## Summary of available actions:

* `add`: adds new elements to the original model
* `modify`: modifies values and/or attributes of elements.
This only updates existing elements and does not add or remove them.
* `remove`: removes the elements from the original model
* `replace`: replaces the elements from the original model to the new provided elements

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

### Remove action

The `remove` action removes elements from the original model.

When `remove` is declared where `@element_id` is specified, the entire element is removed.
Any listed children elements are ignored.
When `remove` is declared in the direct children of `@element_id` there are 2 cases to consider.
If the direct child (where `remove` is defined):
1. Has no children elements, the entire element is removed
1. Has children, the individual children are removed but not the element where `remove` is defined

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
to remove the `//visual[@name="camera_visual"]` element:

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
<joint name='joint' type='revolute'>
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
use the `replace` action to replace `//dynamics` with the desired instead.
