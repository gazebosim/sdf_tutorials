# Composition and Nesting

This document describes SDFormat's support for model level composition in which
a model can be constructed from other models nested within itself.

## Introduction

Models are the fundamental building blocks of a world in SDFormat. As described
in the [Creating Worlds](/tutorials?tut=spec_world) documentation, models can be
defined directly inside a `<world>` element or they can be defined in separate
files and get added into the world with the help of the `<include>` tag. The
latter approach is a form of composition that allows a model to be defined once
and get instantiated multiple times with different parameters such as the
model's name and pose. This form of composition has been available in SDFormat
since version 1.4.

SDFormat version 1.5 introduced a form of composition that allows users to
build a single model by combining other models. This document will focus on
this form of composition.

## Model level composition

Model level composition can be achieved by either defining models directly
inside other models or by defining each model in its own file and including it
into the parent model with the help of the `<include>` tag.

### Defining models directly inside parent models

Models can be nested inside other models by directly defining the nested model
inside the `<model>` tag of the parent model. The following snippet shows
a nested model `sphere` defined inside a parent model `PM`.

```
<model name="PM">
  <link name="body"/>
  <model name="sphere">
    <pose>0 0 0.5 0 0 0</pose>
    <link name="body"/>
  </model>
</model>

``` 

### Defining models in separate files

Models can be defined in individual files by adhering to a model database
directory structure. This directory structure requires that each model has its
own directory that contains at least a metadata file named `model.config` and
a file containing the model definition. The model definition file has `<sdf>`
as its root element followed by `<model>`. The following example shows the
directory structure and contents of a model named `sphere`.


```
.
├── sphere
    ├── model.config
    └── model.sdf
```

```xml
<!-- Metadata file: sphere/model.config -->
<?xml version="1.0" ?>
<model>
  <sdf version="1.5">model.sdf</sdf>
</model>
```

```xml
<!-- Model definition: sphere/model.sdf -->
<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="sphere">
    <link name="body">
      <visual name="v1">
        <geometry>
          <sphere>
            <radius>0.1</radius>
          </sphere>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>

```

More details about the directory structure can be found
[here](http://gazebosim.org/tutorials?tut=model_structure).

#### Including a model

A model that has been defined in a model database can be added to a parent
model using the `<include>` tag. The `<uri>` child element specifies the path
to the model directory. In the `libsdformat` implementation, the input to the
`<uri>` element can be one of the following

  * A path that begins `file:///`. Such paths specify the absolute path to the
    model directory.
  * A path that begins with `file://`. Such paths specify the relative path to the
    model directory. The path is relative to the runtime path of the
    application that uses `libsdformat`.
  * A path that begins with a user defined prefix, such as `model://`.
    `libsdformat` lets users specify a custom prefix and a corresponding path
    to be searched when a URI with the custom prefix is found.
  * A path with no prefix. Such paths specify the relative path to the
    model directory. The path is relative to the runtime path of the
    application that uses `libsdformat`.

The `<include>` tag contains additional child elements that specify certain
properties of the included model.

  * `<name>`: Overrides the name of the nested model. References from the
  parent model to elements inside the nested model must take into account this
  new name. This new name must also be unique among sibling nested models.
  * `<pose>`: Overrides the pose of the nested model. The pose is specified
  with respect to the parent model frame.
  * `<static>`: Overrides the static value of the included model.
  * `<plugin>`: A plugin element to be added to the list of plugins associated
  with the included model.

The following example shows the use of the `<include>` tag to create a model
that contains multiple instances of the `sphere` model shown in the earlier
example.

```
<model name="PM">
  <include>
    <uri>file://sphere</uri>
    <pose>0 0 0.5 0 0 0</pose>
  </include>
  <include>
    <uri>file://sphere</uri>
    <name>sphere1</name>
    <pose>0 0 1 0 0 0</pose>
  </include>
  <include>
    <uri>file://sphere</uri>
    <name>sphere2</name>
    <pose>1 0 1 0 0 0</pose>
  </include>
</model>
```

## Characteristics of nested models 

Whether nested by direct definition or through the use of the `<include>` tag,
the following characteristics are true about nested models.

<!--TODO-->
<!--* Name uniqueness of sibling nested models-->

### Pose and nested models 

The pose of a nested model is expressed with respect to the parent model.
Consider the following example.

```
<model name="PM">
  <include>
    <uri>file://sphere</uri>
    <pose>0 0 0.5 0 0 0</pose>
    <name>sphere</name>
  </include>

  <model name="M1">
    <pose>0 1 0 0 0 0</pose>
    <link name="L1">
      <pose>1 0 0 0 0 0</pose>
    </link>
  </model>
</model>
```

The pose of the `sphere` model is translated `0.5m` in the `z` direction
relative to the origin of `PM`.

The poses of links inside a nested model are expressed with respect to the
frame of the model that contains them. In the example above, the pose of link
`L1` is translated `1m` in the `x` direction relative to the origin of `M1`
, which, in turn is translated `1m` in the `y` direction relative to the origin
of `PM`.

### Joints and nested models 

Joints can connect links from the parent model to links in the nested model.
Normal rules apply for referencing links found in the parent model. Links in
nested models are referenced by using a `model_name::link_name` syntax where
`model_name` is the name of the model as defined by the `<name>` of the model
definition or as overridden by the `<include><name>`. The link from the nested
model can be referenced in either the `<parent>` or `<child>` elements of the
joint.

Note, however, that `libsdformat` does not currently check that nested model
links referenced by joints are valid. This responsibility is left for
downstream applications.

The following snippet demonstrates a fixed joint between one link in the parent
model and another in a nested model:

``` 
<model name="spheres">
  <link name="body"/>
  <include>
    <uri>file://sphere</uri>
    <pose>0 0 0.5 0 0 0</pose>
    <name>sphere</name>
  </include>
  <joint name='j1' type='fixed'>
    <parent>body</parent>
    <child>sphere::body</child>
  </joint>
</model>
```


A joint in a parent model can reference links in nested models as its
`<parent>` or `<child>` element. That is, a joint can be created between two
links that reside in two separate instances of nested models. The following
snippet demonstrates this usage:

``` 
<model name="spheres">
  <link name="body"/>
  <include>
    <uri>file://sphere</uri>
    <pose>0 0 0.5 0 0 0</pose>
    <name>sphere1</name>
  </include>
  <include>
    <uri>file://sphere</uri>
    <pose>1 0 0.5 0 0 0</pose>
    <name>sphere2</name>
  </include>
  <joint name='j1' type='fixed'>
    <parent>sphere1::body</parent> <!-- Link contained in model sphere1 -->
    <child>sphere2::body</child> <!-- Link contained in model sphere2 -->
  </joint>
</model>
```

A joint in a child model can access a link in the parent model or a sibling
model with the syntax `model_name::link_name`. However, this is not recommended
as it requires hard coding the name of the parent sibling model in the
definition of the child model.


<!--# References-->

<!--[] https://bitbucket.org/osrf/gazebo_design/pull-requests/18/add-support-for-nested-models-in-gazebo/-->

<!--[] http://gazebosim.org/tutorials?tut=nested_model&cat=build_robot-->

<!--[] http://gazebosim.org/tutorials?tut=model_structure&cat=build_robot-->

<!--[] https://bitbucket.org/osrf/sdformat/pull-requests/214/support-nesting-of-model-sdf-elements/diff-->
