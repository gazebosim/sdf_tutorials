# Composition

This document describes SDFormat's support for model level composition in which
a model can be constructed from other models nested within itself.

**Note**: This describes older legacy behavior. Newer features, intended for
SDFormat 1.8 / `libsdformat` 11, are described in the
[Composition Proposal](/?tut=composition_proposal).

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
a nested model `sphere` defined inside a parent model `Pm`.

```
<model name="Pm">
  <link name="body"/>
  <model name="sphere">
    <pose>0 0 0.5 0 0 0</pose>
    <link name="body"/>
  </model>
</model>

``` 

#### WARNING

The current version of libsdformat's DOM API (`libsdformat` <= 9.2.x) does not
support models defined directly inside parent models. As a workaround, such
models can be accessed using the `Element` API.

This will change in SDFormat 1.7 as implemented by `libsdformat` 9.3.0.
Please see the
[Pose Frame Semantics - Directly Nested Models]
(/tutorials?tut=pose_frame_semantics_proposal#directly-nested-models) section for more information.

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
model using the `<include>` tag.
The `<include>` tag may contain the following elements that specify the
location of the model and optionally override some of its properties:

  * `<uri>`: A string used to indicate the location of the model directory to
  be included. This is described in more detail below.
  * `<name>`: Overrides the name of the nested model. References from the
  parent model to elements inside the nested model must take into account this
  new name. This new name must also be unique among sibling nested models.
  * `<pose>`: Overrides the pose of the nested model. The pose is specified
  with respect to the parent model frame.
  * `<static>`: Overrides the static value of the included model.
  * `<plugin>`: A plugin element to be added to the list of plugins associated
  with the included model.

In `libsdformat`, the contents of the `<uri>` element are passed to the
[sdf::findFile](https://github.com/osrf/sdformat/blob/sdformat9_9.0.0/src/SDF.cc#L58-L167)
function defined in
[sdf/SDFImpl.hh](https://github.com/osrf/sdformat/blob/sdformat9_9.0.0/include/sdf/SDFImpl.hh#L55-L65).
This function searches for the model files using the steps in the following
order until the model is found:

  1. Users can define paths on their system associated with a specific URI
     scheme using [sdf::addURIPath](https://github.com/osrf/sdformat/blob/sdformat9_9.0.0/include/sdf/SDFImpl.hh#L67-L72).
     For example, if `sdf::addURIPath("model://", path);` has been called,
     including a `<uri>model://sphere</uri>` will search the folder in `path`
     for subfolders named `sphere`.

  1. `libsdformat` treats the URI as a directory path and proceeds to search
     for it within its installation `share` path.

  1. `libsdformat` treats the URI as a directory path and proceeds to search for
     it in the file system. If the URI is an absolute path, `libsdformat`
     checks if the path exists. Otherwise, `libsdformat` searches for the path
     relative to the current working directory of the process that uses
     `libsdformat`.

  1. If the environment variable `SDF_PATH` is set, `libsdformat` forms a new
     path by appending the URI to the contents of `SDF_PATH` and checks if
     the path exists.

  1. `libsdformat` treats the URI as a directory path and forms a new path by
     appending it to the current working directory of the process. It then
     checks if this path exists.

  1. Finally, `libsdformat` uses a mechanism by which users can register
     a custom callback function for URIs that could not be found by any of the
     previous methods.

The following example shows the use of the `<include>` tag to create a model
that contains multiple instances of the `sphere` model shown in the earlier
example.

```
<model name="Pm">
  <include>
    <uri>/path/to/sphere</uri>
    <pose>0 0 0.5 0 0 0</pose>
  </include>
  <include>
    <uri>/path/to/sphere</uri>
    <name>sphere1</name>
    <pose>0 0 1 0 0 0</pose>
  </include>
  <include>
    <uri>/path/to/sphere</uri>
    <name>sphere2</name>
    <pose>1 0 1 0 0 0</pose>
  </include>
</model>
```

#### `libsdformat`'s implementation of `<include>` in models

`libsdformat`'s current implementation of the `<include>` tag works by copying
all links and joints of the child model into the parent model with their poses
modified to be relative to the parent model frame. To avoid name collisions,
the name of the nested model followed by `::` is prepended to the names of
these links and joints. The following example shows how this works. Consider
the following model named `ChildModel` and its parent model `ParentModel`:

```
<model name="ChildModel">
  <link name="L1">
    <pose>0 1 0 0 0 0</pose>
    <visual name="v1">
      <geometry>
        <sphere>
          <radius>0.1</radius>
        </sphere>
      </geometry>
    </visual>
  </link>
  <link name="L2"/>
  <joint name="J1">
    <parent>L1</parent>
    <child>L2</child>
  </joint>
</model>
```

```
<model name="ParentModel">
  <include>
    <uri>/path/to/ChildModel</uri>
    <pose>1 0 1 0 0 0</pose>
  </include>
</model>
```

The result of processing `ParentModel` results in the following model

```
<model name="ParentModel">
  <link name="ChildModel::L1">
    <pose>1 1 1 0 0 0</pose> <!-- Note the modified pose -->
    <visual name="v1"> <!-- Names of child elements of link are not modified -->
      <geometry>
        <sphere>
          <radius>0.1</radius>
        </sphere>
      </geometry>
    </visual>
  </link>
  <link name="ChildModel::L2"/>
  <joint name="ChildModel::J1">
    <parent>ChildModel::L1</parent>
    <child>ChildModel::L2</child>
  </joint>
</model>
```

> **Note** Due to a [bug in libsdformat](https://github.com/osrf/sdformat/issues/219),
the `xyz` vector of joint axes in
nested models is always interpreted to be expressed in the model frame
regardless of the value of the `<use_parent_model_frame>` element.


## Characteristics of nested models 

Whether nested by direct definition or through the use of the `<include>` tag,
the following characteristics are true about nested models.

### Pose and nested models 

The pose of a nested model is expressed with respect to the parent model.
Consider the following example.

```
<model name="Pm">
  <include>
    <uri>/path/to/sphere</uri>
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

The pose of the `sphere` model is translated 0.5 meter in the `z` direction
relative to the origin of `Pm`.

The poses of links inside a nested model are expressed with respect to the
frame of the model that contains them. In the example above, the pose of link
`L1` is translated 1 meter in the `x` direction relative to the origin of `M1`
, which, in turn is translated 1 meter in the `y` direction relative to the origin
of `Pm`.

### Joints and nested models 

Joints can connect links from the parent model to links in the nested model.
Normal rules apply for referencing links found in the parent model. Links in
nested models are referenced by using a `model_name::link_name` syntax where
`model_name` is the name of the model as defined by the `<name>` of the model
definition or as overridden by the `<include><name>`. While this syntax is
imposed by `libsdformat` when model's are composed using the `<include>` tag by
virtue of it prepending link and joint names with `model_name::`, the current
specification does not dictate how child elements are referenced. Nevertheless,
this documentation assumes this syntax as the convention for referencing nested
elements.

The link from the nested model can be referenced in either the `<parent>` or
`<child>` elements of the joint. Note, however, that `libsdformat` does not
currently check that nested model links referenced by joints are valid. This
responsibility is left for downstream applications.

The following snippet demonstrates a fixed joint between one link in the parent
model and another in a nested model:

``` 
<model name="spheres">
  <link name="body"/>
  <include>
    <uri>/path/to/sphere</uri>
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
    <uri>/path/to/sphere</uri>
    <pose>0 0 0.5 0 0 0</pose>
    <name>sphere1</name>
  </include>
  <include>
    <uri>/path/to/sphere</uri>
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
as it breaks encapsulation. The model is no longer standalone, and requires
hard coding the name of the parent sibling model in the definition of the child
model.

## Using `<include>` in `<world>`

When the `<include>` tag is used in worlds the included model is simply copied
into the world with its properties modified as specified by elements such as
`<include><name>` and `<include><pose>`. Other than these modifications, the
model behaves as though it was defined directly inside the world element. The
child elements of the `<include>` tag, such as `<uri>`, `<name>` and `<pose>`
are processed by `libsdformat` as described in the [Including
a model](#including-a-model) section.

The following example shows the use of the `<include>` tag to create a world
that contains multiple instances of the `sphere` model shown in the earlier
example.

```
<world name="W">
  <include>
    <uri>/path/to/sphere</uri>
    <pose>0 0 0.5 0 0 0</pose>
  </include>
  <include>
    <uri>/path/to/sphere</uri>
    <name>sphere1</name>
    <pose>0 0 1 0 0 0</pose>
  </include>
  <include>
    <uri>/path/to/sphere</uri>
    <name>sphere2</name>
    <pose>1 0 1 0 0 0</pose>
  </include>
</model>
```


<!--# References-->

<!--[] https://osrf-migration.github.io/osrf-others-gh-pages/#!/osrf/gazebo_design/pull-requests/18-->

<!--[] http://gazebosim.org/tutorials?tut=nested_model&cat=build_robot-->

<!--[] http://gazebosim.org/tutorials?tut=model_structure&cat=build_robot-->

<!--[] https://osrf-migration.github.io/sdformat-gh-pages/#!/osrf/sdformat/pull-requests/214-->
