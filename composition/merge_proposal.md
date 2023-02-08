# Merging for Composition: Proposed behavior

* **Authors**:
Eric Cousineau `<eric.cousineau@tri.global>`,
Addisu Taddese  `<addisu@openrobotics.org>`
Steve Peters `<scpeters@openrobotics.org>`
* **Status**: Accepted
* **SDFormat Version**: 1.9
* **`libsdformat` Version**: 12.0


All sections affected by amendments are explicitly denoted as being added or
modified.

These are added as amendments given that the current proposal has not yet been
migrated to the specification documentation.

### Amendment 1: World merge-include (`//world/include/@merge`)

* **Status**: Draft
* **SDFormat Version**: 1.10
* **`libsdformat` Version**: TBD


## Introduction

This proposal suggests a new behavior for the `//model/include` tag that copies
and merges the contents of a model file into the current model without adding
additional nested model hierarchy or naming scope. The new merging behavior is
used when the `//include/@merge` attribute is `true`.

In SDFormat 1.8 and earlier, composing an SDFormat model file from content
stored in separate files using `//model/include` requires each included model
to be fully encapsulated and creates a nested model hierarchy that mirrors
the file structure used to store the models.
This approach is guaranteed to avoid name collisions but constrains the
model structure to match the file structure and limits the flexibility of model
composition.

The proposed behavior decouples the model structures available via composition
from the file structure used to store the underlying model components.
This is useful both for creating new models and for decomposing existing models
into separate components without visible changes to downstream consumers,
while maintaining the encapsulation provided by SDFormat 1.8.
The cost of this feature is that users must take care to avoid name collisions
between the entities of the models to be merged: consider an analogy to Python
module imports. A normal `//model/include` is similar to a `import my_model`.
If the `//model/include/name` element is specified, then it is is similar to
`import my_model as renamed_model`. With the proposed behavior, a merge include
is effectively the same as `from my_model import *`.

## Document summary

This proposal includes the following sections:

* Motivation: background and rationale.
* Proposed changes: The addition to the SDFormat specification and the
`libsdformat` implementation.
* Examples: Models and workflows using this features.

## Motivation

The `//world/include` tag was first introduced in SDFormat 1.4 to support
insertion of models into a world. This was the first way to compose an SDFormat
document using content from separate SDFormat files.
The `//model/model` tag was added in SDFormat 1.5 to allow a hierarchical
nesting of models and was accompanied by the `//model/include` tag to allow
nested models to be composed using content from separate SDFormat model files,
though the behavior was not entirely consistent
(see [documentation](/tutorials?tut=composition&ver=1.5)).
The behavior of nested models specified using `//model/model` and
`//model/include` was made consistent through improvements to the specification
in SDFormat 1.8
(see [composition proposal](/tutorials?tut=composition_merge_proposal)).
The SDFormat 1.8 specification allows a parent model to reference frames or
entities in nested child models but not the reverse.
This asymmetry enforces hierarchical encapsulation of models and ensures that
each model is fully defined by its own contents and those of its nested models.
Separate name scopes are defined for each model in the hierarchy to avoid name
collisions.

While useful for avoiding name collisions, hierarchical namespacing causes
entity names to change if an existing model file is refactored into components
in separate files that are reconstituted using `//model/include` tags. Those
name changes may break any downstream consumers of those model files that
depend on the existing naming scheme.

For example, if multiple model files use combinations of repeated arm, flange,
and gripper components, storing the repeated components in separate files
and incorporating them with an `//include` tag would reduce duplication of
model content and allow the components to be used in composition or isolation.
Refactoring an existing model in this way could change the names of relevant
interface elements, most significantly links, joints, and frames. A frame
previously called `composite_arm::gripper_mount` may become something like
`composite_arm::flange::gripper_mount` if it was included in a nested model
file named `flange`.

With the proposed feature, however, the user could choose to preserve the
entity names of interface elements that may be referenced
by downstream models, such as `composite_arm::gripper_mount` from above.

## Proposed changes

### `//model/include/@merge`

This adds a new attribute, `@merge`, to `//model/include` tags that when set to
`true` changes the include behavior to insert the contents (links, joints,
frames, plugins, etc.) of the included model file
directly into the parent model without introducing a new scope into the
model hierarchy. Some model elements are not merged: `//model/static`,
`//model/self_collide`, `//model/enable_wind`, and `//model/allow_auto_disable`.

To posture the included model contents via the `//model/include/pose` tag,
a frame is added as a proxy for the implicit `__model__` frame of the included model.
The proxy frame is attached to the canonical link of the model
to be merged and assigned the pose specified in `//model/include/pose`.
If no `//model/@placement_frame` or `//include/placement_frame` is specified,
the raw pose may simply be copied, but in general the model to be merged should
be loaded into an `sdf::Root` object so that graphs are constructed and
the model pose can be resolved (see code in [parser.cc](https://github.com/gazebosim/sdformat/blob/sdformat12_12.4.0/src/parser.cc#L263-L277)).
For the entities to be merged, any explicit references to the
implicit `__model__` frame are replaced with references to the proxy frame.
Additionally, the name of the proxy frame is inserted anywhere there is an
implicit reference to the included model's `__model__` frame, such as a link
with an empty `//pose/@relative_to` attribute or a frame with an empty
`@attached_to` attribute.
The name of the included model's proxy frame is an implementation detail and
is not guaranteed to be stable.
As of [libsdformat 12.4.0](https://github.com/gazebosim/sdformat/blob/sdformat12_12.4.0/src/parser.cc#L2674),
the name of the included model's proxy frame follows the pattern
`_merged__<model_name>__model__` (where `<model_name>` is the name of the
included model), avoiding a double underscore at the start of the name to
respect the reserved name rules.
In order to reference the model frame of a merge-included model, it is
recommended to add an explicitly named `//frame` to the model that is attached
to the included model's `__model__` frame or to use
`//include/experimental:params` to inject such a frame directly (see
[documentation](/tutorials?tut=param_passing_tutorial)).

### `//world/include/@merge`

**Amendment**: This section has been added as part of Amendment 1.

Merge-include in `<world>` would allow models that themselves contain nested
models to be merged into the world such that the nested models are placed
directly in the `<world>` without the additional name scope of the parent
model. The mechanism for merging works the same way as for models except that
links cannot be merged into the world since `//world/link` is not a valid
SDFormat element. Note: as of [libsdformat
13.x](https://github.com/gazebosim/sdformat/pull/1117), `//world/joint` is
included in the spec, thus `//model/joint` is allowed. The parser should emit
errors if it encounters forbidden elements while trying to merge-include models
into the world. 

## Examples

### Small example:

Given the parent model:

~~~
<sdf version="1.9">
  <model name="robot">
    <include merge="true">
      <uri>test_model</uri>
      <pose>100 0 0 0 0 0</pose>
    </include>
  </model>
</sdf>
~~~

and the included model:

~~~
<sdf version="1.9">
  <model name="test_model">
    <link name="L1" />
    <frame name="F1" />
  </model>
</sdf>
~~~

The resulting merged model is shown below.
A proxy frame is added with the pose
value of `100 0 0 0 0 0` from `//model/include/pose` and attached to `L1`,
which is the canonical link of `test_model`.

~~~
<sdf version='1.9'>
  <model name='robot'>
    <frame name='_merged__test_model__model__' attached_to='L1'>
      <pose relative_to='__model__'>100 0 0 0 0 0</pose>
    </frame>
    <link name='L1'>
      <pose relative_to='_merged__test_model__model__'/>
    </link>
    <frame name='F1' attached_to='_merged__test_model__model__'/>
  </model>
</sdf>
~~~

## Example of `//world/include/@merge`

**Amendment**: This example has been added as part of Amendment 1.

Given a world SDFormat file:

~~~xml
<sdf version="1.10">
  <world name="world_model">
    <include merge="true">
      <uri>multiple_robots</uri>
      <pose>100 0 0   0 0 0</pose>
    </include>
  </world>
</sdf>
~~~

and the included model:

~~~xml
<sdf version="1.10">
  <model name="multiple_robots">
    <include>
      <uri>robot</uri> <!-- `robot` is a model form the previous example -->
      <name>robot1</name>
    </include>
    <include>
      <uri>robot</uri> <!-- `robot` is a model form the previous example -->
      <pose>0 10 0   0 0 0</pose>
      <name>robot2</name>
    </include>
  </model>
</sdf>
~~~

The resulting merged world is shown below.
A proxy frame is added with the pose
value of `100 0 0 0 0 0` from `//world/include/pose` and attached to `robot1::L1`,
which is the canonical link of `multiple_robots`.

~~~xml
<sdf version='1.10'>
  <world name="world_model">
    <frame name='_merged__multiple_robots__model__' attached_to='robot1::L1'>
      <pose relative_to='world'>100 0 0 0 0 0</pose>
    </frame>
    <model name='robot1'>
      <pose relative_to='_merged__multiple_robots__model__'/>
      <link name='L1'/>
      <frame name='F1'/>
    </model>
    <model name='robot2'>
      <pose relative_to='_merged__multiple_robots__model__'>0 10 0   0 0 0</pose>
      <link name='L1'/>
      <frame name='F1'/>
    </model>
  </world>
</sdf>
~~~

### Decomposing an existing model into separate model files

The following is an abridged version of a Clearpath Husky skid-steer with
sensors mounted on a pan-tilt gimbal used in the DARPA Subterranean Challenge
([MARBLE HUSKY SENSOR CONFIG 3](https://app.gazebosim.org/OpenRobotics/fuel/models/MARBLE_HUSKY_SENSOR_CONFIG_3/10)):

~~~
<sdf version="1.7">
  <model name="marble_husky_sensor_config_3">
    <link name="base_link"/>

    <link name="front_left_wheel_link">
      <pose>0.256 0.2854 0.03282 0 0 0</pose>
    </link>
    <joint name="front_left_wheel_joint" type="revolute">
      <child>front_left_wheel_link</child>
      <parent>base_link</parent>
      <axis>
        <xyz expressed_in='__model__'>0 1 0</xyz>
      </axis>
    </joint>

    <link name="front_right_wheel_link">
      <pose>0.256 -0.2854 0.03282 0 0 0</pose>
    </link>
    <joint name="front_right_wheel_joint" type="revolute">
      <child>front_right_wheel_link</child>
      <parent>base_link</parent>
      <axis>
        <xyz expressed_in='__model__'>0 1 0</xyz>
      </axis>
    </joint>

    <link name="rear_left_wheel_link">
      <pose>-0.256 0.2854 0.03282 0 0 0</pose>
    </link>
    <joint name="rear_left_wheel_joint" type="revolute">
      <child>rear_left_wheel_link</child>
      <parent>base_link</parent>
      <axis>
        <xyz expressed_in='__model__'>0 1 0</xyz>
      </axis>
    </joint>

    <link name="rear_right_wheel_link">
      <pose>-0.256 -0.2854 0.03282 0 0 0</pose>
    </link>
    <joint name="rear_right_wheel_joint" type="revolute">
      <child>rear_right_wheel_link</child>
      <parent>base_link</parent>
      <axis>
        <xyz expressed_in='__model__'>0 1 0</xyz>
      </axis>
    </joint>

    <link name="pan_gimbal_link">
      <pose>0.424 0 0.427 0 0 0</pose>
    </link>
    <link name="tilt_gimbal_link">
      <pose>0.424 0 0.460 0 0 0</pose>
      <!-- Based on Intel realsense D435 (intrinsics and distortion not modeled)-->
      <sensor name="camera_pan_tilt" type="rgbd_camera">
        <!-- ... -->
      </sensor>
      <light name="flashlight_flashlight_light_source_lamp_light" type="spot">
        <!-- ... -->
      </light>
    </link>
    <joint name="pan_gimbal_joint" type="revolute">
      <child>pan_gimbal_link</child>
      <parent>base_link</parent>
      <axis>
        <xyz expressed_in="__model__">0 0 1</xyz>
      </axis>
    </joint>
    <joint name="tilt_gimbal_joint" type="revolute">
      <child>tilt_gimbal_link</child>
      <parent>pan_gimbal_link</parent>
      <axis>
        <xyz expressed_in="__model__">0 1 0</xyz>
        <limit>
          <lower>-1.5708</lower> <!-- 90 degrees both direction (mechanical interference)-->
          <upper>1.5708</upper>
          <effort>10</effort>
        </limit>
      </axis>
    </joint>
    <!-- Gimbal Joints Plugins -->
    <plugin
      filename="libgz-gazebo-joint-controller-system.so"
      name="gz::sim::systems::JointController">
      <joint_name>pan_gimbal_joint</joint_name>
      <use_force_commands>true</use_force_commands>
      <p_gain>0.4</p_gain>
      <i_gain>10</i_gain>
    </plugin>
    <plugin
      filename="libgz-gazebo-joint-controller-system.so"
      name="gz::sim::systems::JointController">
      <joint_name>tilt_gimbal_joint</joint_name>
      <use_force_commands>true</use_force_commands>
      <p_gain>0.4</p_gain>
      <i_gain>10</i_gain>
    </plugin>
    <plugin
      filename="libgz-gazebo-joint-state-publisher-system.so"
      name="gz::sim::systems::JointStatePublisher">
      <joint_name>pan_gimbal_joint</joint_name>
      <joint_name>tilt_gimbal_joint</joint_name>
    </plugin>
  </model>
</sdf>
~~~

This model fle can be decomposed into `marble_husky_base.sdf` containing the
chassis and wheels:

~~~
<sdf version="1.7">
  <model name="marble_husky_base">
    <link name="base_link"/>

    <link name="front_left_wheel_link">
      <pose>0.256 0.2854 0.03282 0 0 0</pose>
    </link>
    <joint name="front_left_wheel_joint" type="revolute">
      <child>front_left_wheel_link</child>
      <parent>base_link</parent>
      <axis>
        <xyz expressed_in='__model__'>0 1 0</xyz>
      </axis>
    </joint>

    <link name="front_right_wheel_link">
      <pose>0.256 -0.2854 0.03282 0 0 0</pose>
    </link>
    <joint name="front_right_wheel_joint" type="revolute">
      <child>front_right_wheel_link</child>
      <parent>base_link</parent>
      <axis>
        <xyz expressed_in='__model__'>0 1 0</xyz>
      </axis>
    </joint>

    <link name="rear_left_wheel_link">
      <pose>-0.256 0.2854 0.03282 0 0 0</pose>
    </link>
    <joint name="rear_left_wheel_joint" type="revolute">
      <child>rear_left_wheel_link</child>
      <parent>base_link</parent>
      <axis>
        <xyz expressed_in='__model__'>0 1 0</xyz>
      </axis>
    </joint>

    <link name="rear_right_wheel_link">
      <pose>-0.256 -0.2854 0.03282 0 0 0</pose>
    </link>
    <joint name="rear_right_wheel_joint" type="revolute">
      <child>rear_right_wheel_link</child>
      <parent>base_link</parent>
      <axis>
        <xyz expressed_in='__model__'>0 1 0</xyz>
      </axis>
    </joint>
  </model>
</sdf>
~~~

and `pan_tilt_sensors_3.sdf` containing an additional explicit frame named
`pan_tilt_sensors_3_model` that is attached to the `__model__` frame by default
along with the gimbal with sensors
(but excluding `pan_gimbal_joint` since it references `base_link` and would
violate encapsulation to include it in either file):

~~~
<sdf version="1.7">
  <model name="pan_tilt_sensors_3">
    <frame name="pan_tilt_sensors_3_model"/>
    <link name="pan_gimbal_link"/>
    <link name="tilt_gimbal_link">
      <!-- Based on Intel realsense D435 (intrinsics and distortion not modeled)-->
      <sensor name="camera_pan_tilt" type="rgbd_camera">
        <!-- ... -->
      </sensor>
      <light name="flashlight_flashlight_light_source_lamp_light" type="spot">
        <!-- ... -->
      </light>
    </link>
    <joint name="tilt_gimbal_joint" type="revolute">
      <child>tilt_gimbal_link</child>
      <parent>pan_gimbal_link</parent>
      <axis>
        <xyz expressed_in="__model__">0 1 0</xyz>
        <limit>
          <lower>-1.5708</lower> <!-- 90 degrees both direction (mechanical interference)-->
          <upper>1.5708</upper>
          <effort>10</effort>
        </limit>
      </axis>
    </joint>
    <!-- Gimbal Joints Plugins -->
    <plugin
      filename="libgz-gazebo-joint-controller-system.so"
      name="gz::sim::systems::JointController">
      <joint_name>pan_gimbal_joint</joint_name>
      <use_force_commands>true</use_force_commands>
      <p_gain>0.4</p_gain>
      <i_gain>10</i_gain>
    </plugin>
    <plugin
      filename="libgz-gazebo-joint-controller-system.so"
      name="gz::sim::systems::JointController">
      <joint_name>tilt_gimbal_joint</joint_name>
      <use_force_commands>true</use_force_commands>
      <p_gain>0.4</p_gain>
      <i_gain>10</i_gain>
    </plugin>
    <plugin
      filename="libgz-gazebo-joint-state-publisher-system.so"
      name="gz::sim::systems::JointStatePublisher">
      <joint_name>pan_gimbal_joint</joint_name>
      <joint_name>tilt_gimbal_joint</joint_name>
    </plugin>
  </model>
</sdf>
~~~

The split files can then be recomposed as follows by merge-including both
`marble_husky_base.sdf` and `pan_tilt_sensors_3.sdf` alongside the
`pan_gimbal_joint`, which uses the explictly named frame
`pan_tilt_sensors_3_model` to express the `//joint/axis/xyz` value.

~~~
<sdf version="1.9">
  <model name="marble_husky_sensor_config_3_recomposed">
    <include merge="true">
      <uri>marble_husky_base.sdf</uri>
    </include>

    <include merge="true">
      <uri>pan_tilt_sensors_3.sdf</uri>
      <pose>0.424 0 0.427 0 0 0</pose>
    </include>

    <joint name="pan_gimbal_joint" type="revolute">
      <child>pan_gimbal_link</child>
      <parent>base_link</parent>
      <axis>
        <xyz expressed_in="pan_tilt_sensors_3_model">0 0 1</xyz>
      </axis>
    </joint>
  </model>
</sdf>
~~~

## Appendix

(Unused)
