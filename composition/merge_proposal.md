# Merging for Composition: Proposed behavior

* **Authors**:
Eric Cousineau `<eric.cousineau@tri.global>`,
Addisu Taddese  `<addisu@openrobotics.org>`
Steve Peters `<scpeters@openrobotics.org>`
* **Status**: Draft
* **SDFormat Version**: 1.9
* **`libsdformat` Version**: 12.0

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
into separate components without visibile changes to downstream consumers,
while maintaining the encapsulation provided by SDFormat 1.8.
The cost of this feature is that users must take care to avoid name collisions
between the entities of the models to be merged.

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

As mentioned in the introduction above, if a user wished to split a model into
separate components and then combine them via composition, those changes may now
become visible to downstream consumers.

For example, say a user had combinatorics of arm + flange + gripper
combinations, and wished to defer the combinatorics to SDFormat's `//include`
composition (instead of a text processing method like `xacro`) such that each
component could be used in composition or isolation. If the user made this
change, it could change the name of relevant interface elements, e.g. links,
joints, and frames. For example, what used to be `composite_arm::gripper_mount`
may now need to become something like `composite_arm::flange::gripper_mount`.

However, with the proposed feature, the user could choose to preserve the
naming at the sites of usage, e.g. `composite_arm::gripper_mount` from above.

## Proposed changes

### `//model/include/@merge`

This adds a new attribute, `@merge`, to `//model/include` tags that when set to
`true` changes the include behavior to insert the contents (links, joints,
frames, plugins, etc.) of the included model file
directly into the parent model without introducing a new scope into the
model hierarchy. Some model elements are not merged: `//model/static`,
`//model/self_collide`, `//model/enable_wind`, and `//model/allow_auto_disable`.

To posture the included model contents via the `//model/include/pose` tag,
a frame is added as a proxy for the implicit `__model__` of the included model.
This proxy frame is assigned the name `_merged__<model_name>__model__`
(where `<model_name>` is the name of the included model),
avoiding a double underscore at the start of the name to respect the reserved
name rules. The proxy frame is attached to the canonical link of the model
to be merged and assigned the pose specified in `//model/include/pose`.
For the entities to be merged, any explicit references to the
implicit `__model__` frame are replaced with references to the proxy frame.
Additionally, the name of the proxy frame is inserted anywhere the is an
implicit reference to the included model's `__model__` frame, such as a link
with an empty `//pose/@relative_to` attribute or a frame with an empty
`@attached_to` attribute.

## Examples

### Small example:

Given the parent model:

~~~
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="world_model">
    <model name="robot">
      <include merge="true">
        <uri>test_model</uri>
        <pose>100 0 0 0 0 0</pose>
      </include>
    </model>
  </world>
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
A proxy frame named `_merged__test_model__model__` is added with the pose
value of `100 0 0 0 0 0` from `//model/include/pose` and attached to `L1`,
which is the canonical link of `test_model`.

~~~
<sdf version='1.9'>
  <world name='world_model'>
    <model name='robot'>
      <frame name='_merged__test_model__model__' attached_to='L1'>
        <pose relative_to='__model__'>100 0 0 0 0 0</pose>
      </frame>
      <link name='L1'>
        <pose relative_to='_merged__test_model__model__'/>
      </link>
      <frame name='F1' attached_to='_merged__test_model__model__'/>
    </model>
  </world>
~~~

### Decomposing an existing model into separate model files

The following is an abridged version of a Clearpath Husky skid-steer with
sensors mounted on a pan-tilt gimbal used in the DARPA Subterranean Challenge
([MARBLE HUSKY SENSOR CONFIG 3](https://app.ignitionrobotics.org/OpenRobotics/fuel/models/MARBLE_HUSKY_SENSOR_CONFIG_3)):

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
        <camera name="camera_pan_tilt">
          <horizontal_fov>1.5184</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.01</near>
            <far>300</far>
          </clip>
          <depth_camera>
            <clip>
              <near>0.1</near>
              <far>10</far>
            </clip>
          </depth_camera>
          <noise>
            <type>gaussian</type>
            <mean>0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <pose>0.0 0 0.03 0 0.0 0</pose>
      </sensor>
      <light name="flashlight_flashlight_light_source_lamp_light" type="spot">
        <pose>0.0 0.0 0.065 3.141592653589793 1.5707963267948966 -0.0015926535897931</pose>
        <attenuation>
          <range>50</range>
          <linear>0</linear>
          <constant>0.1</constant>
          <quadratic>0.0025</quadratic>
        </attenuation>
        <diffuse>0.8 0.8 0.5 1</diffuse>
        <specular>0.8 0.8 0.5 1</specular>
        <spot>
          <!-- The lights on the MARBLE ground vehicles are very wide angle, 100W LEDs -->
          <inner_angle>2.8</inner_angle>
          <outer_angle>2.9</outer_angle>
          <falloff>1</falloff>
        </spot>
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
      filename="libignition-gazebo-joint-controller-system.so"
      name="ignition::gazebo::systems::JointController">
      <joint_name>pan_gimbal_joint</joint_name>
      <use_force_commands>true</use_force_commands>
      <p_gain>0.4</p_gain>
      <i_gain>10</i_gain>
    </plugin>
    <plugin
      filename="libignition-gazebo-joint-controller-system.so"
      name="ignition::gazebo::systems::JointController">
      <joint_name>tilt_gimbal_joint</joint_name>
      <use_force_commands>true</use_force_commands>
      <p_gain>0.4</p_gain>
      <i_gain>10</i_gain>
    </plugin>
    <plugin
      filename="libignition-gazebo-joint-state-publisher-system.so"
      name="ignition::gazebo::systems::JointStatePublisher">
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

and `pan_tilt_sensors_3.sdf` containing the gimbal with sensors
(but excluding `pan_gimbal_joint` since it references `base_link` and would
violate encapsulation to include it in either file):

~~~
<sdf version="1.7">
  <model name="pan_tilt_sensors_3">
    <link name="pan_gimbal_link"/>
    <link name="tilt_gimbal_link">
      <!-- Based on Intel realsense D435 (intrinsics and distortion not modeled)-->
      <sensor name="camera_pan_tilt" type="rgbd_camera">
        <camera name="camera_pan_tilt">
          <horizontal_fov>1.5184</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.01</near>
            <far>300</far>
          </clip>
          <depth_camera>
            <clip>
              <near>0.1</near>
              <far>10</far>
            </clip>
          </depth_camera>
          <noise>
            <type>gaussian</type>
            <mean>0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <pose>0.0 0 0.03 0 0.0 0</pose>
      </sensor>
      <light name="flashlight_flashlight_light_source_lamp_light" type="spot">
        <pose>0.0 0.0 0.065 3.141592653589793 1.5707963267948966 -0.0015926535897931</pose>
        <attenuation>
          <range>50</range>
          <linear>0</linear>
          <constant>0.1</constant>
          <quadratic>0.0025</quadratic>
        </attenuation>
        <diffuse>0.8 0.8 0.5 1</diffuse>
        <specular>0.8 0.8 0.5 1</specular>
        <spot>
          <!-- The lights on the MARBLE ground vehicles are very wide angle, 100W LEDs -->
          <inner_angle>2.8</inner_angle>
          <outer_angle>2.9</outer_angle>
          <falloff>1</falloff>
        </spot>
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
      filename="libignition-gazebo-joint-controller-system.so"
      name="ignition::gazebo::systems::JointController">
      <joint_name>pan_gimbal_joint</joint_name>
      <use_force_commands>true</use_force_commands>
      <p_gain>0.4</p_gain>
      <i_gain>10</i_gain>
    </plugin>
    <plugin
      filename="libignition-gazebo-joint-controller-system.so"
      name="ignition::gazebo::systems::JointController">
      <joint_name>tilt_gimbal_joint</joint_name>
      <use_force_commands>true</use_force_commands>
      <p_gain>0.4</p_gain>
      <i_gain>10</i_gain>
    </plugin>
    <plugin
      filename="libignition-gazebo-joint-state-publisher-system.so"
      name="ignition::gazebo::systems::JointStatePublisher">
      <joint_name>pan_gimbal_joint</joint_name>
      <joint_name>tilt_gimbal_joint</joint_name>
    </plugin>
  </model>
</sdf>
~~~

The split files can then be recomposed as follows by merge-including both
`marble_husky_base.sdf` and `pan_tilt_sensors_3.sdf` alongside the
`pan_gimbal_joint`:

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
        <xyz expressed_in="_merged__pan_tilt_sensors_3__model__">0 0 1</xyz>
      </axis>
    </joint>
  </model>
</sdf>
~~~

## Appendix
