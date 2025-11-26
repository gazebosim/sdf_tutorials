# SDFormat extensions to URDF (the `<gazebo>` tag)

When URDF files are loaded by Gazebo (both Gazebo-classic and the new Gazebo),
the URDF content is first converted to SDFormat before being processed by
Gazebo. Users who want to modify the resulting SDFormat output to include SDFormat
specific elements may do so using the `<gazebo>` tag in the original URDF file.
This is known as the Gazebo extension to the [URDF
specification](http://wiki.ros.org/urdf/XML) (see
http://wiki.ros.org/urdf/XML/Gazebo)

The URDF to SDFormat conversion usually happens automatically without users
observing the resulting SDFormat file. To diagnose any issues that might come
up during the conversion, the tool

```gz sdf -p <path to urdf file>```

can be used to convert the URDF file to SDFormat that can be inspected by the user.

The following is the documentation of the various tags available under
`<gazebo>` and their corresponding effect on the SDFormat output. While some of
the content included here is also available in the ["Using a URDF in
Gazebo"](http://gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros) tutorial
of Gazebo-classic, this document should provide a more complete discussion of
the extension, especially regarding fixed joint lumping.


## `<gazebo>` Elements For `<robot>`
`<gazebo>` tag without a `reference` attribute applies to the SDFormat `<model>`
that gets generated from the `<robot>` URDF tag. All elements in the `<gazebo>`
tag are inserted into the SDF `<model>` tag for the generated SDF.

**Example:**

<include src='https://github.com/gazebosim/sdf_tutorials/raw/master/urdf/examples/no_ref_example.urdf' />

results in:

```xml
<!--SDFormat-->
<sdf version='1.9'>
  <model name='no_ref_example'>
    <static>true</static>
    <plugin name='testPlugin' filename='testFileName'/>
  </model>
</sdf>
```

## `<gazebo>` Elements For Links

When using the `<gazebo>` extension for links, the name of the link has to be
specified in the `reference` attribute. There are a number of special tags that
modify the values of elements or attributes in the generated SDFormat file. Any
tag that is not listed in the table below will be directly inserted into the
corresponding `<link>` element in the SDFormat output. This direct insertion is
sometimes referred to as blob insertion.

**Table of elements with special meaning:**

<table>
  <tr>
    <th>Name</th>
    <th>Type</th>
    <th>Description</th>
    <th>Corresponding SDFormat element</th>
  </tr>
  <tr>
    <td>turnGravityOff</td>
    <td>bool</td>
    <td>A value of "true" turns gravity off. Alternatively, <code>gravity</code> (with opposite boolean value) can be used via blob insertion</td>
    <td><code>gravity</code></td>
  </tr>
  <tr>
    <td>dampingFactor</td>
    <td>double</td>
    <td>Exponential velocity decay of the link velocity - takes the value and multiplies the previous link velocity by (1-dampingFactor).</td>
    <td><code>velocity_decay/linear</code> and <code>velocity_decay/angular</code></td>
  </tr>
  <tr>
    <td>maxVel</td>
    <td>double</td>
    <td>Maximum contact correction velocity truncation term. (See the 
    Gazebo-classic tutorial on <a 
    href="https://classic.gazebosim.org/tutorials?tut=physics_params&cat=physics#Constraintsparameters">Constraints 
    Parameters</a> for more detail)</td>
    <td><code>collision/surface/contact/ode/max_vel</code></td>
  </tr>
  <tr>
    <td>minDepth</td>
    <td>double</td>
    <td>Minimum allowable depth before contact correction impulse is applied. 
    (See the Gazebo-classic tutorial on <a 
    href="https://classic.gazebosim.org/tutorials?tut=physics_params&cat=physics#Constraintsparameters">Constraints 
    Parameters</a> for more detail)</td>
    <td><code>collision/surface/contact/ode/min_depth</code></td>
  </tr>
  <tr>
    <td>mu1</td>
    <td rowspan="2">double</td>
    <td rowspan="2">Friction coefficients μ for the principal contact directions along the contact surface as defined by the
      <a href="http://www.ode.org">Open Dynamics Engine (ODE)</a>
      (see parameter descriptions in <a 
      href="http://www.ode.org/ode-latest-userguide.html#sec_7_3_7">ODE's user 
      guide</a> and the Gazebo-classic tutorial on <a 
      href="https://classic.gazebosim.org/tutorials?tut=physics_params&cat=physics#Frictionparameters">Friction 
      Parameters</a> for more detail)
    </td>
    <td><code>collision/surface/friction/ode/mu</code></td>
  </tr>
  <tr>
    <td>mu2</td>
    <td><code>collision/surface/friction/ode/mu2</code></td>
  </tr>
  <tr>
    <td>fdir1</td>
    <td>vector</td>
    <td>3-tuple specifying direction of mu1 in the collision local reference 
    frame. (See the Gazebo-classic tutorial on <a 
    href="https://classic.gazebosim.org/tutorials?tut=physics_params&cat=physics#Frictionparameters">Friction 
    Parameters</a> for more detail)</td>
    <td><code>collision/surface/friction/ode/fdir1</code></td>
  </tr>
  <tr>
    <td>kp</td>
    <td rowspan="2">double</td>
    <td rowspan="2">Contact stiffness k_p and damping k_d for rigid body contacts as defined by ODE
      (<a href="http://www.ode.org/ode-latest-userguide.html#sec_7_3_7">ODE uses erp and cfm</a>
      but there is a
      <a 
      href="https://github.com/osrf/gazebo/blob/gazebo9/gazebo/physics/ode/ODEJoint.cc">mapping 
      between erp/cfm and stiffness/damping</a>. Also see See the 
      Gazebo-classic tutorial on <a 
      href="https://classic.gazebosim.org/tutorials?tut=physics_params&cat=physics#Contactparameters">Contact 
      Parameters</a> for more detail)
    </td>
    <td><code>collision/surface/contact/ode/kp</code></td>
  </tr>
  <tr>
    <td>kd</td>
    <td><code>collision/surface/contact/ode/kd</code></td>
  </tr>
  <tr>
    <td>selfCollide</td>
    <td>bool</td>
    <td>If true, the link can collide with other links in the model.</td>
    <td><code>self_collide</code></td>
  </tr>
  <tr>
    <td>maxContacts</td>
    <td>int</td>
    <td>Maximum number of contacts allowed between two entities. This value overrides the max_contacts element defined in physics.(See the Gazebo-classic tutorial on <a 
    href="https://classic.gazebosim.org/tutorials?tut=physics_params&cat=physics#Contactparameters">Contact 
    Parameters</a> for more detail)</td>
    <td><code>collision/max_contacts</code></td>
  </tr>
  <tr>
    <td>laserRetro</td>
    <td>double</td>
    <td>Intensity value returned by laser sensor.</td>
    <td><code>collision/laser_retro</code></td>
  </tr>
  <tr>
    <td>visual</td>
    <td>element</td>
    <td>The content of the element will be inserted into each <code>visual</code> of the SDFormat link</td>
    <td><code>visual</code></td>
  </tr>
  <tr>
    <td>material</td>
    <td>element</td>
    <td>The content of the element will be inserted into each <code>material</code> of the SDFormat link</td>
    <td><code>visual/material</code></td>
  </tr>
  <tr>
    <td>collision</td>
    <td>element</td>
    <td>The content of the element will be inserted into each <code>collision</code> of the SDFormat link</td>
    <td><code>collision</code></td>
  </tr>
</table>

> **Note:** The XPath used in the "Corresponding SDFormat element" column is relative to the link element.

**Example:**

The following shows how to set the first coefficient of friction for all
`<collision>` elements in a link

<include src='https://github.com/gazebosim/sdf_tutorials/raw/master/urdf/examples/friction_example.urdf' />

This creates the element `//surface/friction/ode/mu` in the collision element of the referenced link.

```xml
<!--SDFormat-->
<sdf version='1.9'>
  <model name='friction_example'>
    <link name='base_link'>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>0.12</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name='base_link_collision'>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>2</radius>
          </sphere>
        </geometry>
        <surface>
          <contact>
            <ode/>
          </contact>
          <friction>
            <ode>
              <mu>0.25</mu>
            </ode>
          </friction>
        </surface>
      </collision>
      <collision name='base_link_collision_1'>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <cylinder>
            <length>2</length>
            <radius>1</radius>
          </cylinder>
        </geometry>
        <surface>
          <contact>
            <ode/>
          </contact>
          <friction>
            <ode>
              <mu>0.25</mu>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
  </model>
</sdf>
```

## Special meaning for `<visual>`, `<collision>`, and `<material>`

The `<visual>` and `<collision>` are meant to update existing visuals and
collisions in the URDF as they get converted to SDFormat. At the time of
writing, these tags do not insert new visuals or collision elements into the
referenced link. Note also that these tags affect all visuals and collisions,
respectively, found in the referenced link.

**Example:**

Given the following URDF file with two visuals, the `<gazebo>` extension
applies the element `<transparency>` to each visual in `base_link`.

<include src='https://github.com/gazebosim/sdf_tutorials/raw/master/urdf/examples/visual_example.urdf' />

Converts to the following SDFormat

```xml
<!--SDFormat-->
<sdf version='1.9'>
  <model name='visual_example'>
    <link name='base_link'>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>0.12</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <visual name='base_link_visual'>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>2</radius>
          </sphere>
        </geometry>
        <transparency>0.25</transparency>
      </visual>
      <visual name='base_link_visual_1'>
        <pose>2 0 0 0 0 0</pose>
        <geometry>
          <cylinder>
            <length>1</length>
            <radius>2</radius>
          </cylinder>
        </geometry>
        <transparency>0.25</transparency>
      </visual>
    </link>
  </model>
</sdf>

```



The `<material>` tag, when used directly under the `<gazebo>` tag, i.e
`//gazebo/material`, accepts a string value of the name of a material defined
in a Gazebo-classic's material [script](
https://github.com/gazebosim/gazebo-classic/blob/d0c34b8a5d6bc9a3271000fb1baeb3f8d9f43afa/media/materials/scripts/gazebo.material).
Examples include colors like `Gazebo/SkyBlue` as well as textures such as
`Gazebo/WoodFloor`. The `<material>` tag affects all visuals found in the
referenced link, similar to the behavior of `<visual>` and `<collision>`

**Example:**

<include src='https://github.com/gazebosim/sdf_tutorials/raw/master/urdf/examples/material_example.urdf' />

results in:

```xml
<!--SDFormat-->
<sdf version='1.9'>
  <model name='material_example'>
    <link name='base_link'>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>0.10000000000000001</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <visual name='base_link_visual'>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>2</radius>
          </sphere>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Orange</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

This tag is only relevant when using Gazebo-classic as the new version of
Gazebo does not use material scripts. However, it is still possible to change
the appearance of visuals that is compatible with the new Gazebo and this is by
using the `//gazebo/visual/material` tag. Note the difference from the previous
tag as the `<material>` tag is not directly under `<gazebo>`, but under
`<visual>`. This `<material>` tag contains child elements as defined in the
[material](http://sdformat.org/spec?ver=1.9&elem=material) specification.

**Example:**

<include src='https://github.com/gazebosim/sdf_tutorials/raw/master/urdf/examples/material_visual_example.urdf' />

results in:

```xml
<!--SDFormat-->
<sdf version='1.9'>
  <model name='material_example'>
    <link name='base_link'>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>0.12</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <visual name='base_link_visual'>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>2</radius>
          </sphere>
        </geometry>
        <material>
          <diffuse>0 0 1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

where the `<diffuse>` tag has been added to the visual of `base_link`.

> **Warning:**
At the time of writing, there is
a [bug](https://github.com/gazebosim/sdformat/issues/333) in the URDF to
SDFormat converter that behaves incorrectly when `<visual>` and `<collision>`
tags in the **URDF** file have names.

### `<gazebo>` Elements For Joints

When using the `<gazebo>` extension for joints, the name of the joint has to be
specified in the `reference` attribute. There are a number of special tags that
modify the values of elements or attributes in the generated SDFormat file. Any
tag that is not listed in the table below will be directly inserted into the 
corresponding `<joint>` element in the SDFormat output.


**Table of elements with special meaning:**

<table>
  <tr>
    <th>Name</th>
    <th>Type</th>
    <th>Description</th>
    <th>Corresponding SDFormat element</th>
  </tr>
  <tr>
    <td>stopCfm</td>
    <td rowspan="2">double</td>
    <td rowspan="2">Joint stop constraint force mixing (cfm) and error reduction parameter (erp) used by ODE</td>
    <td><code>physics/ode/limit/cfm</code></td>
  </tr>
  <tr>
    <td>stopErp</td>
    <td><code>physics/ode/limit/erp</code></td>
  </tr>
  <tr>
    <td>provideFeedback</td>
    <td>bool</td>
    <td>Allows joints to publish their wrench data (force-torque) via a Gazebo plugin</td>
    <td><code>physics/provide_feedback</code> and <code>physics/ode/provide_feedback</code></td>
  </tr>
  <tr>
    <td>implicitSpringDamper</td>
    <td>bool</td>
    <td>If this flag is set to true, ODE will use ERP and CFM to simulate damping.
    This is a more stable numerical method for damping than the default damping tag.
    The cfmDamping element is deprecated and should be changed to implicitSpringDamper.
    </td>
    <td><code>physics/ode/implicit_spring_damper</code></td>
  </tr>
  <tr>
    <td>springStiffness</td>
    <td>double</td>
    <td>Spring stiffness in N/m.</td>
    <td><code>axis/dynamics/spring_stiffness</code></td>
  </tr>
  <tr>
    <td>springReference</td>
    <td>double</td>
    <td>Equilibrium position for the spring.
    <td><code>axis/dynamics/spring_reference</code></td>
  </td>
  </tr>
  <tr>
    <td>fudgeFactor</td>
    <td>double</td>
    <td>Scale the excess for in a joint motor at joint limits. Should be between zero and one.</td>
    <td><code>physics/ode/fudge_factor</code></td>
  </tr>
  <tr>
    <td>preserveFixedJoint</td>
    <td>bool</td>
    <td>By default, fixed joints in the URDF are "lumped", meaning that the
    contents of the child link are merged with the parent link with appropriate
    pose offsets and the joint is discarded. Setting this to <code>true</code>
    preserves the fixed joint and effectively disables fixed joint
    lumping.</td>
    <td></td>
  </tr>
  <tr>
    <td>disableFixedJointLumping</td>
    <td>bool</td>
    <td>By default, fixed joints in the URDF are "lumped", meaning that the
    contents of the child link are merged with the parent link with appropriate
    pose offsets and the joint is discarded. Setting this to <code>true</code>
    disables fixed joint lumping. This has a similar effect as <code>preserveFixedJoint</code>
    but, for backward compatibility reasons, replaces the fixed joint with a
    revolute joint with position limits set to 0. Users are encouraged to use
    <code>preserveFixedJoint</code> instead.
    </td>
    <td></td>
  </tr>
</table>

> **Note:** The XPath used in the "Corresponding SDFormat element" column is relative to the joint element.


**Example:**
The spring reference and stiffness of a joint can be set using
`<springReference>` and `<springStiffness>` respectively.

<include src='https://github.com/gazebosim/sdf_tutorials/raw/master/urdf/examples/joint_example.urdf' />

This creates the elements `//axis/dynamics/spring_reference`
and `//axis/dynamics//spring_stiffness` in the referenced joint
of the SDFormat output.

```xml
<!--SDFormat-->
<sdf version='1.9'>
  <model name='joint_example'>
    <link name='base_link'>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>0.12</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
    </link>
    <joint name='j1' type='revolute'>
      <pose relative_to='base_link'>0 0 1 0 0 0</pose>
      <parent>base_link</parent>
      <child>end_effector</child>
      <axis>
        <xyz>1 0 0</xyz>
        <limit>
          <lower>-10000000000000000</lower>
          <upper>10000000000000000</upper>
        </limit>
        <dynamics>
          <spring_reference>0.5</spring_reference>
          <spring_stiffness>0.25</spring_stiffness>
        </dynamics>
      </axis>
      <physics>
        <ode>
          <limit>
            <cfm>0</cfm>
            <erp>0.20000000000000001</erp>
          </limit>
        </ode>
      </physics>
    </joint>
    <link name='end_effector'>
      <pose relative_to='j1'>0 0 0 0 0 0</pose>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>0.12</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>
```

### Fixed joint lumping

Fixed joint lumping (aka joint reduction), in the URDF to SDFormat conversion
is the process of taking the child link of a *fixed* joint and merging all of
its contents into the parent link. It is an optimization that benefits maximal 
coordinate physics engines by reducing the number of constraints needed to 
simulate the model. The process preserves the total mass of the
two links and computes the center of mass and moment of inertia of the
resultant link. All `visual` and `collision` elements present in the child link
are moved to the parent link with appropriate pose offsets. The fixed joint
itself is discarded and does not appear in the SDFormat output. As of
`libsdformat 9.9.0` `frame` elements that represent the discarded joint and
child link are generated to preserve their pose information.

Fixed joint lumping is enabled by default, but can be disabled by setting
`preserveFixedJoint` or `disableFixedJointLumping` to `true`. The two 
parameters behave similarly, but the `preserveFixedJoint=true` configuration 
results in a joint with a `fixed` type whereas the 
`disableFixedJointLumping=true` configuration results in a revolute joint with 
position limits set to 0. Note that when both `preserveFixedJoint=true` and 
`disableFixedJointLumping=true` are set on a joint, the `preserveFixedJoint` 
setting will take precedence and the resulting joint will have a `fixed` type.
Fixed joint lumping can also be disabled for all joints if 
`ParserConfig::URDFPreserveFixedJoint` is `true`.

> **Warning:** Disabling joint lumping should only be done when both parent and
> child links have positive mass and corresponding `<inertial>` elements.



**Example:** The following URDF demonstrates fixed joint lumping where the
resulting SDFormat output only has one link

<include src='https://github.com/gazebosim/sdf_tutorials/raw/master/urdf/examples/fixed_joint_lumping_example.urdf' />

results in:

```xml
<!--SDFormat-->
<sdf version='1.9'>
  <model name='fixed_joint_lumping_example'>
    <link name='base_link'>
      <inertial>
        <pose>0 0 0.5 0 0 0</pose>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.14499999999999999</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.14499999999999999</iyy>
          <iyz>0</iyz>
          <izz>0.02</izz>
        </inertia>
      </inertial>
      <collision name='base_link_collision'>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>2</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name='base_link_fixed_joint_lump__end_effector_visual'>
        <pose>2 0 1 0 0 0</pose>
        <geometry>
          <cylinder>
            <length>1</length>
            <radius>2</radius>
          </cylinder>
        </geometry>
      </visual>
    </link>
    <frame name='j1' attached_to='base_link'>
      <pose>0 0 1 0 -0 0</pose>
    </frame>
    <frame name='end_effector' attached_to='j1'/>
  </model>
</sdf>
```

Note that the mass of `base_link` is the sum of the masses of the original
`base_link` and `end_effector` links. The `visual` element of `end_effector`
has been merged into `base_link` with a pose value that takes into account the
pose of the original `end_effector` link and joint `j1` as well as the pose of
the original `visual`.

**Example:** The same example above is repeated, but fixed joints preserved (`preserveFixedJoint=true`).

<include src='https://github.com/gazebosim/sdf_tutorials/raw/master/urdf/examples/preserve_fixed_joint_example.urdf' />

results in:

```xml
<!--SDFormat-->
<sdf version='1.9'>
  <model name='preserve_fixed_joint_lumping_example'>
    <link name='base_link'>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>0.25</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name='base_link_collision'>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>2</radius>
          </sphere>
        </geometry>
      </collision>
    </link>
    <joint name='j1' type='fixed'>
      <pose relative_to='base_link'>0 0 1 0 0 0</pose>
      <parent>base_link</parent>
      <child>end_effector</child>
      <axis>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-10000000000000000</lower>
          <upper>10000000000000000</upper>
        </limit>
      </axis>
      <physics>
        <ode>
          <limit>
            <cfm>0</cfm>
            <erp>0.20000000000000001</erp>
          </limit>
        </ode>
      </physics>
    </joint>
    <link name='end_effector'>
      <pose relative_to='j1'>0 0 0 0 0 0</pose>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>0.25</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <visual name='end_effector_visual'>
        <pose>2 0 0 0 0 0</pose>
        <geometry>
          <cylinder>
            <length>1</length>
            <radius>2</radius>
          </cylinder>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

Here, the link `end_effector` is still present in the SDFormat output with mass
and inertia equal to the URDF `end_effector` link. The `visual` of
`end_effector` is also still present in the `end_effector` link of the SDFormat
output. The joint `j1` is also still present and its type is `fixed`.

**Example:** The same example above is repeated, but with fixed joint lumping
disabled (`disableFixedJointLumping=true`).

<include src='https://github.com/gazebosim/sdf_tutorials/raw/master/urdf/examples/disable_fixed_joint_lumping_example.urdf' />

results in:

```xml
<!--SDFormat-->
<sdf version='1.9'>
  <model name='disable_fixed_joint_lumping_example'>
    <link name='base_link'>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>0.25</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name='base_link_collision'>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>2</radius>
          </sphere>
        </geometry>
      </collision>
    </link>
    <joint name='j1' type='revolute'>
      <pose relative_to='base_link'>0 0 1 0 0 0</pose>
      <parent>base_link</parent>
      <child>end_effector</child>
      <axis>
        <limit>
          <lower>0</lower>
          <upper>0</upper>
        </limit>
        <dynamics>
          <damping>0</damping>
          <friction>0</friction>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
        <xyz>0 0 1</xyz>
      </axis>
      <physics>
        <ode>
          <limit>
            <cfm>0</cfm>
            <erp>0.20000000000000001</erp>
          </limit>
        </ode>
      </physics>
    </joint>
    <link name='end_effector'>
      <pose relative_to='j1'>0 0 0 0 0 0</pose>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>0.25</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <visual name='end_effector_visual'>
        <pose>2 0 0 0 0 0</pose>
        <geometry>
          <cylinder>
            <length>1</length>
            <radius>2</radius>
          </cylinder>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

The output is the same as `preserveFixedJoint=true` example, execept that joint 
`j1` has a `revolute` type. 
