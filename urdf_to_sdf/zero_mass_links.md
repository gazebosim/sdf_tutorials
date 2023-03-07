# URDF to SDF: Zero mass links

## Definition of a zero mass URDF link

A URDF link which does not have an `<inertial>` tag defined (`//robot/link/inertial`), or has a zero or negative mass (`//robot/link/inertial/mass/@value`).

## Mass of an SDF Link

An SDF link however, is not allowed to have a zero or negative mass value (`//model/link/inertial/mass`). If not defined, the default mass value is 1, in the units of kilograms (kg). A `0` value in `//model/link/inertial/mass` will result in an positive infinitesimal mass determined by the tolerance of `gz::math::MassMatrix3`.

## Conversion of zero mass URDF links to SDF frames

In order to retain the semantics and hierarchy of each element in the URDF, zero mass URDF links are converted into SDF frames explicitly, only when
* it is not a root link
* link has zero mass
* parent joint of link is a fixed joint
* joint lumping is turned off for parent joint

## Examples

### Case 1: Fixed parent joint, joint lumping occurs, conversion not required

URDF example,
```xml
<robot name='test_robot'>
  <link name='link1'>
    <inertial>
      <mass value='0.1' />
      <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
      <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
    </inertial>
  </link>
  <link name='link2'/> <!-- zero mass link -->
  <joint name='joint1_2' type='fixed'>
    <parent link='link1' />
    <child  link='link2' />
    <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
  </joint>
  <link name='link3'>
    <inertial>
      <mass value='0.1' />
      <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
      <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
    </inertial>
  </link>
  <joint name='joint2_3' type='continuous'>
    <parent link='link2' />
    <child  link='link3' />
    <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
  </joint>
</robot>
```

Converted SDF model has a changed hierarchy as joint lumping occured. With `joint2_3` changing its parent to `link1`, while `joint1_2` and `link2` have been converted into frames and are attached to `link1` and `joint1_2` respectively instead.
```xml
<sdf version="1.7">
    <model name="test_robot">
        <link name="link1">
            <inertial>
                <pose>0.123456789123456 0 0 1.570796326794895 0 0</pose>
                <mass>0.1</mass>
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
        <joint type="revolute" name="joint2_3">
            <pose relative_to="link1">0 0 0 0 0 3.14</pose>
            <parent>link1</parent>
            <child>link3</child>
            <axis>
                <xyz>1 0 0</xyz>
                <limit/>
                <dynamics/>
            </axis>
        </joint>
        <link name="link3">
            <pose relative_to="joint2_3"/>
            <inertial>
                <pose>0.123456789123456 0 0 1.570796326794895 0 0</pose>
                <mass>0.1</mass>
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
        <frame name="joint1_2" attached_to="link1">
            <pose>0 0 0 0 -0 1.57</pose>
        </frame>
        <frame name="link2" attached_to="joint1_2"/>
    </model>
</sdf>
```

In this scenario, joint lumping resolves the issue of a zero mass link for the user, without any need for explicit conversion of zero mass URDF link to SDF frame.

### Case 2: Fixed parent joint, joint lumping turned off, zero mass URDF link converted to SDF frame

URDF example,
```xml
<robot name='test_robot'>
  <link name='link1'>
    <inertial>
      <mass value='0.1' />
      <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
      <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
    </inertial>
  </link>
  <link name='link2'/> <!-- zero mass link -->
  <joint name='joint1_2' type='fixed'>
    <parent link='link1' />
    <child  link='link2' />
    <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
  </joint>
  <link name='link3'>
    <inertial>
      <mass value='0.1' />
      <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
      <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
    </inertial>
  </link>
  <joint name='joint2_3' type='continuous'>
    <parent link='link2' />
    <child  link='link3' />
    <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
  </joint>
  <gazebo reference='joint1_2'>
    <disableFixedJointLumping>true</disableFixedJointLumping>
  </gazebo>
  <gazebo reference='joint1_2'>
    <preserveFixedJoint>true</preserveFixedJoint>
  </gazebo>
</robot>
```

`link2` is converted into a frame attached to `link1`, and `joint1_2` is converted into a frame as well, attached to the newly converted frame `link2`. This time, unlike Case 1, the hierarchy stays almost untouched, with `link2` still being the parent of `joint2_3`,
```xml
<sdf version="1.7">
    <model name="test_robot">
        <link name="link1">
            <inertial>
                <pose>0.123456789123456 0 0 1.570796326794895 0 0</pose>
                <mass>0.1</mass>
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
        <frame name="link2" attached_to="link1">
            <pose relative_to="link1">0 0 0 0 0 1.57</pose>
        </frame>
        <frame name="joint1_2" attached_to="link2">
            <pose relative_to="link1">0 0 0 0 0 1.57</pose>
        </frame>
        <joint type="revolute" name="joint2_3">
            <pose relative_to="link2">0 0 0 0 0 1.57</pose>
            <parent>link2</parent>
            <child>link3</child>
            <axis>
                <xyz>1 0 0</xyz>
                <limit/>
                <dynamics/>
            </axis>
        </joint>
        <link name="link3">
            <pose relative_to="joint2_3"/>
            <inertial>
                <pose>0.123456789123456 0 0 1.570796326794895 0 0</pose>
                <mass>0.1</mass>
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

### Case 3: Fixed child joint, joint lumping turned off, conversion does not happen

URDF example,
```xml
<robot name='test_robot'>
  <link name='link1'>
    <inertial>
      <mass value='0.1' />
      <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
      <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
    </inertial>
  </link>
  <link name='link2'/> <!-- zero mass link -->
  <joint name='joint1_2' type='continuous'>
    <parent link='link1' />
    <child  link='link2' />
    <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
  </joint>
  <link name='link3'>
    <inertial>
      <mass value='0.1' />
      <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
      <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
    </inertial>
  </link>
  <joint name='joint2_3' type='fixed'>
    <parent link='link2' />
    <child  link='link3' />
    <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
  </joint>
  <gazebo reference='joint1_2'>
    <disableFixedJointLumping>true</disableFixedJointLumping>
  </gazebo>
  <gazebo reference='joint1_2'>
    <preserveFixedJoint>true</preserveFixedJoint>
  </gazebo>
</robot>
```

In this case, the conversion does not happen as `link2` does not have a fixed parent joint, and users should expect warning messages like,

```bash
Error Code 18: Msg: urdf2sdf: link[link2] has no <inertial> block defined.
Error Code 18: Msg: urdf2sdf: [1] child links ignored.
Error Code 18: Msg: urdf2sdf: [1] child joints ignored.
Error Code 18: Msg: urdf2sdf: parent joint[joint1_2] ignored.
Error Code 18: Msg: urdf2sdf: link[link2] is not modeled in sdf.
Error Code 18: Msg: urdf2sdf: allowing joint lumping by removing the <disableFixedJointLumping> tag or setting it to false on fixed child joint[joint2_3], or setting ParserConfig::URDFPreserveFixedJoint to true, could help resolve this error.
Error Code 18: Msg: urdf2sdf: link[link2] does not have a fixed parent joint, unable to be converted into a frame in sdf.
```

The conversion still carries on to SDF, but with the parent joint of `link2` and all its child elements ignored in the conversion, resulting in the minimal SDF model below,

```xml
<sdf version="1.7">
    <model name="test_robot">
        <link name="link1">
            <inertial>
                <pose>0.123456789123456 0 0 1.570796326794895 0 0</pose>
                <mass>0.1</mass>
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

In this case, the solution is briefly mentioned in one of the warning messages above,

```bash
Error Code 18: Msg: urdf2sdf: allowing joint lumping by removing the <disableFixedJointLumping> tag or setting it to false on fixed child joint[joint2_3], or setting ParserConfig::URDFPreserveFixedJoint to true, could help resolve this error.
```

Users can all joint lumping on the fixed child joint, and allow the parser to convert the child joint and child link into frames while moving all the inertial values into `link2`. Case 4 below is such an example.

### Case 4: Non-fixed parent joint and no fixed child joints

URDF example,
```xml
<robot name='test_robot'>
  <link name='link1'>
    <inertial>
      <mass value='0.1' />
      <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
      <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
    </inertial>
  </link>
  <link name='link2'/> <!-- zero mass link -->
  <joint name='joint1_2' type='continuous'>
    <parent link='link1' />
    <child  link='link2' />
    <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
  </joint>
  <link name='link3'>
    <inertial>
      <mass value='0.1' />
      <origin rpy='1.570796326794895 0 0' xyz='0.123456789123456 0 0.0' />
      <inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01' />
    </inertial>
  </link>
  <joint name='joint2_3' type='fixed'>
    <parent link='link2' />
    <child  link='link3' />
    <origin xyz='0.0 0.0 0.0' rpy='0.0 0.0 1.57'/>
  </joint>
</robot>
```

Allowing joint lumping on the child fixed joint `joint2_3`, the joint lumping mechanism will move the inertial elements of `link3` into `link2`, with the proper transforms, while converting `joint2_3` into a frame attached to `link2`, and `link3` into a frame attached to `joint2_3`.
```xml
<sdf version="1.7">
    <model name="test_robot">
        <link name="link1">
            <inertial>
                <pose>0.123456789123456 0 0 1.570796326794895 0 0</pose>
                <mass>0.1</mass>
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
        <joint type="revolute" name="joint1_2">
            <pose relative_to="link1">0 0 0 0 0 1.57</pose>
            <parent>link1</parent>
            <child>link2</child>
            <axis>
                <xyz>1 0 0</xyz>
                <limit/>
                <dynamics/>
            </axis>
        </joint>
        <link name="link2">
            <pose relative_to="joint1_2"/>
            <inertial>
                <pose>9.831193880037185e-05 0.1234567499792384 0 0 0 0</pose>
                <mass>0.1</mass>
                <inertia>
                    <ixx>0.009999999999999998</ixx>
                    <ixy>-8.470329472543003e-22</ixy>
                    <ixz>0</ixz>
                    <iyy>0.01</iyy>
                    <iyz>0</iyz>
                    <izz>0.009999999999999998</izz>
                </inertia>
            </inertial>
        </link>
        <frame name="joint2_3" attached_to="link2">
            <pose>0 0 0 0 -0 1.57</pose>
        </frame>
        <frame name="link3" attached_to="joint2_3"/>
    </model>
</sdf>
```

### Special case: sensors targeting joints that will be converted to frames, e.g. force-torque sensor

URDF example,
```xml
<robot name="force_torque_sensor_test">
  <link name="base_link">
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>
  <joint name="joint_1" type="fixed">
    <parent link="base_link"/>
    <child link="link_1"/>
  </joint>
  <link name="link_1"/> <!-- zero mass link -->
  <joint name="joint_2" type="revolute">
    <parent link="base_link"/>
    <child link="link_2"/>
    <axis xyz="0 0 1"/>
    <limit effort="1" lower="-1" upper="1" velocity="1"/>
    <dynamics damping="1"/>
  </joint>
  <link name="link_2">
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>
  <gazebo reference='joint_1'>
    <disableFixedJointLumping>true</disableFixedJointLumping>
  </gazebo>
  <gazebo reference='joint_1'>
    <preserveFixedJoint>true</preserveFixedJoint>
  </gazebo>
  <gazebo reference="joint_1">
    <provideFeedback>true</provideFeedback>
    <sensor name="gzft_sensor" type="force_torque">
      <always_on>1</always_on>
      <update_rate>100.0</update_rate>
      <visualize>1</visualize>
      <force_torque>
        <frame>child</frame>
      </force_torque>
    </sensor>
  </gazebo>
</robot>
```

In this example, `link_1` will be converted to a frame attached to `base_link`, while `joint_1` will be converted to a frame as well attached to the newly converted `link_1`. This will cause the force torque sensor `gzft_sensor` to fail to attach to the designated joint `joint_1`.

This is a special case where users will be required to set realistic inertial values to `link_1` to prevent any conversions to frames.
