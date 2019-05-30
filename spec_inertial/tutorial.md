# Specifying inertial properties in SDFormat

This tutorial describes how to use SDFormat to specify the inertial properties
of links and models.
Inertial properties for a single link include its mass, moment of inertia
matrix, and the location of its center of mass.
These parameters help determine how the link moves in response to external
forces acting on the body.
For models that should not move at all, no matter the external forces that
act on them, the model can be marked as a static model.

## `<static>`

The `<static>` tag can be set to true in a `<model>` to mark the model as
static, in which case all inertial properties for links are ignored, and the
links are fixed in place at their initial pose.
The default value is false.
In the following example, a world consists of a ball that falls on a static
box.

    <world name="ball_falling_ground">

      <model name="ball">
        <!-- <static> is not set, defaults to false. The ball will fall. -->
        <pose>0 0 10 0 0 0</pose>
        <link name="link">
          <collision name="collision">
            <geometry>
              <sphere>
                <radius>0.5</radius>
              </sphere>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <sphere>
                <radius>0.5</radius>
              </sphere>
            </geometry>
          </visual>
        </link>
      </model>

      <model name="ground_box">

        <static>1</static>  <!-- The box will not move. -->

        <link name="flat_box">
          <collision name="collision">
            <geometry>
              <box>
                <size>10 10 0.1</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <box>
                <size>10 10 0.1</size>
              </box>
            </geometry>
          </visual>
        </link>
      </model>

    </world>

## `<inertial>`

The `<inertial>` tag is a container for inertial properties that can be added
to a `<link>`, including mass, moment of inertia, and center of mass (Cm) pose.
An example usage of `<inertial>` is shown in the following snippet.

    <link name="L">

      <inertial>
        <pose>{X_LCm}</pose>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx}</ixx>
          <ixy>{ixy}</ixy>
          <ixz>{ixz}</ixz>
          <iyy>{iyy}</iyy>
          <iyz>{iyz}</iyz>
          <izz>{izz}</izz>
        </inertia>
      </inertial>

    </link>

### `<mass>`

The mass of the link in kilograms (kg) is specified in the `<mass>` tag.
The mass affects both the weight of an object in a gravitational field
and the resistance to linear acceleration from an applied linear force.

### `<pose>`

The `<pose>` in the `<inertial>` block specifies the pose of a body-fixed
link inertia frame.
The link's center of mass is located at the origin of this frame,
and the moment of inertia matrix is interpreted in the coordinates of
this frame.
Note that *inertial frame* is a term of art that typically refers
to a non-accelerating frame and not a body-fixed frame, so that label
is avoided.
If the pose is unspecified, it defaults to an identity pose.

### `<inertia>`

The `<inertia>` tag is used to specify the components of the moment of
the link's 3x3 moment of inertia matrix with respect to the center of mass
and using the coordinates of the link inertia frame.
Since the moment of inertia matrix is symmetric, only 6 components are
needed.
For example, the following symmetric matrix:

    | ixx   ixy   ixz |
    | ixy   iyy   iyz |
    | ixz   iyz   izz |

can be specified with the `<inertia>` tag as follows:

    <inertia>
      <ixx>{ixx}</ixx>
      <ixy>{ixy}</ixy>
      <ixz>{ixz}</ixz>
      <iyy>{iyy}</iyy>
      <iyz>{iyz}</iyz>
      <izz>{izz}</izz>
    </inertia>

If the moment of inertia components are not specified, it defaults to a
3x3 identity matrix.
