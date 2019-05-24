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

## `<inertial>`

The `<inertial>` tag is a container for inertial properties that can be added
to a `<link>`.

    <link name="link">

      <inertial>
        <pose>{X_LCm}</pose>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx}</ixx>
          <iyy>{iyy}</iyy>
          <izz>{izz}</izz>
          <ixy>{ixy}</ixy>
          <ixz>{ixz}</ixz>
          <iyz>{iyz}</iyz>
        </inertia>
      </inertial>

    </link>

