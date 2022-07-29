# Python Get Started

## Overview

This tutorial describes how to get started using SDFormat library with Python.
This Python bindings are available from SDFormat13 (Gazebo Garden).

**NOTE**: If you have compiled SDFormat from source, you should export your `PYTHONPATH`.

```bash
export PYTHONPATH=$PYTHONPATH:<path to your workspace>/install/lib/python
```

## Tutorial

We will run through an example that read a SDF file, modify the structure and
create a new file. Start by creating the following file SDF file `sphere.sdf`
using the editor of your choice:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="sphere_world">
    <model name="sphere">
      <pose>0 1.5 0.5 0 0 0</pose>
      <link name="sphere_link">
        <inertial>
          <inertia>
            <ixx>3</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>3</iyy>
            <iyz>0</iyz>
            <izz>3</izz>
          </inertia>
          <mass>3.0</mass>
        </inertial>
        <collision name="sphere_collision">
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="sphere_visual">
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
            <specular>0 0 1 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

Now let's create a bare-bones main Python file:

```python
def main():
  pass

if __name__ == "__main__":
  main()
```

The easiest way to include SDFormat is through `import sdformat as sdf`.

At this point your main file should look like

```python
import sdformat as sdf

def main():
    pass

if __name__ == "__main__":
    main()
```

Next step it's to read the file. The `load` method might throw an exception,
for this reason we will use `try/except` and if there is any exception we will
print the error in the terminal.

```python
import sdformat as sdf
import sys


def main():
    input_file = 'sphere.sdf'
    root = sdf.Root()
    try:
        root.load(input_file)
    except sdf.SDFErrorsException as e:
        print(e, file=sys.stderr)

if __name__ == "__main__":
    main()
```

We can iterate through the SDF file showing the name of the different elements:

```python
def main():
    input_file = './sphere.sdf'
    root = sdf.Root()
    try:
        root.load(input_file)
    except sdf.SDFErrorsException as e:
        print(e, file=sys.stderr)

    for world_index in range(root.world_count()):
        world = root.world_by_index(world_index)
        print(world.name())

        for model_index in range(world.model_count()):
            model = world.model_by_index(model_index)
            print("\tModel: ", model.name())
            for link_index in range(model.link_count()):
                link = model.link_by_index(link_index)
                print("\t\tLink: ", link.name())
                for collision_index in range(link.collision_count()):
                    collision = link.collision_by_index(collision_index)
                    print("\t\t\tCollision: ", collision.name())
                    for visual_index in range(link.collision_count()):
                        visual = link.visual_by_index(visual_index)
                        print("\t\t\tVisual: ", visual.name())
```

Or you can create and add a new element to the SDF DOM, finally we will save it in a new file:

```python
from ignition.math import Vector3d

...

def main():
    input_file = './sphere.sdf'
    root = sdf.Root()
    try:
        root.load(input_file)
    except sdf.SDFErrorsException as e:
        print(e, file=sys.stderr)

    # Create a new model
    world = root.world_by_index(0)
    world.set_name('shapes')

    model = sdf.Model()
    model.set_name('box')

    link = sdf.Link()
    link.set_name('box_link')

    geometry = sdf.Geometry()
    box = sdf.Box()

    box.set_size(Vector3d(1, 1, 1))
    geometry.set_box_shape(box)
    geometry.set_type(sdf.GeometryType.BOX)

    collision = sdf.Collision()
    collision.set_name('collision_box')
    collision.set_geometry(geometry)

    visual = sdf.Visual()
    visual.set_name('visual_box')
    visual.set_geometry(geometry)

    link.add_visual(visual)
    link.add_collision(collision)
    model.add_link(link)
    world.add_model(model)

    # finally we save the result in a new file
    with open('new_sdf.sdf', "w") as f:
      f.write(root.to_string())
```
