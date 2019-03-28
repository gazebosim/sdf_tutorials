# Setting the material properties of visuals in SDFormat

## The `<material>` tag

A `<visual>` tag may contain a `<material>` tag which is used to set the
color and texture properties of a visual.
The full specification for `<material>` can be found
[here](http://sdformat.org/spec?ver=1.4&elem=material).
In this tutorial, we will only look at specifying the color of a visual.
A more detailed tutorial covering image textures and material scripts can be
found [here](http://gazebosim.org/tutorials?tut=color_model).

SDFormat uses the
[Blinn-Phong](https://en.wikipedia.org/wiki/Blinn%E2%80%93Phong_shading_model)
shading model for specifying how materials are to be rendered. This shading
model contains four components that combine to form the final color: ambient,
diffuse, specular, and emissive. The [OpenGL Programming guide chapter on
Lighting](http://www.glprogramming.com/red/chapter05.html) has detailed
information on what this specification means. SDFormat defines the
corresponding tags,
[&lt;ambient&gt;](http://sdformat.org/spec?ver=1.4&elem=material#material_ambient)
, [&lt;diffuse&gt;](http://sdformat.org/spec?ver=1.4&elem=material#material_diffuse)
, [&lt;specular&gt;](http://sdformat.org/spec?ver=1.4&elem=material#material_specular)
, and
[&lt;emissive&gt;](http://sdformat.org/spec?ver=1.4&elem=material#material_emissive)
as child elements of `<material>`. The value for each of these is a vector of
4 numbers representing red, green, blue, and alpha
([RGBA](https://en.wikipedia.org/wiki/RGBA_color_space)).

### Definition of color components

#### Ambient

The Ambient component is the color of an object due to ambient light, i.e, when
no lights are directly pointing at it. It is completely uniform about the
object. Ambient light is meant to approximate light that has been reflected so
many times it is hard to tell where it came from.

#### Diffuse
This is the color of an object under a pure white light. It is calculated using
the light's direction and the surface normal where the light hits. The part of
an object with a normal antiparallel to a light source will be brightest from
this component.

#### Specular

The Specular component is the color and intensity of a highlight from
a [specular reflection](https://en.wikipedia.org/wiki/Specular_reflection).
Higher values make an object appear more shiny. A polished metal surface would
have a very large specular value, while a piece of paper would have almost none.

#### Emissive

The Emissive component creates the appearance that light was emitted from the
object. This emitted light, however, does not add light to other objects in the
world. Like the Ambient component, emissive adds uniform color to an object.

### How to set the color of an object

Specifying the color of an object involves configuring both lights and the
object's material. Thus, a brief explanation of how the color components of
lights can be set is given next.

#### Setting the color components of Lights

Light color can be specified in the world SDF file.
Ambient light is set globally in
[&lt;scene&gt;](http://sdformat.org/spec?ver=1.6&elem=scene#scene_ambient).
The amount of ambient light in the world is a design choice left to the user. An
indoor world may need a large global ambient light since every wall and surface
is an opportunity to reflect light. A simulation of satellites may have almost
no ambient light since most is radiated out into space.

The [&lt;diffuse&gt;](http://sdformat.org/spec?ver=1.4&elem=light#light_diffuse)
and
[&lt;specular&gt;](http://sdformat.org/spec?ver=1.4&elem=light#light_specular)
tags on a [&lt;light&gt;](http://sdformat.org/spec?ver=1.4&elem=light) set the
color of diffuse and specular components emitted.
These tags require four floating point numbers (RGBA) between 0.0 and 1.0.
The last number (alpha) has no affect on lights.

Lights do not have emissive or ambient components.

The following is an example containing `<scene>` and `<light>` elements

```xml
<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="default">
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.25 0.25 0.25 1</background>
    </scene>
    <light type="directional" name="some_light">
      <diffuse>0.7 0.7 0.7 0</diffuse>
      <specular>1 1 1 0</specular>
      <direction>-1 -1 -1</direction>
    </light>
  </world>
</sdf>
```

#### Component values on Lights versus Materials

The final color of an object depends both on the material and the lights shining
on it. Values on lights represent the intensity of light emitted. Values on
a material represent the percentage of light an object reflects. The same
material may be different colors depending on the light that hits it.

For example, consider a material with a diffuse color of
`RGBA(1, 1, 0.5, 1.0)`. Under a white light it would look yellow. If exposed to
a light emitting a diffuse `RGBA(0, 1, 0, 1)` then it would appear green. If
exposed to a light emitting a diffuse `RGBA(1, 0, 0, 1)` then it would appear
red. A light with diffuse `RGBA(0, 0, 0.75, 1)` would make the object appear
dark blue.

[[file:files/light_and_material_interaction.png|256px]]


#### Combination of components

Each of the four components adds color to an object. The final color of an
object is the sum of all components. After summation, if any red, green, or blue
value goes above 1.0 then it is set to 1.0.

[[file:files/component_affects.png|600px]]

The following SDF snippet represents a sphere with colors shown in the image
above.

```xml
<?xml version="1.0"?>
<sdf version="1.4">
  <world name="default">
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <shadows>false</shadows>
    </scene>
    <light type="directional" name="light1">
      <pose>0 0 50 0 0 0</pose>
      <diffuse>0.5 0.5 0.5 1</diffuse>
      <specular>1 1 1 0</specular>
      <direction>1 0 0.5</direction>
    </light>
    <light type="directional" name="light2">
      <pose>0 0 50 0 0 0</pose>
      <diffuse>0.5 0.5 0.5 1</diffuse>
      <specular>1 1 1 0</specular>
      <direction>1 0 -0.5</direction>
    </light>

    <model name="model">
      <link name="link">
        <pose>0 0 0.0 0 0 0</pose>
        <visual name="sphere_vis">
          <geometry>
            <sphere>
              <radius>1</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0.5 0.75 0 1</ambient>
            <diffuse>0.7 0.9 0 1</diffuse>
            <specular>0.2 0.2 0.2 64</specular>
            <emissive>0.1 0 0.1 1</emissive>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>

```

