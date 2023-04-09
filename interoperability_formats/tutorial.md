# Interoperability with other modeling formats

SDFormat can interoperate with other formats, in particular with [USD](https://graphics.pixar.com/usd/release/index.html)
and [MJCF](https://mujoco.readthedocs.io/en/latest/modeling.html).
We have created command line tools to convert between these formats.
The tools take as input SDFormat files that works in Gazebo Sim and produce as output a MJCF/USD files
that work in Mujoco/NVIDIA Omniverse with approximately equivalent results; and vice versa.

## USD

[gz-usd](https://github.com/gazebosim/gz-usd) provides tools to convert between SDF and USD files.

 - [How to install gz-usd](https://github.com/gazebosim/gz-usd/tree/ahcorde/update/readme#requirements)
 - [Tutorials](https://github.com/gazebosim/gz-usd/blob/ahcorde/update/readme/tutorials/convert_sdf_to_usd.md)

## MJCF

This Python package ([sdformat_mjcf](https://github.com/gazebosim/gz-mujoco/tree/main/sdformat_mjcf)) allows bidirectional
conversion between SDFormat and MJCF to share worlds and robot models.

 - [How to install gz-mujoco](https://github.com/gazebosim/gz-mujoco/tree/main/sdformat_mjcf#install-sdformat-mjcf)
 - [Tools for converting SDFormat to MJCF](https://github.com/gazebosim/gz-mujoco/tree/main/sdformat_mjcf#tools-for-converting-sdformat-to-mjcf)
 - [Tools for converting MJCF to SDFormat](https://github.com/gazebosim/gz-mujoco/tree/main/sdformat_mjcf#tools-for-converting-mjcf-to-sdformat)

**NOTE: MJCF is only available with version 13 and later**.
