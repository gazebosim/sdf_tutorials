# Roadmap

All releases mentioned here are relatively recent.

## `libsdformat` Releases

For more information, see [Changelog](https://github.com/osrf/sdformat/blob/master/Changelog.md).

The following list provides a high-level summary of the specification and C++
version supported by the given release of the library:

* `libsdformat4`: For use in Gazebo 7.
    * 4.4.0 (**released**): C++11, SDFormat 1.6.
* `libsdformat6`: For use in Gazebo 9, 10.
    * 6.2.0 (**released**): C++11, SDFormat 1.6.
* `libsdformat7`:
    * 7.0.0 (*not released*): C++14, SDFormat 1.7. *This was an intermediate
    version that will not be released.*
* `libsdformat8`: For use in Ignition Blueprint.
    * 8.5.0 (**released**): C++17, SDFormat 1.6.
* `libsdformat9`: For use in Gazebo 11, Ignition Citadel.
    * 9.0.0 (**released**): C++17, SDFormat 1.7.
* `libsdformat10`: For use in Ignition Dome.
    * 10.0.0 (**released**): C++17, SDFormat 1.7.
* `libsdformat11`: For use in Ignition Edifice.
    * 11.0.0 (**released**): C++17, SDFormat 1.8.

## Downstream Library Support

In order to promote consistent support of the SDFormat specification, we list
recent releases of some downstream projects and their support of the
specification (as well as the caveats):

* [Classic Gazebo](http://gazebosim.org/#status)
    * 7 (**released**): libsdformat4, SDFormat 1.6
    * 9 (**released**): libsdformat6, SDFormat 1.6
    * 10 (**released**): libsdformat6, SDFormat 1.6
    * 11 (**released**): libsdformat9, SDFormat 1.7
* [Ignition](https://ignitionrobotics.org)
    * Blueprint (**released**): libsdformat8, SDFormat 1.6, but supports only a subset:
        * Does not support directly nested models
    * Citadel (**released**): libsdformat9, SDFormat 1.7, but only a subset:
        * Does not support directly nested models
    * Dome (**released**): libsdformat10, SDFormat 1.7, but only a subset:
        * Does not support directly nested models
    * Edifice (**released**): libsdformat11, SDFormat 1.8
* [Drake](https://github.com/RobotLocomotion/drake/releases):
    * 0.10.0 - 0.13.0 (**released**): SDFormat 1.6, but deviates:
        * Does not support directly nested models
        * Does not support specifying poses of free-floating bodies (as initial
        conditions).
        * Adds extra semantics to `//pose/frame` and `//frame` elements.
    * 0.14.0 (**released**): SDFormat 1.7.

If you would like your project listed here, please [make a pull request](https://github.com/osrf/sdf_tutorials/pulls).
