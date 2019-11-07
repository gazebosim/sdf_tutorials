# Roadmap

All releases mentioned here are relatively recent.

## `libsdformat` Releases

For more information, see [Changelog](https://bitbucket.org/osrf/sdformat/src/default/Changelog.md).

The following list provides a high-level summary of the specification and C++
version supported by the given release of the library:

* `libsdformat6`:
    * 6.2.0 (released): C++11, SDFormat 1.6.
* `libsdformat7`:
    * 7.0.0 (**not released**): C++14, SDFormat 1.7. *This was an intermediate
    version that has no need to be released.*
* `libsdformat8`:
    * 8.3.0 (released): C++17, SDFormat 1.6.
    * 8.4.0 (**not released**): ?
* `libsdformat9`: For use in Ignition Gazebo.
    * 9.0.0 (**target: Dec. 2019**): C++14, SDFormat 1.7.

## Downstream Library Support

In order to promote consistent support of the SDFormat specification, we list
recent releases of some downstream projects and their support of the
specification (as well as the caveats):

* [Classic Gazebo](http://gazebosim.org/#status)
    * 10 (**released**): SDFormat 1.6
    * 11 (**target: Jan. 2020**): SDFormat 1.7
* [ignition Gazebo](https://ignitionrobotics.org/libs/gazebo)
    * 2.10.0 (**released**): SDFormat 1.6, but deviates:
        * Does not support directly nested models
* [Drake](https://github.com/RobotLocomotion/drake/releases):
    * 0.10.0 - 0.11.0 (**released**): SDFormat 1.6, but deviates:
        * Does not support directly nested models
        * Does not support specifying poses of free-floating bodies (as initial
        conditions).
        * Adds extra semantics to `//pose/frame` and `//frame` elements.
    * 0.1x.0 (**target: Dec. 2019**): SDFormat 1.7.

If you would like your project listed here, please [make a pull request](https://bitbucket.org/osrf/sdf_tutorials/pull-requests/).
