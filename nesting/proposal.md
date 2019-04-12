# WIP Model Nesting: Proposal

At present, SDFormat adds an `<include/>` tag. However, it has the following
design issues:

*   There is no specification on how encapsulated an included file should be,
and at what stage of construction inclusion is permitted.
    * Can the included file refer to joints that it does not define?
        * Example: Adding a gripper to an IIWA. There may be a weld pose based
        on the pneumatic or electric flange.
*   There is no explicit specification about namespacing, relative references
or absolute references, etc.
    * This is being addressed in the [Pose Frame Semantics Proposal PR](https://bitbucket.org/osrf/sdf_tutorials/pull-requests/7/pose-frame-semantics-proposal-for-new/diff)
*   There is not enough cowbell. WE NEED MORE COWBELL!

TODO(eric): Add more when there is sufficient cowbell.
