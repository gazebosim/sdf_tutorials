# Convert SDF to USD

## Prerequisites

* sdformat with USD support (see the [sdformat installation instructions](/tutorials?tut=installD))
* Omniverse Create

### Installing Omniverse Create

Download Omniverse from https://www.nvidia.com/en-us/omniverse/.

After logging in to the launcher, go to the "Exchange" tab and search for "Create". After installing it, you can launch it from the "Library" tab.

### Convert SDF to USD

If sdformat is built with USD support, there should be a `sdf2usd` cli program.

```
SDF to USD converter
Usage: sdf2usd [OPTIONS] [input] [output]

Positionals:
  input TEXT                  Input filename. Defaults to input.sdf unless otherwise specified.
  output TEXT                 Output filename. Defaults to output.usd unless otherwise specified.

Options:
  -h,--help                   Print this help message and exit
  --help-all                  Show all help
  --version                   
```

Example of converting sdf to usd

```bash
sdf2usd /usr/share/ignition/ignition-gazebo6/worlds/shapes.sdf shapes.usda
```

*Note: The format of the output depends on the extension, use `.usd` to create a USD binary file instead.*

Test the result by opening it in Omniverse Create.

<!--TODO: Bug in sdf2usd? Some shapes are clipping through the plane-->
[[file:files/omni-create.png|720px]]

