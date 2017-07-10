# SDF Quickstart Guide

This guide will walk you through sdformat's [API](http://sdformat.org/api), showing you the basics on how to use it to parse your SDFs. You'll find the API quite regular, making it very easy to pick up.

## Prerequisites

Make sure you have [sdformat](http://sdformat.org/tutorials?tut=install) installed on your system.

## Parsing a model SDF

### Developing the code

Consider the code below:

```c++
#include <iostream>

#include <sdf/sdf.hh>

int main(int argc, const char* argv[]) {
  // check arguments
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <sdf-path>" << std::endl;
    return -1;
  }
  const std::string sdfPath(argv[1]);

  // load and check sdf file
  sdf::SDFPtr sdfElement(new sdf::SDF());
  sdf::init(sdfElement);
  if (!sdf::readFile(sdfPath, sdfElement)) {
    std::cerr << sdfPath << " is not a valid SDF file!" << std::endl;
    return -2;
  }

  // start parsing model
  const sdf::ElementPtr rootElement = sdfElement->Root();
  if (!rootElement->HasElement("model")) {
    std::cerr << sdfPath << " is not a model SDF file!" << std::endl;
    return -3;
  }
  const sdf::ElementPtr modelElement = rootElement->GetElement("model");
  const std::string modelName = modelElement->Get<std::string>("name");
  std::cout << "Found " << modelName << " model!" << std::endl;

  // parse model links
  sdf::ElementPtr linkElement = modelElement->GetElement("link");
  while (linkElement) {
    const std::string linkName = linkElement->Get<std::string>("name");
    std::cout << "Found " << linkName << " link in "
              << modelName << " model!" << std::endl;
    linkElement = linkElement->GetNextElement("link");
  }

  // parse model joints
  sdf::ElementPtr jointElement = modelElement->GetElement("joint");
  while (jointElement) {
    const std::string jointName = jointElement->Get<std::string>("name");
    std::cout << "Found " << jointName << " joint in "
              << modelName << " model!" << std::endl;

    const sdf::ElementPtr parentElement = jointElement->GetElement("parent");
    const std::string parentLinkName = parentElement->Get<std::string>();

    const sdf::ElementPtr childElement = jointElement->GetElement("child");
    const std::string childLinkName = childElement->Get<std::string>();

    std::cout << "Joint " << jointName << " connects " << parentLinkName
              << " link to " << childLinkName << " link" << std::endl;

    jointElement = jointElement->GetNextElement("joint");
  }

  return 0;
}
```

Let's break it down a bit.

----------------

```c++
sdf::SDFPtr sdfElement(new sdf::SDF());
sdf::init(sdfElement);
```

This is the main entrypoint to the API: any SDF tree, either in a file  or as a string, will be parsed into this object. On calling `sdf::init()`, the XML schemas for all SDF versions are loaded in order to provide format validation later on.

----------------

```c++
if (!sdf::readFile(sdfPath, sdfElement)) {
  std::cerr << sdfPath << " is not a valid SDF file!" << std::endl;
  return -2;
}
```

Given an `sdf_path` , `sdf::readFile` will attempt to parse such file into our `sdf_element`. If it fails to do so, either because the file could not be accessed or it was not a valid SDF or URDF (yes, sdformat handles both formats seamlessly!), it will return `false` . 

----------------

```c++
const sdf::ElementPtr rootElement = sdfElement->Root();
if (!rootElement->HasElement("model")) {
  std::cerr << sdfPath << " is not a model SDF file!" << std::endl;
  return -3;
}
```

To traverse the parsed SDF tree, we start at the root. It is important to note that the `root_element` here is **not** the same element as the one given explicitly on the SDF (i.e. the a `<world>` element on a typical world SDF file) but one above. That's why we check for model element existence.  

----------------

Next, you'll notice a pattern for iterating through an element children of the same type (that is, with the same tag):

```c++
sdf::ElementPtr linkElement = modelElement->GetElement("link");
while (linkElement) {
    // parse link element
    linkElement = linkElement->GetNextElement("link");
}
```

At the beginning, we get the first link element from the model element.  If it doesn't exist, `model_element->GetElement("link")` will return a `null` pointer, and the loop will be skipped. If it does exist, we'll process that link element in the loop. We then get the next link element, but this time from the current link element instead of the model element using `link_element->GetNextElement("link")`. The latter method also returns a `null` pointer if no element could be found, so we'll eventually leave the loop when no more elements of that child remain.

To summarize, while `GetElement()` retrieves *child* elements, `GetNextElement()` retrieves *sibling* elements. With that in mind, you'll find the iterations over links, joints, collisions, etc., quite straightforward and easy to grasp.

----------------------
Let's now focus on the joints elements' iteration:

```c++
const std::string jointName = jointElement->Get<std::string>("name");
std::cout << "Found " << jointName << " joint in "
          << modelName << " model!" << std::endl;

const sdf::ElementPtr parentElement = jointElement->GetElement("parent");
const std::string parentLinkName = parentElement->Get<std::string>();

const sdf::ElementPtr childElement = jointElement->GetElement("child");
const std::string childLinkName = childElement->Get<std::string>();

std::cout << "Joint " << jointName << " connects " << parentLinkName
          << " link to " << childLinkName << " link" << std::endl;

```

As you might guess, we're getting the `name` attribute of the current `joint_element` and then retrieving its `parent` and `child` elements' values. It is interesting to note all `Get()` method's use cases: `Get()` is a template method that returns the given attribute value or the element value itself (that is, the plain text in between tags) if called with no arguments. To this end, it has been specialized to parse such value accordingly into primitive types (i.e. `double`), common std types (i.e. `std::string`) or [`ignition::math`](http://ignitionrobotics.org/libraries/math) types for the most complex ones (like poses).

### Building the code

Create an empty directory:

```sh
mkdir sdf_tutorial
cd sdf_tutorial
```

Copy the code into a file and name it `check_sdf.cc`. Along with it, add a `CMakeLists.txt` file and copy the following into it:

```
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(SDFormat REQUIRED)

include_directories(${SDFormat_INCLUDE_DIRS})
link_directories(${SDFormat_LIBRARY_DIRS})

add_executable(check_sdf check_sdf.cc)
target_link_libraries(check_sdf ${SDFormat_LIBRARIES})
```

Now build it:

```sh
mkdir build
cd build
cmake ..
make
```

You can find plenty of SDF samples to play with [here](http://models.gazebosim.org/). Just fetch the `model.sdf` of your model of choice and give it a more meaningful name. For example, running `./build/check_sdf` on the `husky.sdf` file (`husky/model.sdf` on the site) will generate the following output:

```
Found husky model!
Found base_link link in husky model!
Found back_left_wheel link in husky model!
Found back_right_wheel link in husky model!
Found front_left_wheel link in husky model!
Found front_right_wheel link in husky model!
Found back_left_joint joint in husky model!
Joint back_left_joint connects base_link link to back_left_wheel link
Found back_right_joint joint in husky model!
Joint back_right_joint connects base_link link to back_right_wheel link
Found front_left_joint joint in husky model!
Joint front_left_joint connects base_link link to front_left_wheel link
Found front_right_joint joint in husky model!
Joint front_right_joint connects base_link link to front_right_wheel link
```
