# SDF Quickstart Guide

This guide will walk you through sdformat's [API](http://sdformat.org/api), showing you the basics on how to use it to parse your SDFs. So let's get startd with some code examples! You'll find the API quite regular, making it very easy to pick up.

> **Note:** All code snippets below are for didactic purposes only and thus lack any application specific functionality that would make use of the parser.

## Prerequisites

Make sure you have [sdformat](http://sdformat.org/tutorials?tut=install) installed on you system.

## Parsing a model SDF

Consider the following code snippet. 
```c++
#include <sdf/sdf.hh>
#include <igntion/math/Pose3.hh>

int main(int argc, const char* argv[]) {
	const std::string sdf_path(argv[1]);
	
	sdf::SDFPtr sdf_element(new sdf::SDF());
	sdf::init(sdf_element);
	if (!sdf::readFile(sdf_path, sdf_element)) {
		// handle sdf parsing error
	}
	// keep parsing
	const sdf::ElementPtr root_element = sdf_element->Root();
	if (!root_element->HasElement("model")) {
		// handle non model file error
	}
	sdf::ElementPtr model_element = root->GetElement("model");
	
	sdf::ElementPtr link_element = model_element->GetElement("link");
	while (link_element) {
		const std::string link_name = link_element->Get("name");
		ignition::math::Pose3<double> pose;
		if (link_element->HasElement("pose")) {
			sdf::ElementPtr pose_element = 
				link_element->GetElement("pose");
			pose = pose_element->Get();
		}
		// do something with link name and pose
		
		sdf::ElementPtr collision_element =
			link_element->GetElement("collision");
		while (collision_element) {
			// do something with the collision element
			collision_element = 
				collision_element->GetNextElement("collision");
		}
		link_element = link_element->GetNextElement("link");
	}
	
	sdf::ElementPtr joint_element = model_element->GetElement("joint");
	while (joint_element) {
		const std::string joint_name = joint_element->Get("name");
		// do something with the joint
		joint_element = joint_element->GetNextElement("joint");
	}
	
	// do something with the model
	return 0;
}
```

Let's break it down a bit.

----------------

```c++
sdf::SDFPtr sdf_element(new sdf::SDF());
sdf::init(sdf);
```

This is the main entrypoint to the API: any SDF tree, either in a file  or as a string, will be parsed into this object. On calling `sdf::init()`, XML schemas are loaded, in order to provide format validation later on. It handles all SDF versions!

----------------
  
```c++
if (!sdf::readFile(sdf_path, sdf_element)) {
	// handle model file error
}
// keep parsing
```

Given an `sdf_path` , `sdf::readFile` will attempt to parse such file into our `sdf_element`. If it fails to do so, either because the file could not be accessed or it was not a valid SDF or URDF (yes, sdformat handles both formats seamlessly!), it will return `false` . 

----------------

```c++
const sdf::ElementPtr root_element = sdf->Root();
if (!root_element->HasElement("model")) {
	// handle non model file error
}
```

To traverse the parsed SDF tree, we start at the root. It is important to note that the `root_element` here is **not** the same element as the one given explicitly on the SDF (i.e. the a `<world>` element on a typical world SDF file) but one above. That's why we check for model element existence.  

----------------

Next, you'll notice a pattern for iterating through an element children of the same type (that is, with the same tag).

```c++
sdf::ElementPtr link_element = mode_element->GetElement("link");
while (link_element) {
	// parse link element
	link_element = link_element->GetNextElement("link");
}
```
Take the code above as an example. At the beginning, we get the first link element from the model element.  If it doesn't exist, `model_element->GetElement("link")` will return a `null` pointer, and the loop will be skipped. If it does exist, we'll process that link element in the loop. We then get the next link element, **but this time from the current link element instead of the model element** using `link_element->GetNextElement("link")`. The latter method also returns a `null` pointer if no element could be found, so we'll eventually leave the loop when no more elements of that child remain. To summarize, while `GetElement()` retrieves *child* elements, `GetNextElement()` retrieves *sibling* elements.

With that in mind, you'll find the iterations over links, joints, collisions, etc., quite straightforward and easy to grasp.

----------------------
Let's focus on the link elements' iteration. 

```c++
const std::string link_name = link_element->Get("name");
ignition::math::Pose3d pose = ignition::math::Pose3d::Identity();
if (link_element->HasElement("pose")) {
	sdf::ElementPtr pose_element = 
	link_element->GetElement("pose");
	pose = pose_element->Get();
	// store link pose
}
```

As you might guess, we're getting the `name` attribute of the current `link_element` and then retrieving then its `pose` child element value, defaulting to the identity if none was found.  The `Get()` method is, on this regard, quite polymorphic. It is actually a template method, that returns the given attribute value or the element value itself (that is, the plain text in between tags) if called with no arguments. To this end, it has been specialized to parse such value accordingly into primitive types (i.e. `double`), common std types (i.e. `std::string`) or [`ignition::math`](http://ignitionrobotics.org/libraries/math) types for the most complex ones. 




