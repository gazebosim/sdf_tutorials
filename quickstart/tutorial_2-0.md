# Using SDFormat

## Prerequisites

Make sure you have SDFormat installed on you system (follow tutorial-2.0 if in doubt).

## Parsing a model from an SDF file

Let's get right into a code snippet! You'll find sdformat's API quite regular, making it very easy to pick up.

```c++
#include <iostream>
#include <sdf/sdf.hh>

int main(int argc, const char* argv[]) {
	if (argc < 2) {
		std::cerr << "No SDF file path provided" << std::endl; 
		return -1;
	}
	const std::string path_to_sdf(argv[1]);
	
	sdf::SDFPtr sdf(new sdf::SDF());
	sdf::init(sdf);
	if (!sdf::readFile(path_to_sdf, sdf)) {
		// handle sdf file parsing error
	}
	// keep parsing
	const sdf::ElementPtr root_element = sdf->Root();
	if (!root_element->HasElement("model")) {
		// handle non model file error
	}
	sdf::ElementPtr model_element = root->GetElement("model");
	
	sdf::ElementPtr link_element = model_element->GetElement("link");
	while (link_element) {
		std::string link_name = link_element->Get("name");
	
		if (link_element->HasElement("pose")) {
			sdf::ElementPtr pose_element = 
				link_element->GetElement("pose");
			ignition::math::Pose3d pose = pose_element->Get();
			// store link pose
		}
		link_element->GetElement("collision")
		link_element = link_element->GetNextElement("link");
	}
	
	sdf::ElementPtr joint_element = model_element->GetElement("joint");
	while (joint_element) {
		std::string joint_name = joint_element->Get("name");
			
			
		joint = joint->GetNextElement();
	}
	model = model->GetNextElement();
}

```

Ok let's break it down a bit:

```c++
sdf::SDFPtr sdf(new sdf::SDF());
sdf::init(sdf);
```

Here, an `sdf::SDFPtr` object gets initialized. This is the main entrypoint to the API: any SDF description, be it a file or a string, will be parsed into this object. But let's get back to these lines. On calling `sdf::init()`, XML schemas for all SDF versions are loaded, in order to provide format validation later on.

```c++
if (!sdf::readFile(path_to_sdf, sdf)) {
	// handle file error
}
// keep parsing
```
Given an `sdf_file_path` , `sdf::readFile` will attempt to parse such file into our `sdf_element`. If it fails to do so, either because the file could not be accessed or it was not a valid SDF or URDF (yes, sdformat handles both formats seamlessly), it will return `false` . 

```c++
const sdf::ElementPtr root_element = sdf->Root();
if (!root_element->HasElement("model")) {
	// handle non model file error
}
```

To traverse the parsed SDF tree, we start at the root. It is important to note that the `root_element` here is **not** the same element as the one given explicitly on the SDF (i.e. the a `<world>` element on a typical world SDF file) but one above. That's why we check for model element existance.  

Next, you'll notice a pattern for iterating through element childs of the same type (that is, with the same tag).

```c++
sdf::ElementPtr model_element = root_element->GetElement("model");
while (model_element) {
	// parse model element
	model = model->GetNextElement("model");
}
```
At the beginning, we get the first child element of the given type from the parent element.  If it doesn't exist, `GetElement("type")` will return a `null` pointer, and the loop will be skipped. If it does exist, we'll process that child element in the loop. And then we get the next child element, **but this time from the current child element instead of the parent element** using `GetNextElement("type")`. The latter method also returns a `null` pointer if no element could be found, so we'll eventually leave the loop when no more elements of that child remain. To summarize, while `GetElement()` retrieves *child* elements, `GetNextElement()` retrieves *sibling* elements.

With that in mind, you'll find the iterations over links, joints, collisions, etc., quite straightforward and easy to grasp.




