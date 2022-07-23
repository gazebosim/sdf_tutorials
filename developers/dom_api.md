# `libsdformat`'s DOM API

The [Document Object Model
(DOM)](https://en.wikipedia.org/wiki/Document_Object_Model) is programmatic API
for accessing and mutating XML documents. According to this generic definition,
libsdformat's `sdf::Element` (primarily used via `sdf::ElementPtr`, i.e.,
`std::shared_ptr<sdf::Element>`) can be considered a DOM API, but in this article
and in libsdformat in general, the DOM API refers to classes, such as
`sdf::World`, `sdf::Model`, and `sdf::Link` , that provide a higher level
abstraction to the contents of an SDFormat document. For this article, we'll
refer to `sdf::Element` simply as the Element API.


## Background

The Element API is a layer of abstraction that sits on top of the XML parser
(currently `TinyXML2`) and provides schema validation and a simpler API to
manipulate an SDFormat XML document. The parsing functions in libsdformat
populate `sdf::ElementPtr` objects by processing `TinyXML2` objects according
to the schema of each SDFormat tag.

Consider the following SDFormat document 

```xml
<sdf version="1.8">
  <model name="robot">
    <self_collide>true</self_collide>
    <link name="base_link"/>
    <link name="sensor_link"/>
  </model>
</sdf>

```

Using the Element API to we can access various parts of the document.

```c++

// Parse the file
sdf::SDFPtr doc = std::make_shared<sdf::SDF>();
sdf::init(doc);
sdf::Errors errors;
sdf::readFile("path/to/example/file.sdf", doc, errors);

// Get pointer to the root element
sdf::ElementPtr root = doc->Root();

// Access the SDFormat spec version
root->Get<std::string>("version");

// Access the first <model>
auto model = root->FindElement("model");

// Access the <self_collide> tag
model->Get<bool>("self_collide");

// Access the first <link> under <model>
auto firstLink = model->FindElement("link");

// Access the second <link> under <model>
auto secondLink = firstLink->GetNextElement("link")
```


## Motivation for the DOM API

As seen in the example above, using the Element API requires using string
literals when accessing elements. This is inefficient and quite error prone.
For example, to get the `self_collide` tag, if one accidentally misspells the
word like so:

```c++
  model->Get<bool>("self_colide"); // notice the single 'l' in colide
```

the `Get` function looks for `self_colide` and when it can't find it, it will
return a default initialized boolean, which is `false` while the value
specified in the example file is `true`. This is one of the
primary motivations for creating the DOM API where instead one would call
`sdf::Model::SelfCollide` to access the value of the tag. Any misspelling would
be caught by the compiler.

* Accessing tags by name or index is cumbersome.

* Constructing new SDFormat documents using the Element API is cumbersome
  * Example

* Desire to programmatically create valid SDFormat documents without code
duplication.

<!--TODO-->
<!--* Why not use xsd to automatically generate DOM classes-->

## API Design

* Each DOM class is named after a corresponding SDFormat element, e.g.,
`sdf::World` represents `<world>` and `sdf::Model` represents `<model>`.

* When parsing from an SDFormat file or string, each DOM class is responsible
for populating its content from a given `sdf::ElementPtr` object. This is done
in the `Load` member function of the DOM class.

* The typical entry point for an end user is the `sdf::Root` object. While it's
possible to call the `Load` member function of an object that's
not `sdf::Root`, it is not recommended as the functionality of the object will
be limited, (e.g., Frame semantics will not work).

* If a DOM class represents an SDFormat tag that has children, then the
class will provide member function to access the corresponding DOM objects that
represent the children. The access can be by index or by name e.g.,
`sdf::Model::LinkByIndex` and `sdf::Model::LinkByName`

* The DOM class contains the member function `Element()` to access the
`sdf::ElementPtr` object from which it was loaded. This is meant to provide
access to SDFormat tags that are currently not covered by the DOM class
including [custom tags](http://sdformat.org/tutorials?tut=custom_elements_attributes_proposal&cat=pose_semantics_docs&).

* The DOM class contains mutator member functions, however, any mutation of
a DOM object does not update the corresponding entries in the `sdf::ElementPtr`
object contained in the DOM object

* Similarly, mutation of the `sdf::ElementPtr` object of a DOM object does not
affect the contents of the DOM object.

* In general, mutation of the `sdf::ElementPtr` in a DOM object is not
recommended as it is meant to represent the original state of the SDFormat tag
as it was loaded from XML.

### Lifetime relationships

* Parent DOM objects have access to their children, but children do not have
access to their parents. This entails that the lifetime of a parent object must
exceed that of the child. It is invalid to use a pointer to a child object
after the parent object has been destroyed.

* Copying DOM objects makes a shallow copy of the underlying `sdf::ElementPtr`,
thus any mutation that is done to the `sdf::ElementPtr` through one DOM objects
affects the `sdf::ElementPtr` of the copy.

* Each successfully loaded DOM object contains weak pointers to frame graphs
that are shared by all DOM objects loaded from the same SDFormat document. The
owner of the data is `sdf::Root`, thus, it is invalid to use frame semantics
functions after `sdf::Root` object has been destroyed.


### Current state of mutability

* The DOM API has limited functionality for mutating DOM objects. In general,
the API does now allow modifying existing child objects or adding or removing
child objects.

* In libsdformat >= 11.0.0, the frame graph is only built during
`sdf::Root::Load`. Subsequent updates to the names or poses of DOM objects is
not reflected in the frame graph.

* In libsdformat >= 9.0.0 and libsdformat < 11.0.0, the frame graph is built
during `sdf::World::Load` and `sdf::Model::Load` where `sdf::World` and
`sdf::Model` contain their own graphs. Subsequent updates to the
names or poses of DOM objects is not reflected in the frame graph.
