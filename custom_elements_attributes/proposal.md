# Custom elements and attributes

* **Authors**:
Addisu Taddese `<addisu@openrobotics.org>`
* **Status**: Accepted
* **SDFormat Version**: 1.7
* **`libsdformat` Version**: 9.0

## Motivation

SDFormat aims to be a specification for describing simulations without being
closely coupled with specific simulators such as Gazebo. It is also a goal of
this project to provide [libsdformat](https://github.com/osrf/sdformat) as the
canonical library for parsing SDFormat files. As SDFormat gets more widely used
by other applications, it is natural for developers of those applications to
want to extend SDFormat beyond the latest standard to specify parameters for
additional features they create. Since these features and their corresponding
parameters may start out being specific to each application, it may make more
sense to make a custom extension rather than immediately trying to extend the
existing SDFormat specification. Thus, developers are forced to either
implement their own parser instead of using libsdformat or resort to specifying
their custom parameters in an external file and handle cross referencing in
their code. Both of these options are suboptimal. Writing their own parsers is
time consuming and may become a maintenance burden. Trying to maintain
different model files for a single model, e.g. an SDFormat file and an
auxiliary set of custom parameters, may be error prone as the two files must
always be kept in sync, for example, if the name of an element changes.


## Proposal

The proposed solution for the described problem is to give developers the
ability to specify custom XML elements and attributes in SDFormat. This
proposal will describe

  * A convention for specifying custom elements and attributes in SDFormat
  * Support/API in libsdformat for handling and exposing the custom elements
    and attributes
  * Rules for where custom elements and attributes are admissible

### Specifying custom elements and attributes

> **Note**: TinyXML, used by libsdformat to parse SDFormat files, does not have
> native support for XML namespaces. Some of the following requirements and
> conventions are in place due to this limitation.

To avoid name clashes with future versions of SDFormat and to ensure the
interoperability of SDFormat files between various software, custom elements
and attributes are required to be prefixed by an XML [namespace
prefix](https://www.w3.org/TR/xml-names/#NT-NCName). Consequently, elements and
attributes that are not prefixed do not have a namespace and are considered
valid only if they appear in the SDFormat standard.

Namespaces are to be declared using an attribute name that begins with `xmlns:`
(see [W3C Recommendation](https://www.w3.org/TR/xml-names/) for more details).
A Namespace declared using the attribute name `xmlns` (without the colon) is
not permitted due to the aforementioned limitation in libsdformat.

Here's an example of a **namespace prefix**, `foo`, a **namespace name**,
`http://example.org/schema`, a **custom attribute** `foo:type` that specifies
that the world uses 2 dimensional physics and a **custom element**
`<foo:description>` that adds a description of the world that can then be
displayed by the application at runtime.

```
<sdf version="1.6" xmlns:foo="http://example.org/schema">
  <world name="W" foo:type="2d">
    <foo:description>Description of this world</foo:description>
  </world>
</sdf>
```

Attributes of custom elements may be specified without namespace prefixes. This
can be justified since these attributes are properties of the custom elements
and would not clash with attributes used by the standard SDFormat
specification. Requiring namespaces for such attributes would make SDFormat
files too verbose. The following is an example of this usage:

```
<sdf version="1.6" xmlns:foo="http://example.org/schema">
  <world name="W" foo:type="2d">
    <foo:vehicle name="V1" type="4wheel"/>
  </world>
</sdf>
```

In this example the attributes `name` and `type` of the custom element
`foo:vehicle` appear without namespace.

As shown in the examples so far, it is customary to set the value of the
namespace, i.e, the namespace name, to the URI containing the schema of the
namespace. Note, however, that libsdformat treats the provided URI as any other
string literal with no special meaning.

This proposal advocates for a convention where namespaces are declared only in
the root element of an SDFormat file. This would mean only the `<sdf>` tag in
an SDFormat file may have the `xmlns:` attribute.

However, the proposal still allows for namespaces to be declared on any element
of SDFormat. Although not enforced by libsdformat, users should comply with the
namespace [scoping](https://www.w3.org/TR/xml-names/#scoping) and
[uniqueness](https://www.w3.org/TR/xml-names/#uniqAttrs) rules defined in the
[W3C Recommendation](https://www.w3.org/TR/xml-names) when declaring such
namespaces. The scoping rules state:

> The scope of a namespace declaration declaring a prefix extends from the
> beginning of the start-tag in which it appears to the end of the
> corresponding end-tag.

Example:

```
<model name="M1" xmlns:flatland="https://example.org/flatland/schema">
  <flatland:custom_elem>Description of this world</flatland:custom_elem> <!--Valid use of <flatland:custom_elem>-->
  <link name="L" flatland:custom_attr="A" /> <!--Valid use of <flatland:custom_attr>-->
</model>

<model name="M2">
  <flatland:custom_elem>Description of this world</flatland:custom_elem> <!--Invalid use of <flatland:custom_elem>. The flatland prefix is not in scope-->
  <link name="L" flatland:custom_attr="A" /> <!--Invalid use of <flatland:custom_attr>-->
</model>

```

Since default namespaces are not permitted in this proposal, the uniqueness
rules apply only to namespace prefixes. The following is an example that
violates the uniqueness rules

```
<model name="M1" xmlns:flatland="https://example.org/flatland/schema" xmlns:flatland="https://example.org/some_other_schema">
</model>
```

### Support/API in libsdformat

The current behavior of libsdformat is to ignore unknown elements, but to print
a warning for unknown attributes. The proposed behavior is for libsdformat to
stop issuing warnings if it detects an unknown attribute that is prefixed by
a namespace prefix.  Since default namespaces are not allowed, this amounts to
checking if an attribute name contains the `:` character. This relies on the
[conformance requirement](https://www.w3.org/TR/xml-names/#Conformance) of the
W3C recommendation that in a namespace-well-formed document, all element and
attribute names contain either zero or one colons.

The proposed API makes use of the `sdf::ElementPtr` (a pointer to
`sdf::Element`) as the proxy to XML elements. libsdformat provides an
`sdf::ElementPtr` for any of the XML elements that appear in the SDFormat
specification, such as `<world>`, `<model>`, and `<link>`. This can be
accomplished by calling `Element()` on the corresponding classes `sdf::World`,
`sdf::Model`, and `sdf::Link`. The `sdf::ElementPtr` to the root `<sdf>`
element is obtained by calling `sdf::Root::Element()`.

Once an `sdf::ElementPtr` object is available, its attributes can be retrieved
by calling `sdf::Element::GetAttribute()`. To get the value of a custom
attribute, the name of the attribute passed to `GetAttribute` must contain the
namespace prefix of the attribute. For example, if the custom attribute name is
`flatland:custom_attr`, the function call would be
`GetAttribute("flatland:custom_attr")`.

Child elements of `sdf::ElementPtr` are obtained by calling
`sdf::Element::GetElement()`. This function takes the names of the child
element as its first argument and returns an `sdf::ElementPtr`, which can be
further queried for more sibling or child elements using
`sdf::Element::GetNextElement()` or `sdf::Element::GetElement()` respectively.
To get an `sdf::ElementPtr` for a custom element, the name of the element
passed to `GetElement` must contain the namespace prefix of the element. For
example, if the custom element name is `foo:description`, the function call
would be `GetElement("foo:description")`. Note that
`sdf::Element::GetElement()` can modify the underlying `sdf::Element`
by adding a child element with the provided argument if the child element does
not already exist. Thus, it is safer to first check if the child element exists
using `sdf::Element::HasElement()`.

An alternative, but more convenient, function to retrieve the values of
attributes or child elements is `sdf::Elemenet::Get<T>()`. Refer to the API
[documentation](http://sdformat.org/api) for its usage.

#### Examples

Consider the following SDFormat file that utilizes custom elements and
attributes

```
<?xml version="1.0" ?>
<sdf xmlns:flatland="http://example.org/flatland/schema" version="1.6">
  <world name="W" flatland:type="2d">
    <flatland:description>Description of this world</flatland:description>
    <model name="M1">
      <link name="L1" flatland:custom_attr_str="A" flatland:custom_attr_int="5" />
      <link name="L2" />
      <joint name="J1" type="revolute">
        <parent>L1</parent>
        <child>L2</child>
      </joint>

      <flatland:transmission name="simple_trans">
        <flatland:type>transmission_interface/SimpleTransmission</flatland:type>
        <flatland:joint name="J1">
          <flatland:hardware_interface>EffortJointInterface</flatland:hardware_interface>
        </flatland:joint>
      </flatland:transmission>
    </model>
  </world>
</sdf>
```

The code that retrieves the custom elements and attributes is given by

```
#include "sdf/sdf.hh"

...

const std::string SDF_TEST_FILE = /* Fill in with file path */

sdf::Root root;
root.Load(SDF_TEST_FILE);

const sdf::World *world = root.WorldByIndex(0);

// Use of sdf::World::Element() to obtain an sdf::ElementPtr object
sdf::ElementPtr worldElement = world->Element();

// Use of sdf::ElementPtr::GetAttribute()
sdf::ParamPtr typeParam = worldElement->GetAttribute("flatland:type");
std::string simType;
// Use of sdf::ParamPtr::Get<T>()
typeParam->Get<std::string>(simType);

const sdf::Model *model = world->ModelByIndex(0);

const sdf::Link *link1 = model->LinkByIndex(0);
// Use of sdf::Link::Element() to obtain an sdf::ElementPtr object
sdf::ElementPtr link1Element = link1->Element();

// Use of sdf::ElementPtr::Get<T>() to obtain the value of an attribute
auto customAttrStr = link1Element->Get<std::string>("flatland:custom_attr_str");

auto customAttrInt = link1Element->Get<int>("flatland:custom_attr_int");

// Use of sdf::Model::Element() to obtain an sdf::ElementPtr object
sdf::ElementPtr modelElement = model->Element();

sdf::ElementPtr transmission = modelElement->GetElement("flatland:transmission");
auto transmissionName = transmission->Get<std::string>("name");

auto transmissionType = transmission->Get<std::string>("flatland:type");

sdf::ElementPtr tranJointElement = transmission->GetElement("flatland:joint");
auto tranJointName = tranJointElement->Get<std::string>("name");

sdf::ElementPtr transHwInterfaceElement =
    tranJointElement->GetElement("flatland:hardware_interface");

// Use of sdf::ElementPtr::Get<T>() to obtain the value of a child element
auto tranHwInterface =
    tranJointElement->Get<std::string>("flatland:hardware_interface");

...
```


### Rules for where custom elements and attributes can be used

The following are general rules that constrain where and how custom elements
and attributes can be used in an SDFormat document.

1. Custom elements and attributes can only add new information. They cannot be
used to remove or replace information. For example, a user who wants to specify
the dimensions of a `<cylinder>` element using its diameter instead of its
radius, as is the case in the standard SDFormat specification, may not add
a custom `<prefix:diameter>` element to replace `<radius>`. In the case of
`<cylinder>`, `<radius>` is a required element and must be specified.

1. Custom elements may not change the topology or parent-child relationship of
XML elements in an SDFormat document. For example, links are always direct
children of models. It is conceivable that a user might want to wrap a number
of links in a model with a custom element to give them a certain semantics as
in the following snippet. 

    ```
    <model name="M">
      <prefix:kinematic_links>
        <link name="L1"/>
        <link name="L2"/>
      </prefix:kinematic_links>
    </model>
    ```
Since this changes the topology of the XML elements defined in the standard SDFormat
specification, it is not permitted.
