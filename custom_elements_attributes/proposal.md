# Custom elements and attributes

## Motivation

SDFormat aims to be a specification for describing simulations without being
closely coupled with specific simulators such as Gazebo. It is also a goal of
this project to provide [libsdformat](https://bitbucket/osrf/sdformat) as the
canonical library for parsing SDFormat files. As SDFormat gets more widely used
by other applications, it is natural for developers of those applications to
want to extend SDFormat beyond the latest standard to specify parameters for
additional features they create. Since these features and their corresponding
parameteres may start out being specific to each application, it may make more
sense to make a custom extension rather than immediately tryng to extend the
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

> **Note**: TinyXML, used by libsdformat to parse SDFormat files, does have native
> support XML namespaces. Some of the following requirements and conventions
> are in place due to this limitation.

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
<sdf xmlns:foo="http://example.org/schema">
  <world name="W" foo:type="2d">
    <foo:description>Description of this world</foo:description>
  </world>
</sdf>
```

Attributes of custom elements may be specified without namespace prefixes. This
can be justfied since these attributes are properties of the custom elements
and would not clash with attributes used by the standard SDFormat
specification. Requiring namespaces for such attributes would make SDFormat
files too verbose. The following is an example of this usage:

```
<sdf xmlns:foo="http://example.org/schema">
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

Namespaces may be declared on any element of SDFormat.
Although not enforced by libsdformat, users should comply with the namespace
[scoping](https://www.w3.org/TR/xml-names/#scoping) and
[uniqueness](https://www.w3.org/TR/xml-names/#uniqAttrs) rules defined in the
[W3C Recommendation](https://www.w3.org/TR/xml-names) when declaring namespaces.
The scoping rules state:

> The scope of a namespace declaration declaring a prefix extends from the
> beginning of the start-tag in which it appears to the end of the
> corresponding end-tag.

Example:

```
<model name="M1" xmlns:mysim="https://example.org/mysim/schema">
  <mysim:custom_elem>Description of this world</mysim:custom_elem> <!--Valid use of <mysim:custom_elem>-->
  <link name="L" mysim:custom_attr="A" /> <!--Valid use of <mysim:custom_attr>-->
</model>

<model name="M2">
  <mysim:custom_elem>Description of this world</mysim:custom_elem> <!--Invalid use of <mysim:custom_elem>. The mysim prefix is not in scope-->
  <link name="L" mysim:custom_attr="A" /> <!--Invalid use of <mysim:custom_attr>-->
</model>

```

This proposal advocates for a convention where namespaces are declared only in
the root element of an SDFormat file. This means only the `<sdf>` tag in an
SDFormat world file or the topmost `<model>` tag in an SDFormat model file may
have the `xmlns:` attribute.

Since default namespaces are not permitted in this proposal, the uniqueness
rules apply only to namespace prefixes. The following is an example that
violates the uniqueness rules

```
<model name="M1" xmlns:mysim="https://example.org/mysim/schema" xmlns:mysim="https://example.org/some_other_schema">
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
attribute names contain either zero or one colon.

The proposed API makes use of the `sdf::ElementPtr` (a pointer to
`sdf::Element`) as the proxy to XML elements. libsdformat provides an
`sdf::ElementPtr` for any of the XML elements that appear in the SDFormat
specification, such as `<world>`, `<model>`, and `<link>`. This can be
accomplished by calling `Element()` on the corresponding classes `sdf::World`,
`sdf::Model`, and `sdf::Link`. The `sdf::ElementPtr` to the root `<sdf>`
element is obtained by calling `sdf::Root::Element()`.

Once an `sdf::ElementPtr` object is available, its attributes can be retrieved
by calling `sdf::Element::GetAttribute()`. This function takes the attribute
name as its first argument and returns an `sdf::ParamPtr` (a pointer to
`sdf::Param`). libsdformat provides a function template `sdf::Param::Get<T>`
that can be used to obtain the value of the attribute converted to the passed
in type `T`. To get the value of a custom attribute, the name of the attribute
passed to `GetAttribute` must contain the namespace prefix of the attribute.
For example, if the custom attribute name is `mysim:custom_attr`, the function
call would be `GetAttribute("mysim:custom_attr")`.

Child elements of `sdf::ElementPtr` are obtained by calling
`sdf::Element::GetElement()`. This function takes the names of the child
element as its first argument and returns an `sdf::ElementPtr`, which can be
further querried for more sibling or child elements using
`sdf::Element::GetNextElement()` or `sdf::Element::GetElement()` respectively.
To get an `sdf::ElementPtr` for a custom element, the name of the element
passed to `GetElement` must contain the namespace prefix of the element.
For example, if the custom element name is `foo:description`, the function call
would be `GetElement("foo:description")`.

#### Examples
TODO

### Rules for where custom elements and attributes can be used
TODO
