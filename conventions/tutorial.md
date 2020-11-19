# Conventions

## Syntax

All tutorials use [XPath syntax](https://www.w3schools.com/xml/xpath_syntax.asp)
to describe elements and attributes concisely.

SDFormat example:

```xml
<model name="model_name">
  <pose>0 0 0.5 0 0 0</pose>
  <link name="link">
  </link>
  <joint type="revolute" name="my_joint">
  </joint>
</model>
```

SDFormat nested tags and attributes can be represented in XPath syntax. For example,
the `<model>` tag above can be referred to as `//model`; the `name` attribute of the
`<model>` tag are referenced as `//model/@name`; the `<link>` and `<joint>`entities
within `<model>` can be referred to as `//model/link` and `//model/joint`.
