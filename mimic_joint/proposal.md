# Proposal Procedure and Format

* **Authors**:
Steve Peters`<scpeters@openrobotics.org>`,
Aditya Pande `<aditya.pande@osrfoundation.org>`
* **Status**: *Draft*
* **SDFormat Version**: *1.8+*
* **`libsdformat` Version**: *11*

## Introduction

This proposal suggests adding a new joint type called the Mimic joint
that adds a linear equality constraint between the output position of
two joints.

Currently, the Gearbox joint adds a similar constraint on the rotation
of `//parent` and `//child` links relative to a reference link
`//gearbox_reference_body`, with the rotation axes specified by
`//axis/xyz` for `//parent` and `//axis2/xyz` for `//child`.
The Gearbox joint is typically used in conjunction with a pair of
revolute joints with identical axes but requires duplication of the
axis definitions.
Another drawback of the Gearbox joint is that  it only constrains rotational
motion, so it cannot model constraints involving translational motion,
such as a rack and pinion mechanism.

The Mimic joint will simplify the definition of this constraint by
specifying joints instead of links in `//parent` and `//child` so
that the joint axis information does not need to be duplicated.
The Mimic joint will be more flexible than the Gearbox joint by
allowing constraints on the output of prismatic joints and other
joints with translational outputs.

## Document summary

Make a bullet list of all major sections:

* "*{section}*: *{single-sentence summary of content in that section}*".

## Syntax (optional)

* Define any possibly-unclear syntax used in the document.

## Motivation

* Elaborate on "Background" statement from "Introduction".
* Elaborate on "Conclusion" statement from "Introduction".
  * "*{changes covered in proposal}* will improve *{concept being iterated on}* by *{key improvements}*".
* Explain why *{changes}* are beneficial/chosen to solve the issues of *{concept}*.
* If you have discussions or material that can easily be cross-referenced and
  accessed, be sure to include them here.

## Proposed changes

Introduce the section before listing changes

### *{number}* *{Proposed change}*

**Change**

"*{concept}* must *{behavior}*".

**Details**

* "This is achieved by..." (or simply list details without standard language).
* Use bullet lists for long lists of like-details.
* Code snippets/examples belong here.

**Previous behavior**

* "Previously..." or "In *{previous version}*..."
* Referring to current behavior, but written in past tense (POV of the
proposal).

**Justification**

"*{change}* is necessary because..."

For proposals with many "Proposed changes":

* Number each group of like changes under a descriptive `###` subheading.
* Individual changes under `####` subheadings.
* "Alternatives considered" for each individual change under `#####` subheading.
* If "Previous behavior" and/or "Justification" sections are shared by all the
individual changes under one `###` heading, those parts can go directly under
the `###` heading instead of under each `####` heading.

## Examples

* Introduce the section before listing examples.

## Appendix
