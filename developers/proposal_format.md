# Proposal Outline for SDFormat

This format is intended to guide the writing of a proposal for features for the
SDFormat specifciation, and possibly implementation in `libsdformat`.

For examples of proposals, please see:

* [Pose Frame Semantics Proposal](/tutorials?tut=pose_frame_semantics_proposal)
* [Custom elements and attributes](/tutorials?tut=custom_elements_attributes_proposal)

Anything below this horizontal rule is what should be incorporated into the
proposal document. Items in braces or quotes are generally placeholders and
should be replaced or removed.

---

* **Authors**:
Jane Doe `<jane.doe@example>`,
John Doe `<john.doe@example>`
* **Status**: *{Draft|Accepted|Rejected|Final}*
* **SDFormat Version**: *{targeted specification}*
* **`libsdformat` Version**: *{targeted implementation, commit if applicable}*

## Introduction

**Purpose statement (1 sentence)**

* "This proposal suggests *{high-level summary of changes}* in order to
*{goal}*..."
* (optional) "... for *{audience that requested/benefits from the changes}*".

**Background (1-2 sentences)**

"Currently, *{concept being iterated on}* behaves as so..."
* "... which is an issue because..."

**Conclusion (1 sentence)**

"*{changes}* intend to *{improvement}* on *{background}*".

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
