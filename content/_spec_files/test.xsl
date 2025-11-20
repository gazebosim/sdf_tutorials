<xsl:stylesheet version="1.0"
xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
xmlns:dc="http://purl.org/dc/elements/1.1/"
xmlns:dcq="http://purl.org/dc/qualifiers/1.0/"
xmlns:fo="http://www.w3.org/1999/XSL/Format"
xmlns:fn="http://www.w3.org/2005/xpath-functions">

<xsl:output method="html"/>

<xsl:template match="element">
  <li>
    <span class="tree-element">&lt;<xsl:value-of select="@name"/>&gt;</span><i> element</i>

    <div class="row tree-contents">
      <div class="col-xs-2">
        <b>Required: </b> <xsl:value-of select="@required"/> <br/>
        <b>Type: </b> <xsl:value-of select="@type"/>  <br/>
        <b>Default: </b> <xsl:value-of select="@default"/> <br/>
      </div>
      <div class="col-xs-10">
        <b>Description: </b> <xsl:value-of select="description"/>
      </div>
    </div>
    <ul>
      <xsl:apply-templates/>
    </ul>
  </li>
</xsl:template>

<xsl:template match="description">
</xsl:template>

<xsl:template match="attribute">
  <li><span class="tree-attribute"> version </span> <i>Attribute</i>
    <div class="row tree-contents">
      <div class="col-xs-2">
        <b>Required: </b> <xsl:value-of select="@required"/> <br/>
        <b>Type: </b> <xsl:value-of select="@type"/>  <br/>
        <b>Default: </b> <xsl:value-of select="@default"/> <br/>
      </div>
      <div class="col-xs-10">
        <b>Description: </b> <xsl:value-of select="description"/>
      </div>
    </div>
  </li>
</xsl:template>

</xsl:stylesheet>
