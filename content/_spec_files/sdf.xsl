<xsl:stylesheet version="1.0"
xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
xmlns:dc="http://purl.org/dc/elements/1.1/"
xmlns:dcq="http://purl.org/dc/qualifiers/1.0/"
xmlns:fo="http://www.w3.org/1999/XSL/Format"
xmlns:fn="http://www.w3.org/2005/xpath-functions">

<xsl:output method="html"/>

<xsl:template match="/element[@name='sdf'] | element[@name='sdf']//*">

  <xsl:if test="@name='sdf'">
  <p>
    <b>Notes:</b> This is the root SDFormat element.
  </p>
  </xsl:if>

  <li>
    <xsl:choose>
      <xsl:when test="*/*">
        <span class='tree-collapse glyphicon glyphicon-plus'></span>
      </xsl:when>

      <xsl:otherwise>
        <span class='tree-collapse glyphicon glyphicon-minus'></span>
      </xsl:otherwise>
    </xsl:choose>

    <a name="{../@name}_{@name}"></a>
    <a href="/spec/__VERSION__/sdf#{../@name}_{@name}">

    <span class="tree-element">
      <h5>&lt;<xsl:value-of select="@name"/>&gt;<small> Element</small></h5>
      <div class="row tree-contents">
        <div class="col-xs-4">
          <b>Required: </b> <xsl:value-of select="@required"/><br/>
          <b>Type: </b> <xsl:value-of select="@type"/><br/>
          <b>Default: </b> <xsl:value-of select="@default"/><br/>
        </div>
        <div class="col-xs-8">
          <b>Description: </b> <xsl:value-of select="description"/>
        </div>
      </div>
    </span>
    </a>

    <xsl:if test="*">
      <ul>
        <xsl:apply-templates/>
      </ul>
    </xsl:if>

  </li>
</xsl:template>

<xsl:template match="element[@name='sdf']/attribute">
  <li>
    <a name="{../@name}_{@name}"></a>
    <a href="/spec/__VERSION__/sdf#{../@name}_{@name}">
    <span class="tree-attribute">
      <h5><xsl:value-of select='@name'/><small> Attribute</small></h5>
      <div class="row tree-contents">
        <!--<div style="width: 200px; display: inline-block">-->
        <div class="col-xs-4">
          <b>Required: </b> <xsl:value-of select="@required"/> <br/>
          <b>Type: </b> <xsl:value-of select="@type"/>  <br/>
          <b>Default: </b> <xsl:value-of select="@default"/> <br/>
        </div>
        <!--<div style="width: 400px; display: inline-block">-->
        <div class="col-xs-8">
          <b>Description: </b> <xsl:value-of select="description"/>
        </div>
      </div>
    </span>
    </a>
  </li>
</xsl:template>

<xsl:template match="/element[@name='sdf']/element[@name='world']">
  <li>
    <a href="/spec/__VERSION__/world" class="other-tab">
      <span class='tree-collapse glyphicon glyphicon-chevron-right'></span>
    </a>
    <a href="/spec/__VERSION__/sdf#{../@name}_{@name}">
    <span class="tree-element">
      <h5>&lt;<xsl:value-of select="@name"/>&gt;<small> Element</small></h5>
      <div class="row tree-contents">
        <div class="col-xs-4">
          <b>Required: </b> <xsl:value-of select="@required"/><br/>
          <b>Type: </b> <xsl:value-of select="@type"/><br/>
          <b>Default: </b> <xsl:value-of select="@default"/><br/>
        </div>
        <div class="col-xs-8">
          <b>Description: </b> <xsl:value-of select="description"/>
        </div>
      </div>
    </span></a>
  </li>
</xsl:template>

<xsl:template match="/element[@name='sdf']/element[@name='model']">
  <li>
    <a href="/spec/__VERSION__/model" class="other-tab">
      <span class='tree-collapse glyphicon glyphicon-chevron-right'></span>
    </a>
    <a href="/spec/__VERSION__/sdf#{../@name}_{@name}">
    <span class="tree-element">
      <h5>&lt;<xsl:value-of select="@name"/>&gt;<small> Element</small></h5>
      <div class="row tree-contents">
        <div class="col-xs-4">
          <b>Required: </b> <xsl:value-of select="@required"/><br/>
          <b>Type: </b> <xsl:value-of select="@type"/><br/>
          <b>Default: </b> <xsl:value-of select="@default"/><br/>
        </div>
        <div class="col-xs-8">
          <b>Description: </b> <xsl:value-of select="description"/>
        </div>
      </div>
    </span></a>
  </li>
</xsl:template>

<xsl:template match="/element[@name='sdf']/element[@name='actor']">
  <li>
    <a href="/spec/__VERSION__/actor" class="other-tab">
      <span class='tree-collapse glyphicon glyphicon-chevron-right'></span>
    </a>
    <a href="/spec/__VERSION__/sdf#{../@name}_{@name}">
    <span class="tree-element">
      <h5>&lt;<xsl:value-of select="@name"/>&gt;<small> Element</small></h5>
      <div class="row tree-contents">
        <div class="col-xs-4">
          <b>Required: </b> <xsl:value-of select="@required"/><br/>
          <b>Type: </b> <xsl:value-of select="@type"/><br/>
          <b>Default: </b> <xsl:value-of select="@default"/><br/>
        </div>
        <div class="col-xs-8">
          <b>Description: </b> <xsl:value-of select="description"/>
        </div>
      </div>
    </span></a>
  </li>
</xsl:template>

<xsl:template match="/element[@name='sdf']/element[@name='light']">
  <li>
    <a href="/spec/__VERSION__/light" class="other-tab">
      <span class='tree-collapse glyphicon glyphicon-chevron-right'></span>
    </a>
    <a href="/spec/__VERSION__/sdf#{../@name}_{@name}">
    <span class="tree-element">
      <h5>&lt;<xsl:value-of select="@name"/>&gt;<small> Element</small></h5>
      <div class="row tree-contents">
        <div class="col-xs-4">
          <b>Required: </b> <xsl:value-of select="@required"/><br/>
          <b>Type: </b> <xsl:value-of select="@type"/><br/>
          <b>Default: </b> <xsl:value-of select="@default"/><br/>
        </div>
        <div class="col-xs-8">
          <b>Description: </b> <xsl:value-of select="description"/>
        </div>
      </div>
    </span></a>
  </li>
</xsl:template>


<xsl:template match="//description">
</xsl:template>

</xsl:stylesheet>
