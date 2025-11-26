<xsl:stylesheet version="1.0"
xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
xmlns:dc="http://purl.org/dc/elements/1.1/"
xmlns:dcq="http://purl.org/dc/qualifiers/1.0/"
xmlns:fo="http://www.w3.org/1999/XSL/Format"
xmlns:fn="http://www.w3.org/2005/xpath-functions">

<xsl:output method="html"/>

<xsl:template match="/element[@name='sdf']/element[@name='world']/element[@name='model']/element[@name='link'] | /element[@name='sdf']/element[@name='world']/element[@name='model']/element[@name='link']//*">
  <xsl:if test="@name='link'">
  <p>
    <b>Parent element: </b>&lt;model&gt;<br/>
    <b>Notes:</b> A model may contain many links.
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
    <a href="/spec/__VERSION__/link#{../@name}_{@name}">

      <span class="tree-element">
        <xsl:choose>
          <xsl:when test="not(@copy_data)">
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
          </xsl:when>
          <xsl:otherwise>
            <h5>Elements</h5>
            <div class="row tree-contents">
              <div class="col-xs-8">
                <b>Description: </b> Arbitrary elements and attributes that can be used to configure the plugin
              </div>
            </div>
          </xsl:otherwise>
        </xsl:choose>
      </span>
    </a>

    <xsl:if test="*">
      <ul>
        <xsl:apply-templates/>
      </ul>
    </xsl:if>

  </li>
</xsl:template>


<xsl:template match="/element[@name='sdf']/element[@name='world']/element[@name='model']/element[@name='link']//attribute">
  <li>
    <a name="{../@name}_{@name}"></a>
    <a href="/spec/__VERSION__/link#{../@name}_{@name}">
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

    <xsl:if test="*">
      <ul>
        <xsl:apply-templates/>
      </ul>
    </xsl:if>

  </li>
</xsl:template>


<xsl:template match="/element[@name='sdf']/element[@name='world']/element[@name='model']/element[@name='link']/element[@name='light']">
  <li>
    <a href="/spec/__VERSION__/light" class="other-tab">
      <span class='tree-collapse glyphicon glyphicon-chevron-right'></span>
    </a>

    <a href="/spec/__VERSION__/link#{../@name}_{@name}">
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

<xsl:template match="/element[@name='sdf']/element[@name='world']/element[@name='model']/element[@name='link']/element[@name='collision']">
  <li>
    <a href="/spec/__VERSION__/collision" class="other-tab">
      <span class='tree-collapse glyphicon glyphicon-chevron-right'></span>
    </a>
    <a href="/spec/__VERSION__/link#{../@name}_{@name}">
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

<xsl:template match="/element[@name='sdf']/element[@name='world']/element[@name='model']/element[@name='link']/element[@name='visual']">
  <li>
    <a href="/spec/__VERSION__/visual" class="other-tab">
      <span class='tree-collapse glyphicon glyphicon-chevron-right'></span>
    </a>
    <a href="/spec/__VERSION__/link#{../@name}_{@name}">
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

<xsl:template match="/element[@name='sdf']/element[@name='world']/element[@name='model']/element[@name='link']/element[@name='sensor']">
  <li>
    <a href="/spec/__VERSION__/sensor" class="other-tab">
      <span class='tree-collapse glyphicon glyphicon-chevron-right'></span>
    </a>
    <a href="/spec/__VERSION__/link#{../@name}_{@name}">
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
