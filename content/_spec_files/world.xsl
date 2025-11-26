<xsl:stylesheet version="1.0"
xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"
xmlns:dc="http://purl.org/dc/elements/1.1/"
xmlns:dcq="http://purl.org/dc/qualifiers/1.0/"
xmlns:fo="http://www.w3.org/1999/XSL/Format"
xmlns:fn="http://www.w3.org/2005/xpath-functions">

<xsl:output method="html"/>

<xsl:template match="/element[@name='sdf']/element[@name='world']">
  <p>
    <b>Parent element: </b>&lt;sdf&gt;<br/>
    <b>Notes:</b> Multiple worlds can exist in a single SDFormat file.
  </p>

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
    <a href="/spec/__VERSION__/world#{../@name}_{@name}">

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

<xsl:template match="element[@name='sdf']/element[@name='world']//attribute">
  <li>
    <a name="{../@name}_{@name}"></a>
    <a href="/spec/__VERSION__/world#{../@name}_{@name}">
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

<xsl:template match="/element[@name='sdf']/element[@name='world']/element[@name!='physics' and @name!='scene' and @name!='model' and @name!='joint' and @name!='link'] | /element[@name='sdf']/element[@name='world']/element[@name!='physics' and @name!='scene' and @name!='model' and @name!='joint' and @name!='link']//*">
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
    <a href="/spec/__VERSION__/world#{../@name}_{@name}">

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

<xsl:template match="/element[@name='sdf']/element[@name='world']/element[@name='actor']">
  <li>
    <a href="/spec/__VERSION__/actor" class="other-tab">
      <span class='tree-collapse glyphicon glyphicon-chevron-right'></span>
    </a>
    <a href="/spec/__VERSION__/world#{../@name}_{@name}">
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

<xsl:template match="/element[@name='sdf']/element[@name='world']/element[@name='light']">
  <li>
    <a href="/spec/__VERSION__/light" class="other-tab">
      <span class='tree-collapse glyphicon glyphicon-chevron-right'></span>
    </a>

    <a href="/spec/__VERSION__/world#{../@name}_{@name}">
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

<xsl:template match="/element[@name='sdf']/element[@name='world']/element[@name='model'] | /element[@name='sdf']/element[@name='world']/element[@name='population']/element[@name='model']">
  <li>
    <a href="/spec/__VERSION__/model" class="other-tab">
      <span class='tree-collapse glyphicon glyphicon-chevron-right'></span>
    </a>

    <a href="/spec/__VERSION__/world#{../@name}_{@name}">
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

<xsl:template match="/element[@name='sdf']/element[@name='world']/element[@name='state']">
  <li>
    <a href="/spec/__VERSION__/state" class="other-tab">
      <span class='tree-collapse glyphicon glyphicon-chevron-right'></span>
    </a>
    <a href="/spec/__VERSION__/world#{../@name}_{@name}">
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

<xsl:template match="/element[@name='sdf']/element[@name='world']/element[@name='physics']">
  <li>
    <a href="/spec/__VERSION__/physics" class="other-tab">
      <span class='tree-collapse glyphicon glyphicon-chevron-right'></span>
    </a>
    <a href="/spec/__VERSION__/world#{../@name}_{@name}">
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

<xsl:template match="/element[@name='sdf']/element[@name='world']/element[@name='scene']">
  <li>
    <a href="/spec/__VERSION__/scene" class="other-tab">
      <span class='tree-collapse glyphicon glyphicon-chevron-right'></span>
    </a>
    <a href="/spec/__VERSION__/world#{../@name}_{@name}">
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

<xsl:template match="/element[@name='sdf']/element[@name='world']/element[@name='road']/element[@name='material']">
  <li>
    <a href="/spec/__VERSION__/material" class="other-tab">
      <span class='tree-collapse glyphicon glyphicon-chevron-right'></span>
    </a>
    <a href="/spec/__VERSION__/world#{../@name}_{@name}">
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

<xsl:template match="/element[@name='sdf']/element[@name='world']/element[@name='joint']">
  <li>
    <a href="/spec/__VERSION__/joint" class="other-tab">
      <span class='tree-collapse glyphicon glyphicon-chevron-right'></span>
    </a>
    <a href="/spec/__VERSION__/world#{../@name}_{@name}">
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
