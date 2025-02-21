<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:param name="lth" select="0"/>
<xsl:param name="rth" select="10"/>
<xsl:output method="xml" indent="yes"/>
  
  <xsl:template match="launch">
    <xsl:copy>
      <xsl:apply-templates select="include"/>
      <xsl:apply-templates select="test[number(substring-after(@test-name,'t')) &gt;= $lth and number(substring-after(@test-name,'t')) &lt;= $rth]"/>
    </xsl:copy>
  </xsl:template>
  
  <xsl:template match="test">
      <xsl:copy-of select="."/>
  </xsl:template>
 <xsl:template match="include">
    <xsl:copy-of select="."/>
  </xsl:template>
</xsl:stylesheet>

