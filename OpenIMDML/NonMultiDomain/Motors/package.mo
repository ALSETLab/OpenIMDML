within OpenIMDML.NonMultiDomain;
package Motors "Single phase and three phase non-multidomain motor models"

  annotation (preferredView = "info", Icon(graphics={
        Rectangle(
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          extent={{-100,-100},{100,100}},
          radius=25.0,
          lineThickness=0.5),
        Rectangle(
          extent={{-56,72},{-76,-48}},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={128,128,128}),
        Rectangle(
          extent={{-56,82},{24,62}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-66,-78},{-56,-78},{-26,-8},{24,-8},{54,-78},{64,-78},{64,
              -88},{-66,-88},{-66,-78}},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-56,72},{64,-48}},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,128,255}),
        Text(
          extent={{-52,46},{56,-22}},
          textColor={102,44,145},
          textStyle={TextStyle.Bold,TextStyle.Italic},
          textString="NMD")}));
end Motors;
