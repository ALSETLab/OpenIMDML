within ;
package OpenIMDML "Open Instance Multi-Domain Machine
Library using Modelica"

annotation (uses(
    Modelica(version="4.0.0"),
    OpenIPSL(version="2.0.0-beta.1"),
    Complex(version="4.0.0"),
    Modelon(version="4.0"),
    DymolaModels(version="1.5.0"),
    Cooling(version="1.4.4")), Icon(graphics={
      Rectangle(
        extent={{-100,100},{100,-100}},
        lineColor={238,46,47},
        fillPattern=FillPattern.Solid,
        radius=25,
        fillColor={215,215,215},
        lineThickness=1),
        Polygon(
          origin={-2.179,39},
          fillPattern=FillPattern.Solid,
          points={{-70,-90},{-60,-90},{-30,-20},{20,-20},{50,-90},{60,-90},{
              60,-100},{-70,-100},{-70,-90}},
        lineColor={95,95,95},
        pattern=LinePattern.None),
        Rectangle(
          origin={-6,20.817},
          fillColor={238,46,47},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-54.179,-59.817},{65.821,60.183}},
        lineColor={255,0,0}),
        Rectangle(
          origin={-0.179,21},
          fillColor={128,128,128},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-80,-60},{-60,60}}),
        Rectangle(
          origin={-0.179,21},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid,
          extent={{-60,50},{20,70}}),
        Rectangle(
          origin={-0.179,21},
          fillColor={95,95,95},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{60,-10},{80,10}}),
      Text(
        extent={{-100,-60},{100,-100}},
        textColor={238,46,47},
        textStyle={TextStyle.Italic},
        textString="OpenIMDML")}),
  version="1",
  conversion(noneFromVersion=""));
end OpenIMDML;
