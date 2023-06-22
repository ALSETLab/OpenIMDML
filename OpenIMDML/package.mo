within ;
package OpenIMDML "Open Instance Multi-Domain Machine
Library using Modelica"

annotation (preferredView = "info",uses(
    Modelica(version="4.0.0"),
    OpenIPSL(version="3.1.0-dev"),
    Complex(version="4.0.0"),
    DymolaModels(version="1.5.0")), Icon(graphics={
      Rectangle(
        extent={{-100,100},{100,-100}},
        lineColor={238,46,47},
        fillPattern=FillPattern.Solid,
        radius=25,
        fillColor={215,215,215},
        lineThickness=1),
        Polygon(
          origin={5.821,39},
          fillPattern=FillPattern.Solid,
          points={{-70,-90},{-60,-90},{-30,-20},{20,-20},{50,-90},{60,-90},{
              60,-100},{-70,-100},{-70,-90}},
        lineColor={95,95,95},
        pattern=LinePattern.None),
        Rectangle(
          origin={-0.62821,16.3307},
          fillColor={238,46,47},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-51.5508,-55.3307},{62.6282,55.6693}},
        lineColor={255,0,0}),
        Rectangle(
          origin={8.537,16.5},
          fillColor={128,128,128},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-80.716,-55.5},{-60.537,55.5}}),
        Rectangle(
          origin={1.821,17},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid,
          extent={{-60,50},{20,70}}),
        Rectangle(
          origin={1.821,15},
          fillColor={95,95,95},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{60,-10},{80,10}}),
      Text(
        extent={{-96,-66},{86,-94}},
        textColor={238,46,47},
        textStyle={TextStyle.Italic},
          textString="OpenIMDML")}),
  version="1",
  conversion(noneFromVersion=""));
end OpenIMDML;
