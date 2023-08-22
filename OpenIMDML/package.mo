within ;
package OpenIMDML "Open Instance Multi-Domain Machine
Library using Modelica"
annotation (preferredView = "info",uses(
    Modelica(version="4.0.0"),
    OpenIPSL(version="3.0.1"),
    Complex(version="4.0.0")),
     Icon(graphics={
      Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0,
          lineThickness=0.5),
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
          fillPattern=FillPattern.VerticalCylinder,
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
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-60,50},{20,70}}),
        Rectangle(
          origin={1.821,15},
          fillColor={95,95,95},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{60,-10},{80,10}}),
      Text(
        extent={{-102,-62},{102,-98}},
        textColor={238,46,47},
        textStyle={TextStyle.Italic},
          textString="OpenIMDML")}),
  version="1",
  conversion(noneFromVersion=""));
    //DymolaModels(version="1.5.0")),
end OpenIMDML;
