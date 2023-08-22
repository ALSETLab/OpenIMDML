within OpenIMDML.Controls.VariableSpeedDrive;
package PowerElectronics "Variable speed drive power electronics model"

  model AC2DCandDC2AC "Phasor based voltage source converter model."
     extends OpenIPSL.Electrical.Essentials.pfComponent(
      final enabledisplayPF=false,
      final enablefn=false,
      final enableV_b=false,
      final enableangle_0=true,
      final enableP_0 = false,
      final enableQ_0=false,
      final enablev_0=true,
      final enableS_b=true);

      import Modelica.Constants.eps;
      import Modelica.Constants.pi;
    OpenIPSL.Interfaces.PwPin p
      annotation (Placement(transformation(extent={{-110,-10},{-90,10}}),
          iconTransformation(extent={{-110,-10},{-90,10}})));
    OpenIPSL.Interfaces.PwPin n annotation (Placement(transformation(extent={{110,-10},
              {130,10}}), iconTransformation(extent={{110,-10},{130,10}})));
    Modelica.Electrical.Analog.Sources.SignalVoltage Voltage annotation (
        Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={-46,-14})));
    Modelica.Blocks.Sources.RealExpression Vd0(y=3*sqrt(6)*Vs.y*(V_b)/Modelica.Constants.pi)
                                               annotation (Placement(transformation(extent={{-84,-24},
              {-64,-4}})));
    Modelica.Electrical.Analog.Basic.Resistor Resistor(R=Rdc)
      annotation (Placement(transformation(extent={{-42,-10},{-22,10}})));
    Modelica.Electrical.Analog.Basic.Inductor Inductor(i(start=Il0, fixed=true),
                                                       L=Ldc)
      annotation (Placement(transformation(extent={{-16,-10},{4,10}})));
    Modelica.Electrical.Analog.Ideal.IdealOpeningSwitch switch(Ron=1e-5, Goff=1e-5)
      annotation (Placement(transformation(extent={{10,-10},{30,10}})));
    Modelica.Electrical.Analog.Basic.Capacitor Capacitor(v(start=Vc0, fixed=true),
                                                                       C=Cdc)
                                                                 annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={30,-14})));
    Modelica.Electrical.Analog.Sources.SignalCurrent signalCurrent annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={50,-14})));
    Modelica.Blocks.Sources.RealExpression Ii(y=Pmotor.y*S_b/Capacitor.v)
                                                                 annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=180,
          origin={78,-14})));
    Modelica.Blocks.Sources.RealExpression Vs(y=sqrt(p.vr^2 + p.vi^2))
      annotation (Placement(transformation(extent={{-70,50},{-50,70}})));
    Modelica.Blocks.Sources.BooleanExpression open_circuit_condition(y=if
          Resistor.i < 0 then true else false)
      annotation (Placement(transformation(extent={{52,6},{32,26}})));
    Modelica.Blocks.Sources.RealExpression Pmotor(y=-(n.vr*n.ir + n.vi*n.ii))
      annotation (Placement(transformation(extent={{24,70},{44,90}})));
    Modelica.Blocks.Sources.RealExpression Qmotor(y=n.vr*n.ii - n.vi*n.ir)
      annotation (Placement(transformation(extent={{24,54},{44,74}})));
    OpenIPSL.Types.PerUnit P;
    OpenIPSL.Types.PerUnit Q;
    OpenIPSL.Types.PerUnit S;

    Modelica.Blocks.Sources.RealExpression Vmotor(y=Capacitor.v*m_input/(2*sqrt(2)
          *V_b))
      annotation (Placement(transformation(extent={{24,38},{44,58}})));
    Modelica.Blocks.Interfaces.RealInput m_input annotation (Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={60,-120}), iconTransformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={60,-120})));
    Modelica.Blocks.Sources.RealExpression vr_m(y=Vmotor.y*cos(0))
      annotation (Placement(transformation(extent={{62,70},{82,90}})));
    Modelica.Blocks.Sources.RealExpression vi_m(y=Vmotor.y*sin(0))
      annotation (Placement(transformation(extent={{62,54},{82,74}})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{-14,-50},{6,-30}})));

    Modelica.Blocks.Interfaces.RealOutput Vc "Value of Real output" annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-50,-110}), iconTransformation(
          extent={{-18.1811,-18.1819},{21.819,-58.1819}},
          rotation=270,
          origin={-1.8181,-118.181})));
    parameter Modelica.Units.SI.Resistance Rdc=0.1
      "DC Link Resistance"
      annotation (Dialog(group="DC Link Parameters"));
    parameter Modelica.Units.SI.Inductance Ldc=0.001
      "DC Link Inductance"
      annotation (Dialog(group="DC Link Parameters"));
    parameter Modelica.Units.SI.Capacitance Cdc=0.02
      "DC Link Capacitance"
      annotation (Dialog(group="DC Link Parameters"));
    parameter Real m0= 0.1 "Initial PWM Modulation Value" annotation (Dialog(group="DC Link Parameters"));
    Modelica.Blocks.Sources.RealExpression Smotor(y=sqrt(Pmotor.y^2 + Qmotor.y^2))
      annotation (Placement(transformation(extent={{62,38},{82,58}})));
  protected
    parameter OpenIPSL.Types.Voltage Vd00 = 3*sqrt(6)*v_0*V_b/Modelica.Constants.pi;
    parameter OpenIPSL.Types.Voltage Vc0 = 2*sqrt(2)*Vmotor0*V_b/m0;
    parameter OpenIPSL.Types.Current Il0 = 0;
    parameter OpenIPSL.Types.PerUnit Vmotor0 = (3*sqrt(3)/(2*pi))*m0;

    Modelica.Blocks.Sources.RealExpression Capacitor_Voltage(y=Capacitor.v)
      annotation (Placement(transformation(extent={{-84,-84},{-64,-64}})));
  //initial equation
    //der(Resistor.i) = 0;
    //der(Capacitor.v) = 0;

  equation
    connect(Vd0.y, Voltage.v)
      annotation (Line(points={{-63,-14},{-58,-14}}, color={0,0,127}));
    connect(Voltage.p, Resistor.p)
      annotation (Line(points={{-46,-4},{-46,0},{-42,0}},      color={0,0,255}));
    connect(Resistor.n, Inductor.p)
      annotation (Line(points={{-22,0},{-16,0}},     color={0,0,255}));
    connect(Inductor.n, switch.p)
      annotation (Line(points={{4,0},{10,0}},     color={0,0,255}));
    connect(switch.n, Capacitor.p)
      annotation (Line(points={{30,0},{30,-4}},    color={0,0,255}));
    connect(Voltage.n, Capacitor.n) annotation (Line(points={{-46,-24},{-46,-28},{
            30,-28},{30,-24}}, color={0,0,255}));
    connect(switch.n, signalCurrent.p) annotation (Line(points={{30,0},{50,0},{50,
            -4}},                                                                            color={0,0,255}));
    connect(signalCurrent.n, Capacitor.n) annotation (Line(points={{50,-24},{50,-28},
            {30,-28},{30,-24}}, color={0,0,255}));
    connect(signalCurrent.i, Ii.y) annotation (Line(points={{62,-14},{67,-14}}, color={0,0,127}));
    connect(open_circuit_condition.y, switch.control)
      annotation (Line(points={{31,16},{20,16},{20,12}},    color={255,0,255}));
      P =  p.vr*p.ir + p.vi*p.ii;
      Q = (-p.vr*p.ii) + p.vi*p.ir;
      Q = 0;
      S = sqrt(P^2 + Q^2);
      Resistor.i = smooth(0,(P*S_b)/Vd0.y);

      n.vr = vr_m.y;
      n.vi = vi_m.y;
    connect(ground.p, Capacitor.n) annotation (Line(points={{-4,-30},{-4,-28},{30,
            -28},{30,-24}},
                       color={0,0,255}));
    connect(Capacitor_Voltage.y, Vc) annotation (Line(points={{-63,-74},{-50,-74},
            {-50,-110}}, color={0,0,127}));
    annotation (preferredView = "info",Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
              {120,100}}),
          graphics={Rectangle(
            extent={{-100,100},{120,-100}},
            lineColor={28,108,200}),
          Line(points={{-100,-100},{120,100}}, color={28,108,200}),
          Text(
            extent={{-80,80},{40,20}},
            textColor={28,108,200},
            textString="AC/DC"),
          Text(
            extent={{-20,-20},{100,-80}},
            textColor={28,108,200},
            textString="DC/AC")}),                                                                 Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={Rectangle(
            extent={{14,90},{94,40}},
            lineColor={0,0,255},
            pattern=LinePattern.Dash), Text(
            extent={{24,98},{84,92}},
            lineColor={0,0,255},
            pattern=LinePattern.Dash,
            textString="Motor Variables"),                                                       Rectangle(
            extent={{-80,80},{-40,40}},
            lineColor={0,0,255},
            pattern=LinePattern.Dash), Text(
            extent={{-92,88},{-30,82}},
            lineColor={0,0,255},
            pattern=LinePattern.Dash,
            textString="Grid Variables"),                                                        Rectangle(
            extent={{-94,-54},{-54,-94}},
            lineColor={0,0,255},
            pattern=LinePattern.Dash), Text(
            extent={{-106,-46},{-44,-52}},
            lineColor={0,0,255},
            pattern=LinePattern.Dash,
            textString="DC Link Variables")}));
  end AC2DCandDC2AC;
  annotation (preferredView = "info", Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0,
          lineThickness=0.5),
        Polygon(points={{36,0},{-34,50},{-34,-50},{36,0}},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid,
          lineThickness=1,
          pattern=LinePattern.None,
          lineColor={238,46,47}),
        Line(
          points={{-78,0},{-34,0}},
          color={238,46,47},
          thickness=1),
        Line(
          points={{36,50},{36,-52}},
          color={238,46,47},
          thickness=1),
        Line(
          points={{36,0},{80,0}},
          color={238,46,47},
          thickness=1),
        Line(
          points={{36,-16},{64,-38}},
          color={238,46,47},
          thickness=1),
        Line(
          points={{64,-38},{64,-56}},
          color={238,46,47},
          thickness=1)}));
end PowerElectronics;
