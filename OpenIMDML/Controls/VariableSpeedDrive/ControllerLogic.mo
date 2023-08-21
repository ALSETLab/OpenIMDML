within OpenIMDML.Controls.VariableSpeedDrive;
package ControllerLogic "Controller logic for variable speed drive"

  model VoltsHertzController "Volts/Hertz controller model"
    extends OpenIPSL.Electrical.Essentials.pfComponent(
      final enabledisplayPF=false,
      final enablefn=false,
      final enableV_b=false,
      final enableangle_0=false,
      final enablev_0=false,
      final enableQ_0=false,
      final enableS_b=true);

      import Modelica.Constants.pi;

     parameter Real f_max = 80 "Maximum input voltage frequency" annotation (Dialog(group="VSD project specifics"));
     parameter Real f_min = 40 "Minimum input voltage frequency" annotation (Dialog(group="VSD project specifics"));
     parameter Real m0= 0.1 "Initial PWM Modulation Value" annotation (Dialog(group="VSD project specifics"));

    Modelica.Blocks.Interfaces.RealInput motor_speed annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-110,-20}),
                            iconTransformation(
          extent={{-20,-20},{20,20}},
          rotation=180,
          origin={120,40})));
    OpenIPSL.NonElectrical.Continuous.SimpleLag Speed_Sensor(K=1, T=Tr,
      y_start=Modelica.Constants.eps)
      annotation (Placement(transformation(extent={{-88,-30},{-68,-10}})));
    parameter OpenIPSL.Types.Time Tr=0.01 "Lag time constant"
      annotation (Dialog(group="Control Parameters"));
    Modelica.Blocks.Math.Add add(k1=-1)
      annotation (Placement(transformation(extent={{-88,-68},{-68,-48}})));
    Modelica.Blocks.Nonlinear.Limiter limiter(uMax=we_max, uMin=we_min)
      annotation (Placement(transformation(extent={{44,-36},{64,-16}})));
    Modelica.Blocks.Math.Gain gain(k=Kp)
      annotation (Placement(transformation(extent={{-48,-52},{-28,-32}})));
    Modelica.Blocks.Continuous.Integrator integrator(k=Ki,
      initType=Modelica.Blocks.Types.Init.InitialState,
      y_start=m0*(2*Modelica.Constants.pi*SysData.fn))
      annotation (Placement(transformation(extent={{-48,-84},{-28,-64}})));
    Modelica.Blocks.Math.Add add1(k1=+1)
      annotation (Placement(transformation(extent={{-18,-68},{2,-48}})));
    Modelica.Blocks.Math.Add add2(k1=+1)
      annotation (Placement(transformation(extent={{16,-36},{36,-16}})));
    Modelica.Blocks.Interfaces.RealOutput we(start=0.01*2*Modelica.Constants.pi*
          SysData.fn)
                 "Connector of Real output signal"
      annotation (Placement(transformation(extent={{100,-60},{140,-20}}),
          iconTransformation(extent={{100,-60},{140,-20}})));
    Modelica.Blocks.Math.Gain gain1(k=1)
      annotation (Placement(transformation(extent={{10,-10},{-10,10}},
          rotation=0,
          origin={30,10})));
    Modelica.Blocks.Interfaces.RealOutput m "Output signal connector"
      annotation (Placement(transformation(extent={{-20,-20},{20,20}},
          rotation=90,
          origin={46,120}), iconTransformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={46,120})));
      Real Kf= 1/(2*pi*fn) "Gain value multiplied with input signal"
      annotation (Dialog(group="Control Parameters"));
    parameter Real Kp=5 "Gain value multiplied with input signal"
      annotation (Dialog(group="Control Parameters"));
    parameter Real Ki=0.1 "Integrator gain"
      annotation (Dialog(group="Control Parameters"));
    parameter Real we_max=2*pi*f_max "Maximum Synchronous Speed"
      annotation (Dialog(group="Control Parameters"));
    parameter Real we_min=2*pi*f_min "Minimum Synchronous Speed"
      annotation (Dialog(group="Control Parameters"));
    Modelica.Blocks.Interfaces.RealInput Vc annotation (Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=270,
          origin={-46,120}), iconTransformation(
          extent={{-20,-20},{20,20}},
          origin={-46,120},
          rotation=270)));
    Modelica.Blocks.Interfaces.RealInput W_ref "Connector of Real input signal 2"
      annotation (Placement(transformation(extent={{-140,-20},{-100,20}}),
          iconTransformation(extent={{-140,-20},{-100,20}})));
    Modelica.Blocks.Nonlinear.Limiter limiter1(uMax=1, uMin=0) annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={24,52})));
    Modelica.Blocks.Math.Gain gain2(k=1/V_b)
      annotation (Placement(transformation(extent={{-52,48},{-32,68}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=Kf)
      annotation (Placement(transformation(extent={{-88,20},{-68,40}})));
    Modelica.Blocks.Math.Product product1
      annotation (Placement(transformation(extent={{-52,14},{-32,34}})));
    Modelica.Blocks.Continuous.FirstOrder firstOrder(
      T=0.01,
      initType=Modelica.Blocks.Types.Init.InitialState,
      y_start=m0)
      annotation (Placement(transformation(extent={{48,42},{68,62}})));
  equation
    connect(motor_speed, Speed_Sensor.u) annotation (Line(points={{-110,-20},{-90,
            -20}},               color={0,0,127}));
    connect(Speed_Sensor.y, add.u1) annotation (Line(points={{-67,-20},{-62,-20},{
            -62,-38},{-96,-38},{-96,-52},{-90,-52}},
                             color={0,0,127}));
    connect(add.y, gain.u) annotation (Line(points={{-67,-58},{-56,-58},{-56,-42},
            {-50,-42}},
                 color={0,0,127}));
    connect(integrator.u, gain.u) annotation (Line(points={{-50,-74},{-56,
            -74},{-56,-42},{-50,-42}},
                         color={0,0,127}));
    connect(gain.y, add1.u1) annotation (Line(points={{-27,-42},{-22,-42},{-22,-52},
            {-20,-52}},
                   color={0,0,127}));
    connect(integrator.y, add1.u2) annotation (Line(points={{-27,-74},{
            -22,-74},{-22,-64},{-20,-64}},
                            color={0,0,127}));
    connect(add2.u1, Speed_Sensor.y)
      annotation (Line(points={{14,-20},{-67,-20}},
                                                  color={0,0,127}));
    connect(add.u2, W_ref) annotation (Line(points={{-90,-64},{-100,-64},{-100,0},
            {-120,0}},   color={0,0,127}));
    connect(Vc, gain2.u) annotation (Line(points={{-46,120},{-46,74},{-66,74},{-66,
            58},{-54,58}},          color={0,0,127}));
    connect(gain1.u, we) annotation (Line(points={{42,10},{72,10},{72,-40},{120,-40}},
                                                             color={0,0,
            127}));
    connect(add1.y, add2.u2) annotation (Line(points={{3,-58},{8,-58},{8,-32},{14,
            -32}},             color={0,0,127}));
    connect(add2.y, limiter.u)
      annotation (Line(points={{37,-26},{42,-26}}, color={0,0,127}));
    connect(limiter.y, we) annotation (Line(points={{65,-26},{88,-26},{88,-40},{120,
            -40}},       color={0,0,127}));
    connect(integrator.u, add.y) annotation (Line(points={{-50,-74},{-56,
            -74},{-56,-58},{-67,-58}},
                             color={0,0,127}));
    connect(realExpression.y, product1.u1)
      annotation (Line(points={{-67,30},{-54,30}}, color={0,0,127}));
    connect(gain1.y, product1.u2) annotation (Line(points={{19,10},{-66,10},{-66,18},
            {-54,18}}, color={0,0,127}));
    connect(limiter1.y, firstOrder.u) annotation (Line(points={{35,52},{46,52}},
                             color={0,0,127}));
    connect(firstOrder.y, m) annotation (Line(points={{69,52},{72,52},{72,78},{46,
            78},{46,120}}, color={0,0,127}));
    connect(product1.y, limiter1.u) annotation (Line(points={{-31,24},{-4,24},{-4,
            52},{12,52}}, color={0,0,127}));
    annotation (preferredView = "info", Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
              {80,100}}),                                         graphics={
          Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={28,108,200}),        Text(
            extent={{-80,82},{80,-78}},
            lineColor={28,108,200},
            textString=" V/f
Control")}),                                                       Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{80,100}})));
  end VoltsHertzController;

  model PIcontroller4pumpexample "PI controller for pump example"
    Modelica.Blocks.Interfaces.RealInput m_flow_ref annotation (Placement(
          transformation(extent={{-120,50},{-100,70}}), iconTransformation(extent=
             {{-120,50},{-100,70}})));
    Modelica.Blocks.Math.Add add(k2=-1) annotation (Placement(
          transformation(extent={{-54,-10},{-34,10}})));
    Modelica.Blocks.Continuous.Integrator integrator(k=ki, y_start=1)
      annotation (Placement(transformation(extent={{-10,-40},{10,-20}})));
    Modelica.Blocks.Math.Gain gain(k=kp)
      annotation (Placement(transformation(extent={{-8,20},{12,40}})));
    Modelica.Blocks.Math.Add add1(k2=+1)
      annotation (Placement(transformation(extent={{26,-10},{46,10}})));
    Modelica.Blocks.Math.Gain PI_convert(k=mflow_2_speed)
      annotation (Placement(transformation(extent={{56,-10},{76,10}})));
    Modelica.Blocks.Interfaces.RealInput m_flow annotation (Placement(
          transformation(extent={{-120,-70},{-100,-50}}), iconTransformation(
            extent={{-120,-70},{-100,-50}})));
    Modelica.Blocks.Interfaces.RealOutput Wref "Output signal connector"
      annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    parameter Real kp=1 "Proportional Gain in the PI Controller."
      annotation (Dialog(group="Pump Control Setting"));
    parameter Real ki=0.1 "Integrator gain in the PI Controller."
      annotation (Dialog(group="Pump Control Setting"));
    parameter Real mflow_2_speed=188.495/585.18
      "Linear gain converter from mass flow rate to motor reference speed."
      annotation (Dialog(group="Pump Control Setting"));
  equation
    connect(gain.u,add. y) annotation (Line(points={{-10,30},{-28,30},{-28,0},{-33,
            0}},        color={0,0,127}));
    connect(integrator.u,add. y) annotation (Line(points={{-12,-30},{-28,-30},{-28,
            0},{-33,0}},      color={0,0,127}));
    connect(integrator.y,add1. u2) annotation (Line(points={{11,-30},{22,-30},{22,
            -6},{24,-6}},    color={0,0,127}));
    connect(gain.y,add1. u1) annotation (Line(points={{13,30},{22,30},{22,6},{24,6}},
                       color={0,0,127}));
    connect(add1.y,PI_convert. u)
      annotation (Line(points={{47,0},{54,0}},       color={0,0,127}));
    connect(m_flow_ref, add.u1) annotation (Line(points={{-110,60},{-62,60},{-62,6},
            {-56,6}}, color={0,0,127}));
    connect(m_flow, add.u2) annotation (Line(points={{-110,-60},{-62,-60},{-62,-6},
            {-56,-6}}, color={0,0,127}));
    connect(PI_convert.y, Wref)
      annotation (Line(points={{77,0},{110,0}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200}),
            Text(
            extent={{-78,30},{80,-20}},
            lineColor={0,0,0},
            textString="%name")}), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end PIcontroller4pumpexample;
  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0,
          lineThickness=0.5),
                        Line(
          origin={0.061,4.184},
          points={{81.939,36.056},{65.362,36.056},{14.39,-26.199},{-29.966,
              113.485},{-65.374,-61.217},{-78.061,-78.184}},
          color={238,46,47},
          smooth=Smooth.Bezier,
          thickness=1)}));
end ControllerLogic;
