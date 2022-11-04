within OpenIMDML.MultiDomainModels.Motors;
model MD_ALL_IN_ONE_ThreePhaseMotor
  "All-in-one Three-Phase Induction Motor ModelFramework for all three-phase MultiDomain motor models"
  parameter OpenIPSL.Types.ApparentPower M_b = 15e6 "Machine base power"
                                                                        annotation (Dialog(group="Power flow data"));
  extends OpenIPSL.Electrical.Essentials.pfComponent(
    final enablefn=false,
    final enableV_b=false,
    final enableangle_0=false,
    final enablev_0=false,
    final enableQ_0=false,
    final enableP_0=false,
    final enabledisplayPF=true,
    final enableS_b=false);

  parameter Boolean Sup = true "True: Start-up process, False: Steady-state condition" annotation (Dialog(group="Motor Setup"));
  parameter Boolean Ctrl = true "True: Model for VSD control, False: Model not controllable"
                                                                                            annotation (Dialog(group="Motor Setup"));
  parameter Real N = 1 "Number of pair of Poles"
                                                annotation (Dialog(group="Machine Parameters"));
  parameter Modelica.Units.SI.Time H = 0.4 "Inertia constant"
                                                             annotation (Dialog(group="Machine Parameters"));

  replaceable ThreePhase.BaseClasses.BaseMultiDomainThreePhase motor(
    M_b=M_b,
    P_0=P_0,
    Q_0=Q_0,
    v_0=v_0,
    angle_0=angle_0,
    Sup=Sup,
    Ctrl=Ctrl,
    N=N,
    H=H) annotation (
    Dialog(group="Motor Selection"),
    choicesAllMatching=true,
    Placement(transformation(extent={{10,-10},{-10,10}})));
  OpenIPSL.Interfaces.PwPin pwpin
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flange1
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  Modelica.Blocks.Interfaces.RealOutput wr
    "Absolute angular velocity of flange as output signal" annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-60,-110}), iconTransformation(
        extent={{-11.7614,-11.7642},{28.2274,-51.7642}},
        rotation=270,
        origin={-28.2358,-111.761})));
  Modelica.Blocks.Interfaces.RealInput we if Ctrl annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-120})));
  Modelica.Blocks.Interfaces.RealInput mech_torque annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={60,-120})));
equation
  connect(motor.p, pwpin)
    annotation (Line(points={{-10,0},{-100,0}}, color={0,0,255}));
  connect(motor.flange, flange1)
    annotation (Line(points={{10,0},{100,0}}, color={0,0,0}));
  connect(motor.wr, wr) annotation (Line(points={{-6,-12},{-6,-96},{-60,-96},{-60,
          -110}}, color={0,0,127}));
  connect(motor.we, we)
    annotation (Line(points={{0,-12},{0,-120}}, color={0,0,127}));
  connect(motor.mech_torque, mech_torque) annotation (Line(points={{6,-12},{6,-96},
          {60,-96},{60,-120}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Text(
          extent={{-90,100},{90,60}},
          textColor={0,140,72},
          textString="MD ALL-IN-ONE"),
          Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,255}),
                                           Text(
          extent={{-50,48},{50,-52}},
          lineColor={0,0,0},
          textString="M"),                Ellipse(
          fillColor={255,255,255},
          extent={{-60,-60},{60,60}}),
                          Text(
          origin={0,-80},
          extent={{-70,-10},{70,10}},
          fontName="Arial",
          lineColor={0,0,0},
          textString="%name")}),                                 Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end MD_ALL_IN_ONE_ThreePhaseMotor;
