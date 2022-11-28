within OpenIMDML.MultiDomainModels.Motors.SinglePhase.BaseClasses;
partial model BaseMultiDomainSinglePhase_forpaper
  "Base class model for the single-phase and dual-phase multi-domain induction motor models."

    Modelica.Blocks.Sources.RealExpression Rotor_Speed(y=nr)
    annotation (Placement(transformation(extent={{70,-10},{50,10}})));
  Modelica.Mechanics.Rotational.Components.Inertia Rotor_Inertia(J=J_load, w(fixed=
          true, start=w0))
    annotation (Placement(transformation(extent={{-40,-10},{-60,10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flange annotation (
      Placement(transformation(extent={{-110,-10},{-90,10}}),
        iconTransformation(extent={{-110,-10},{-90,10}})));
  Modelica.Blocks.Interfaces.RealOutput wr
    "Absolute angular velocity of flange as output signal"
    annotation (Placement(transformation(extent={{-100,-50},{-120,-30}}),
        iconTransformation(
        extent={{20,-20},{-20,20}},
        rotation=90,
        origin={60,-120})));
  Modelica.Blocks.Interfaces.RealInput mech_torque annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-120}),   iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={-60,-120})));
         OpenIPSL.Interfaces.PwPin pwpin
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));

    parameter OpenIPSL.Types.ApparentPower M_b = 15e6 "Machine base power"
                                                                          annotation (Dialog(group="Power flow data"));
    extends OpenIPSL.Electrical.Essentials.pfComponent(
    final enabledisplayPF=false,
    final enablefn=false,
    final enableV_b=true,
    final enableangle_0=false,
    final enablev_0=false,
    final enableQ_0=false,
    final enableP_0=false,
    final enableS_b=true);

  import Modelica.Constants.pi;
  import OpenIPSL.NonElectrical.Functions.SE;
  import Modelica.Constants.eps;

  parameter Boolean Sup = true "True: Start-up process, False: Steady-state condition" annotation (Dialog(group="Motor Setup"));
  parameter Real N = 1 "Number of pair of Poles"
                                                annotation (Dialog(group="Machine parameters"));
  parameter Modelica.Units.SI.Time H = 0.4 "Inertia constant"
                                                             annotation (Dialog(group="Machine parameters"));

  Modelica.Units.SI.AngularVelocity nr;
  Modelica.Units.SI.AngularVelocity ns;
  OpenIPSL.Types.PerUnit s;
  Modelica.Units.SI.Torque T_b;
  OpenIPSL.Types.PerUnit Tmech_pu_sys;
  OpenIPSL.Types.PerUnit Tmech_pu_motor;

  Modelica.Mechanics.Rotational.Sources.Speed speed(exact=true) annotation (Placement(
        transformation(
        extent={{-10,10},{10,-10}},
        rotation=180)));

protected
  parameter Modelica.Units.SI.Inertia J_load = 2*H*M_b/((2*pi*fn/N)^2);
  parameter Modelica.Units.SI.AngularVelocity w_b=2*pi*fn/N "Base freq in rad/s";
  parameter Real CoB = M_b/S_b;
  parameter Modelica.Units.SI.AngularVelocity w0 = if Sup == true then Modelica.Constants.eps else w_b;
equation

  //Rotor speed equation
  ns = w_b;
  nr = (1-s)*ns;

  //Conversion from SI torqur to p.u. torque
  Tmech_pu_sys = mech_torque/T_b;
  Tmech_pu_motor = Tmech_pu_sys/CoB;

  //Torque System base
  T_b = S_b/w_b;

  connect(Rotor_Inertia.flange_b,flange)
    annotation (Line(points={{-60,0},{-100,0}}, color={0,0,0}));
  connect(speed.w_ref,Rotor_Speed. y)
    annotation (Line(points={{12,-1.9984e-15},{12,0},{49,0}},
                                                       color={0,0,127}));
  connect(speed.flange,Rotor_Inertia. flange_a) annotation (Line(points={{-10,7.21645e-16},
          {-40,0}},              color={0,0,0}));
  connect(wr,Rotor_Speed. y) annotation (Line(points={{-110,-40},{40,-40},{40,0},
          {49,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0}),              Text(
          extent={{-50,44},{50,-56}},
          lineColor={0,0,0},
          textString="M"),                Ellipse(
          fillColor={255,255,255},
          extent={{-56,-56},{56,56}}),
        Text(
          extent={{-176,20},{-116,-20}},
          textColor={28,108,200},
          textString="flange"),
        Text(
          extent={{120,22},{180,-18}},
          textColor={28,108,200},
          textString="pwpin"),
                          Text(
          origin={0,-80},
          extent={{-80,-20},{80,20}},
          fontName="Arial",
          lineColor={0,0,0},
          textString="BaseMultiDomainMotor
Single-Phase"),
        Text(
          extent={{-90,90},{-30,50}},
          textColor={0,140,72},
          textString="MD"),
        Text(
          extent={{46,-138},{76,-178}},
          textColor={28,108,200},
          textString="wr"),
        Text(
          extent={{-124,-84},{10,-234}},
          textColor={28,108,200},
          textString="mech_torque")}),                           Diagram(
        coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={28,108,200},
          lineThickness=0.5)}));
end BaseMultiDomainSinglePhase_forpaper;
