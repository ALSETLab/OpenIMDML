within OpenIMDML.Examples.MultiDomainExamples.SinglePhaseInductionMotors;
model MultiDomainDPIM
  extends Modelica.Icons.Example;
  extends OpenIMDML.Examples.BaseClasses.ValidationPartial1;
  OpenIMDML.MultiDomainModels.Motors.SinglePhase.MD_DPIM_2 DPIM(
    M_b(displayUnit="V.A") = 5000,
    V_b=230,
    P_0=1000,
    Q_0=5000,
    v_0=1,
    angle_0=0,
    N=1,
    H=0.1,
    init=2,
    Sup=true,
    switch_open_speed=0.1,
    Lmainr=0.000588,
    Lmain=0.086,
    Lauxr=0.000909,
    Laux=0.196,
    Lr=0.0000047,
    Rmain=0.58,
    Rr=0.0000376,
    Raux=3.37,
    Cc=0.000035)
    annotation (Placement(transformation(extent={{20,-10},{0,10}})));
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor
    annotation (Placement(transformation(extent={{24,-10},{44,10}})));
  Modelica.Mechanics.Rotational.Components.Inertia load_inertia(J=0.01)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,0})));
  Modelica.Mechanics.Rotational.Sources.Torque torque(useSupport=false)
    annotation (Placement(transformation(extent={{60,-60},{80,-40}})));
  Modelica.Blocks.Sources.Constant ConstantTorque(k=-150)
    annotation (Placement(transformation(extent={{20,-60},{40,-40}})));
  Modelica.Mechanics.Rotational.Components.SpringDamper springDamper(c=0.5, d=
        0.5) annotation (Placement(transformation(extent={{52,-10},{72,10}})));
equation
  connect(torque.flange, load_inertia.flange_b) annotation (Line(points={{80,
          -50},{106,-50},{106,0},{100,0}}, color={0,0,0}));
  connect(DPIM.flange, torqueSensor.flange_a)
    annotation (Line(points={{20,0},{24,0}}, color={0,0,0}));
  connect(torqueSensor.tau, DPIM.mech_torque) annotation (Line(points={{26,-11},
          {26,-20},{16,-20},{16,-12}}, color={0,0,127}));
  connect(ConstantTorque.y, torque.tau)
    annotation (Line(points={{41,-50},{58,-50}}, color={0,0,127}));
  connect(load_bus.p, DPIM.p)
    annotation (Line(points={{-16,0},{0,0}}, color={0,0,255}));
  connect(torqueSensor.flange_b, springDamper.flange_a)
    annotation (Line(points={{44,0},{52,0}}, color={0,0,0}));
  connect(springDamper.flange_b, load_inertia.flange_a)
    annotation (Line(points={{72,0},{80,0}}, color={0,0,0}));
  annotation (preferredView = "info",
  Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},
            {120,100}})),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},{120,100}})),
    experiment(
      StopTime=3,
      __Dymola_NumberOfIntervals=50000,
      __Dymola_Algorithm="Dassl"),
    Documentation(info="<html>
<p>The example MultiDomainDPIM contains a multi-domain dual-phase induction motor (SPIM) model which drives a mechanical load. The DPIM induction motor circuitry is in steady-state, while the operation of the motor can be toggled between steady-state operation and motor start-up.</p>
<p>There are two different modes of operation: Start-up (Sup = 2), and Steady-state (Sup = 1). The user can also select the configuration of the auxiliary winding circuitry, either split-phase or capacity-start. </p>
<p>The reader can find more details about the dual-phase induction motor modeling in Chapter 09 - <i>Fitzgerald, Arthur Eugene, et al. Electric machinery. Vol. 5. New York: McGRAW-hill, 2003.</i> </p>
<p>Simulate the system for 10 seconds. Variables of interest are:</p>
<ul>
<li><code>DPIM.s</code></li>
<li><code>DPIM.wr</code></li>
<li><code>DPIM.P</code></li>
<li><code>DPIM.Q</code></li>
<li><code>DPIM.Pc</code></li>
<li><code>DPIM.Qc</code></li>
</ul>
</html>"));
end MultiDomainDPIM;
