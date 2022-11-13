within OpenIMDML.Examples.MultiDomainExamples.SinglePhaseInductionMotors;
model MultiDomainSPIM
  extends Modelica.Icons.Example;
  extends OpenIMDML.Examples.BaseClasses.ValidationPartial1;
  OpenIMDML.MultiDomainModels.Motors.SinglePhase.MD_SPIM SPIM(
    M_b(displayUnit="V.A") = 3500,
    V_b=230,
    P_0=1000,
    Q_0=5000,
    v_0=1,
    angle_0=0,
    N=1,
    H=0.1,
    R1=0.01,
    R2=0.05,
    X1=0.01,
    X2=0.01,
    Xm=0.1)
         annotation (Placement(transformation(extent={{20,-10},{0,10}})));
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor
    annotation (Placement(transformation(extent={{24,-10},{44,10}})));
  Modelica.Mechanics.Rotational.Components.Inertia load_inertia(J=0.1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,0})));
  Modelica.Mechanics.Rotational.Sources.Torque torque
    annotation (Placement(transformation(extent={{60,-60},{80,-40}})));
  Modelica.Blocks.Sources.Ramp VariableTorque(
    height=-10,
    duration=3.9,
    offset=-0.01,
    startTime=0.1)
    annotation (Placement(transformation(extent={{20,-60},{40,-40}})));
  Modelica.Mechanics.Rotational.Components.SpringDamper springDamper(c=0.5, d=0.5)
             annotation (Placement(transformation(extent={{52,-10},{72,10}})));
equation
  connect(torque.flange, load_inertia.flange_b) annotation (Line(points={{80,
          -50},{106,-50},{106,0},{100,0}}, color={0,0,0}));
  connect(VariableTorque.y, torque.tau)
    annotation (Line(points={{41,-50},{58,-50}}, color={0,0,127}));
  connect(SPIM.flange, torqueSensor.flange_a)
    annotation (Line(points={{20,0},{24,0}}, color={0,0,0}));
  connect(torqueSensor.tau, SPIM.mech_torque) annotation (Line(points={{26,-11},
          {26,-20},{16,-20},{16,-12}}, color={0,0,127}));
  connect(load_inertia.flange_a, springDamper.flange_b)
    annotation (Line(points={{80,0},{72,0}}, color={0,0,0}));
  connect(springDamper.flange_a, torqueSensor.flange_b)
    annotation (Line(points={{52,0},{44,0}}, color={0,0,0}));
  connect(load_bus.p, SPIM.p)
    annotation (Line(points={{-16,0},{0,0}}, color={0,0,255}));
  annotation ( preferredView = "info",
              Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,
            -100},{120,100}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},{120,
            100}})),
    experiment(
      StopTime=10,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"),
    Documentation(info="<html>
<p>The example MultiDomainSPIM contains a multi-domain single-phase induction motor (SPIM) model which drives a mechanical load. The SPIM induction motor circuitry and operation are in steady-state, which explains the lack of start-up capability of this model. </p>
<p>The reader can find more details about the single-phase induction motor modeling in Chapter 09 - <i>Fitzgerald, Arthur Eugene, et al. Electric machinery. Vol. 5. New York: McGRAW-hill, 2003.</i> </p>
<p>Simulate the system for 10 seconds. Variables of interest are:</p>
<ul>
<li><code>SPIM.s</code></li>
<li><code>SPIM.wr</code></li>
<li><code>SPIM.P</code></li>
<li><code>SPIM.Q</code></li>
<li><code>SPIM.Pc</code></li>
<li><code>SPIM.Qc</code></li>
</ul>
</html>"));
end MultiDomainSPIM;
