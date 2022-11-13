within OpenIMDML.Examples.MultiDomainExamples.ThreePhaseInductionMotors.MotorValidation;
model ValidationMultiDomainTypeI
  extends Modelica.Icons.Example;
  extends OpenIMDML.Examples.BaseClasses.ValidationPartial2;
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor1
    annotation (Placement(transformation(extent={{82,-10},{102,10}})));
  Modelica.Mechanics.Rotational.Components.Inertia load_inertia1(J=1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={122,0})));
  Modelica.Mechanics.Rotational.Sources.Torque torque1
    annotation (Placement(transformation(extent={{112,-50},{132,-30}})));
  Modelica.Blocks.Sources.RealExpression Torque1(y=-(0.1*(15/100)*(Motor1.s) + 0.5
        *(15/100)*(1 - Motor1.s)^2)*Motor1.T_b)
    annotation (Placement(transformation(extent={{72,-50},{92,-30}})));
  Modelica.Blocks.Sources.RealExpression SS1(y=2*Modelica.Constants.pi*(
        SysData.fn))
                annotation (Placement(transformation(extent={{20,-46},{40,-26}})));
  MultiDomainModels.Motors.ThreePhase.PSAT.MotorTypeI Motor1(V_b=23000)
    annotation (Placement(transformation(extent={{68,-10},{48,10}})));
equation
  connect(torqueSensor1.flange_b,load_inertia1. flange_a)
    annotation (Line(points={{102,0},{112,0}},   color={0,0,0}));
  connect(torque1.flange,load_inertia1. flange_b) annotation (Line(points={{132,-40},
          {142,-40},{142,0},{132,0}},      color={0,0,0}));
  connect(Torque1.y,torque1. tau)
    annotation (Line(points={{93,-40},{110,-40}},color={0,0,127}));
  connect(Motor1.flange, torqueSensor1.flange_a)
    annotation (Line(points={{68,0},{82,0}}, color={0,0,0}));
  connect(torqueSensor1.tau, Motor1.mech_torque) annotation (Line(points={{84,-11},
          {84,-20},{64,-20},{64,-12}}, color={0,0,127}));
  connect(SS1.y, Motor1.we)
    annotation (Line(points={{41,-36},{58,-36},{58,-12}}, color={0,0,127}));
  connect(Motor1.p, bus4_mt1.p)
    annotation (Line(points={{48,0},{20,0}}, color={0,0,255}));
  annotation (preferredView = "info",Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
            -100},{160,100}})),
                          Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-160,-100},{160,100}})),
    experiment(
      StopTime=10,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"),
    Documentation(info="<html>
<p>The example MultiDomainTypeIValidation contains a multi-domain three-phase type I induction motor model which drives a mechanical load. 
The three-phase induction motor type I is based on the non multi-domain model from <i>Milano, Federico. Power system modelling and scripting. Springer Science & Business Media, 2010. </i><\\p>
<p>In order to validate the multi-domain model, this validation example reproduces Example 11.7 Motor Startup from <i>Chow, Joe H., and Juan J. Sanchez-Gasca. Power system modeling, computation, and control. John Wiley & Sons, 2020.</i> </p>
<p>The startup example from the book utilizes a <strong>three-phase type III induction motor </strong>. In this particular Modelica validation example, the idea is to test the more simple model type I to compare results.</p>
<p>Simulate the system for 10 seconds. Variables of interest are:</p>
<ul>
<li><code>Motor1.s</code></li>
<li><code>Motor1.wr</code></li>
<li><code>Motor1.P</code></li>
<li><code>Motor1.Q</code></li>
<li><code>Motor1.Pmotor</code></li>
</ul>
</html>"));
end ValidationMultiDomainTypeI;
