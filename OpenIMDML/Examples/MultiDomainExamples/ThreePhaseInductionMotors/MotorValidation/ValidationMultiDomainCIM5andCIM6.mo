﻿within OpenIMDML.Examples.MultiDomainExamples.ThreePhaseInductionMotors.MotorValidation;
model ValidationMultiDomainCIM5andCIM6
  extends Modelica.Icons.Example;
  extends OpenIMDML.Examples.BaseClasses.ValidationPartial2;
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensorCIM5
    annotation (Placement(transformation(extent={{82,-10},{102,10}})));
  Modelica.Mechanics.Rotational.Sources.Torque torqueCIM5
    annotation (Placement(transformation(extent={{112,-50},{132,-30}})));
  Modelica.Mechanics.Rotational.Components.Inertia load_inertiaCIM5(J=1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={122,0})));
  Modelica.Blocks.Sources.RealExpression TorqueCIM5(y=-(0.1*(15/100)*(CIM.s) +
        0.5*(15/100)*(1 - CIM.s)^2)*CIM.T_b)
    annotation (Placement(transformation(extent={{72,-50},{92,-30}})));
  Modelica.Blocks.Sources.RealExpression SSCIM(y=2*Modelica.Constants.pi*(
        SysData.fn))
    annotation (Placement(transformation(extent={{20,-46},{40,-26}})));
  MultiDomainModels.Motors.ThreePhase.PSSE.CIM5_CIM6 CIM(V_b=23000)
    annotation (Placement(transformation(extent={{68,-10},{48,10}})));
equation
  connect(torqueSensorCIM5.flange_b,load_inertiaCIM5. flange_a)
    annotation (Line(points={{102,0},{112,0}},       color={0,0,0}));
  connect(torqueCIM5.flange,load_inertiaCIM5. flange_b) annotation (Line(points={{132,-40},
          {142,-40},{142,0},{132,0}},                   color={0,0,0}));
  connect(TorqueCIM5.y,torqueCIM5. tau)
    annotation (Line(points={{93,-40},{110,-40}},    color={0,0,127}));
  connect(torqueSensorCIM5.tau, CIM.mech_torque) annotation (Line(points={{84,-11},
          {84,-20},{64,-20},{64,-12}}, color={0,0,127}));
  connect(SSCIM.y, CIM.we)
    annotation (Line(points={{41,-36},{58,-36},{58,-12}}, color={0,0,127}));
  connect(CIM.flange, torqueSensorCIM5.flange_a)
    annotation (Line(points={{68,0},{82,0}}, color={0,0,0}));
  connect(bus4_mt1.p, CIM.p)
    annotation (Line(points={{20,0},{48,0}}, color={0,0,255}));
  annotation (preferredView = "info",Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
            -100},{160,100}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-160,-100},{
            160,100}})),
    experiment(
      StopTime=10,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"),
    Documentation(info="<html>
<p>The example MultiDomainCIM5andCIM6Validation contains a multi-domain three-phase of the PSSE CIM family of induction motor models, which drives a mechanical load.
The three-phase induction motor CIM5orCIM6 is based on the non multi-domain model from <i>Siemens PTI, PSS®e 34.2.0 model library, Siemens Power Technologies International, Schenectady, NY (2017). </i>.<\\p>
<p>In order to validate the multi-domain model, this validation example reproduces Example 11.7 Motor Startup from <i>Chow, Joe H., and Juan J. Sanchez-Gasca. Power system modeling, computation, and control. John Wiley & Sons, 2020.</i>  </p>
<p>The startup example from the book utilizes a <strong>three-phase type III induction motor </strong>. In this particular Modelica validation example, the idea is to test the more detailed model induction motor from the CIM family to compare results.</p>
<p>Simulate the system for 10 seconds. Variables of interest are:</p>
<ul>
<li><code>CIM.s</code></li>
<li><code>CIM.wr</code></li>
<li><code>CIM.P</code></li>
<li><code>CIM.Q</code></li>
<li><code>CIM.Pmotor</code></li>
</ul>
</html>"));
end ValidationMultiDomainCIM5andCIM6;
