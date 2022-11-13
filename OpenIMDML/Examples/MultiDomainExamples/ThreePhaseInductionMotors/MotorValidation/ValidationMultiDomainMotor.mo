within OpenIMDML.Examples.MultiDomainExamples.ThreePhaseInductionMotors.MotorValidation;
model ValidationMultiDomainMotor
  extends Modelica.Icons.Example;
  extends OpenIMDML.Examples.BaseClasses.ValidationPartial2;

  OpenIPSL.Types.PerUnit Torq;
  Modelica.Units.SI.AngularVelocity sync_speed;

  Modelica.Mechanics.Rotational.Sensors.TorqueSensor Torque_Sensor
    annotation (Placement(transformation(extent={{82,-10},{102,10}})));
  Modelica.Mechanics.Rotational.Components.Inertia Load_Inertia(J=1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={122,0})));
  Modelica.Mechanics.Rotational.Sources.Torque Torque
    annotation (Placement(transformation(extent={{112,-50},{132,-30}})));
  Modelica.Blocks.Sources.RealExpression Torque_Equation(y=Torq)
    annotation (Placement(transformation(extent={{72,-50},{92,-30}})));
  Modelica.Blocks.Sources.RealExpression Synchronous_Speed(y=sync_speed)
    annotation (Placement(transformation(extent={{20,-46},{40,-26}})));
  MultiDomainModels.Motors.MD_ALL_IN_ONE_ThreePhaseMotor motor(M_b=15000000,
      redeclare OpenIMDML.MultiDomainModels.Motors.ThreePhase.PSAT.MotorTypeIII
      motor) annotation (Placement(transformation(extent={{48,-10},{68,10}})));
equation

  Torq = -(0.1*(15/100)*(motor.motor.s) + 0.5*(15/100)*(1 - motor.motor.s)^2)*motor.motor.T_b;
  sync_speed = 2*Modelica.Constants.pi*SysData.fn;

  connect(Torque_Sensor.flange_b, Load_Inertia.flange_a)
    annotation (Line(points={{102,0},{112,0}},
                                             color={0,0,0}));
  connect(Torque.flange,Load_Inertia. flange_b) annotation (Line(points={{132,-40},
          {142,-40},{142,0},{132,0}}, color={0,0,0}));
  connect(Torque_Equation.y, Torque.tau)
    annotation (Line(points={{93,-40},{110,-40}},color={0,0,127}));
  connect(motor.flange1, Torque_Sensor.flange_a)
    annotation (Line(points={{68,0},{82,0}}, color={0,0,0}));
  connect(Torque_Sensor.tau, motor.mech_torque) annotation (Line(points={{84,-11},
          {84,-20},{64,-20},{64,-12}},          color={0,0,127}));
  connect(Synchronous_Speed.y, motor.we) annotation (Line(points={{41,-36},{58,-36},
          {58,-12}},          color={0,0,127}));
  connect(motor.pwpin, bus4_mt1.p)
    annotation (Line(points={{48,0},{20,0}}, color={0,0,255}));
  annotation (preferredView = "info", Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-100},
            {160,100}})), Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-160,-100},{160,100}})),
    experiment(
      StopTime=5,
      __Dymola_NumberOfIntervals=10000,
      Tolerance=1e-05,
      __Dymola_Algorithm="Dassl"),
    Documentation(info="<html>
<p>The example MultiDomainMotorValidation contains an <strong>ALL-IN-ONE</strong> multi-domain three-phase that allows the user to select any three-phase multi-domain motor from the library.
The all-in-one induction motor model contains a replaceable model, which is the BaseMultiDOmainThreePhase base class .<\\p>
<p>In order to validate the multi-domain model, this validation example reproduces Example 11.7 Motor Startup from <i>Chow, Joe H., and Juan J. Sanchez-Gasca. Power system modeling, computation, and control. John Wiley & Sons, 2020.</i>  </p>
<p>The startup example from the book utilizes a <strong>three-phase type III induction motor </strong>. In this particular Modelica validation example, the idea is to test the generic motor model and its object-oriented modularity.</p>
<p>Simulate the system for 10 seconds. Variables of interest are:</p>
<ul>
<li><code>motor.motor.s</code></li>
<li><code>motor.motor.wr</code></li>
<li><code>motor.motor.P</code></li>
<li><code>motor.motor.Q</code></li>
<li><code>motor.motor.Pmotor</code></li>
</ul>
</html>"));
end ValidationMultiDomainMotor;
