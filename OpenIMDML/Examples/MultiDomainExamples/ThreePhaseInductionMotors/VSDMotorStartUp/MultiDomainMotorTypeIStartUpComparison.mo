within OpenIMDML.Examples.MultiDomainExamples.ThreePhaseInductionMotors.VSDMotorStartUp;
model MultiDomainMotorTypeIStartUpComparison "Validation example for the variable speed drive driven type I motor"
  extends Modelica.Icons.Example;
  extends OpenIMDML.Examples.BaseClasses.ValidationPartial3;
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor1
    annotation (Placement(transformation(extent={{92,40},{112,60}})));
  Modelica.Mechanics.Rotational.Components.Inertia load_inertia1(J=1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={130,50})));
  Modelica.Mechanics.Rotational.Sources.Torque torque1
    annotation (Placement(transformation(extent={{120,-10},{140,10}})));
  MultiDomainModels.Motors.ThreePhase.PSAT.MotorTypeI Motor1_with_VSD(V_b=23000)
    annotation (Placement(transformation(extent={{80,40},{60,60}})));
  Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC AC2DC_and_DC2AC(V_b=23000,
      v_0=1) annotation (Placement(transformation(extent={{28,40},{48,60}})));
  Controls.VariableSpeedDrive.Controls.VoltsHertz_Controller VfController(
    V_b=23000,
    f_max=60,
    f_min=0,
    VSDstart=0.001,
    Kp=1) annotation (Placement(transformation(extent={{28,-10},{46,10}})));
  Modelica.Blocks.Sources.Ramp Sync_Speed(
    height=(2*Modelica.Constants.pi*SysData.fn),
    duration=40,
    offset=0.001*(2*Modelica.Constants.pi*SysData.fn))
    annotation (Placement(transformation(extent={{-6,-10},{14,10}})));
  Modelica.Blocks.Sources.RealExpression TorqueEquation(y=-(0.1*(15/100)*(
        Motor1_with_VSD.s) + 0.5*(15/100)*(1 - Motor1_with_VSD.s)^2)*
        Motor1_with_VSD.T_b)
    annotation (Placement(transformation(extent={{88,-10},{108,10}})));
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor2
    annotation (Placement(transformation(extent={{90,-60},{110,-40}})));
  Modelica.Mechanics.Rotational.Components.Inertia load_inertia2(J=1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={130,-50})));
  Modelica.Mechanics.Rotational.Sources.Torque torque2
    annotation (Placement(transformation(extent={{120,-100},{140,-80}})));
  Modelica.Blocks.Sources.RealExpression Torque_Equation(y=-(0.1*(15/100)*(
        Motor1.s) + 0.5*(15/100)*(1 - Motor1.s)^2)*Motor1.T_b)
    annotation (Placement(transformation(extent={{80,-100},{100,-80}})));
  Modelica.Blocks.Sources.RealExpression SS1(y=2*Modelica.Constants.pi*(SysData.fn))
                annotation (Placement(transformation(extent={{20,-86},{40,-66}})));
  MultiDomainModels.Motors.ThreePhase.PSAT.MotorTypeI Motor1(V_b=23000)
    annotation (Placement(transformation(extent={{80,-60},{60,-40}})));
equation
  connect(torqueSensor1.flange_b,load_inertia1. flange_a)
    annotation (Line(points={{112,50},{120,50}}, color={0,0,0}));
  connect(torque1.flange,load_inertia1. flange_b) annotation (Line(points={{140,0},
          {150,0},{150,50},{140,50}},      color={0,0,0}));
  connect(Motor1_with_VSD.flange, torqueSensor1.flange_a)
    annotation (Line(points={{80,50},{92,50}}, color={0,0,0}));
  connect(torqueSensor1.tau, Motor1_with_VSD.mech_torque) annotation (Line(
        points={{94,39},{94,32},{76,32},{76,38}}, color={0,0,127}));
  connect(AC2DC_and_DC2AC.n, Motor1_with_VSD.p)
    annotation (Line(points={{48,50},{60,50}}, color={0,0,255}));
  connect(VfController.we, Motor1_with_VSD.we)
    annotation (Line(points={{50,-4},{70,-4},{70,38}}, color={0,0,127}));
  connect(Motor1_with_VSD.wr, VfController.motor_speed)
    annotation (Line(points={{64,38},{64,4},{50,4}}, color={0,0,127}));
  connect(AC2DC_and_DC2AC.Vc, VfController.Vc)
    annotation (Line(points={{33.4545,38},{33.4,12}}, color={0,0,127}));
  connect(VfController.m, AC2DC_and_DC2AC.m_input) annotation (Line(points={{42.6,12},
          {42.5455,12},{42.5455,38}},     color={0,0,127}));
  connect(Sync_Speed.y, VfController.W_ref)
    annotation (Line(points={{15,0},{26,0}}, color={0,0,127}));
  connect(TorqueEquation.y, torque1.tau)
    annotation (Line(points={{109,0},{118,0}}, color={0,0,127}));
  connect(bus4_mt1.p, AC2DC_and_DC2AC.p)
    annotation (Line(points={{20,50},{28,50}}, color={0,0,255}));
  connect(torqueSensor2.flange_b,load_inertia2. flange_a)
    annotation (Line(points={{110,-50},{120,-50}},
                                                 color={0,0,0}));
  connect(torque2.flange,load_inertia2. flange_b) annotation (Line(points={{140,-90},
          {150,-90},{150,-50},{140,-50}},  color={0,0,0}));
  connect(Torque_Equation.y, torque2.tau)
    annotation (Line(points={{101,-90},{118,-90}}, color={0,0,127}));
  connect(Motor1.flange,torqueSensor2. flange_a)
    annotation (Line(points={{80,-50},{90,-50}},
                                             color={0,0,0}));
  connect(torqueSensor2.tau,Motor1. mech_torque) annotation (Line(points={{92,-61},
          {92,-68},{76,-68},{76,-62}}, color={0,0,127}));
  connect(SS1.y,Motor1. we)
    annotation (Line(points={{41,-76},{70,-76},{70,-62}}, color={0,0,127}));
  connect(bus4_mt2.p, Motor1.p)
    annotation (Line(points={{20,-50},{60,-50}}, color={0,0,255}));
  annotation (preferredView = "info",Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-120},
            {160,120}})), Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-160,-120},{160,120}})),
    experiment(
      StopTime=50,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"),
    Documentation(info="<html>
<p>The example MultiDomainMotorTypeIStartUpComparison contains two separate systems meant for comparison. The top system is a multi-domain three-phase type I induction motor model controlled by a Variable Speed Drive while the bottom system is the same example but with no VSD controller.
The purpose of this example is to compare the startup current magnitude of both systems and to point out one of the major benefits of using a VSD in the startup process. Starting an induction motor from a halt condition
generates a peak starting current in the motor which is detrimental to the stability of the electrical system and to the magnitization coils of the machine itself. 
The user can compare both startup currents in one example and confirm that the startup current magnitude in the VSD example is substantially smaller in comparison to the non VSD example.</p>
 
<p>The Volts/Hertz scalar variable speed drive model is based on paper <i>Del Rosso, A. D., Mariano Anello, and E. Spittle. \"Stability Assessment of Isolated Power Systems with Large Induction Motor Drives.\" 2006 IEEE/PES Transmission & Distribution Conference and Exposition: Latin America. IEEE, 2006. </i><\\p>
<p>Simulate the system for 50 seconds. Variables of interest and comparison are:</p>
<ul>
<li><code>Motor1.Imag vs Motor1_with_VSD.Imag</code></li>
<li><code>Motor1.s vs Motor1_with_VSD.s</code></li>
<li><code>Motor1.wr vs Motor1_with_VSD.wr</code></li>
<li><code>Motor1.P vs Motor1_with_VSD.P</code></li>
<li><code>Motor1.Q vs Motor1_with_VSD.Q</code></li>
<li><code>VfController.m</code></li>
<li><code>VfController.we</code></li>
</ul>
</html>"));
end MultiDomainMotorTypeIStartUpComparison;
