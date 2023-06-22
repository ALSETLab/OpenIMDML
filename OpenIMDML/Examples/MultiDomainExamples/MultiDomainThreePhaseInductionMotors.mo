within OpenIMDML.Examples.MultiDomainExamples;
package MultiDomainThreePhaseInductionMotors
  "Simple systems to test out the multi-domain three-phase motor models"
  extends Modelica.Icons.ExamplesPackage;

  package MultiDomainThreePhaseMotorValidation
    "Multi-domain three-phase motor validation examples"
    extends Modelica.Icons.ExamplesPackage;

    model MultiDomainTypeI
      "Multi-Domain validation example for the Type I motor model"
      extends Modelica.Icons.Example;
      extends OpenIMDML.Examples.BaseClasses.ValidationPartial2(inf1(X_d=0));
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
      MultiDomainModels.Motors.ThreePhase.PSAT.MD_MotorTypeI Motor1(V_b=23000)
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
          StopTime=5,
          __Dymola_NumberOfIntervals=5000,
          __Dymola_Algorithm="Dassl"),
        Documentation(info="<html>
<p>The example MultiDomainTypeI contains a multi-domain three-phase type I induction motor model which drives a mechanical load. 
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
    end MultiDomainTypeI;

    model MultiDomainTypeIII
      "Multi-Domain validation example for the Type III motor model"
      extends Modelica.Icons.Example;
      extends OpenIMDML.Examples.BaseClasses.ValidationPartial2(inf1(X_d=0));
      Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor3
        annotation (Placement(transformation(extent={{82,-10},{102,10}})));
      Modelica.Mechanics.Rotational.Sources.Torque torque3
        annotation (Placement(transformation(extent={{112,-50},{132,-30}})));
      Modelica.Mechanics.Rotational.Components.Inertia load_inertia3(J=1)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={122,0})));
      Modelica.Blocks.Sources.RealExpression Torque3(y=-(0.1*(15/100)*(Motor3.s) + 0.5
            *(15/100)*(1 - Motor3.s)^2)*Motor3.T_b)
        annotation (Placement(transformation(extent={{72,-50},{92,-30}})));
      Modelica.Blocks.Sources.RealExpression SS3(y=2*Modelica.Constants.pi*(
            SysData.fn))
                    annotation (Placement(transformation(extent={{20,-46},{40,
                -26}})));
      MultiDomainModels.Motors.ThreePhase.PSAT.MD_MotorTypeIII Motor3(V_b=23000)
        annotation (Placement(transformation(extent={{68,-10},{48,10}})));
    equation
      connect(torqueSensor3.flange_b,load_inertia3. flange_a)
        annotation (Line(points={{102,0},{112,0}}, color={0,0,0}));
      connect(torque3.flange,load_inertia3. flange_b) annotation (Line(points={{132,-40},
              {142,-40},{142,0},{132,0}},      color={0,0,0}));
      connect(Torque3.y,torque3. tau)
        annotation (Line(points={{93,-40},{110,-40}},  color={0,0,127}));
      connect(torqueSensor3.tau, Motor3.mech_torque) annotation (Line(points={{84,-11},
              {84,-20},{64,-20},{64,-12}}, color={0,0,127}));
      connect(SS3.y, Motor3.we)
        annotation (Line(points={{41,-36},{58,-36},{58,-12}}, color={0,0,127}));
      connect(Motor3.flange, torqueSensor3.flange_a)
        annotation (Line(points={{68,0},{82,0}}, color={0,0,0}));
      connect(bus4_mt1.p, Motor3.p)
        annotation (Line(points={{20,0},{48,0}}, color={0,0,255}));
      annotation (preferredView = "info",Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
                -100},{160,100}})),                                  Diagram(
            coordinateSystem(preserveAspectRatio=false, extent={{-160,-100},{
                160,100}})),
        experiment(
          StopTime=5,
          __Dymola_NumberOfIntervals=5000,
          __Dymola_Algorithm="Dassl"),
        Documentation(info="<html>
<p>The example MultiDomainTypeIII contains a multi-domain three-phase type III induction motor model which drives a mechanical load.
The three-phase induction motor type III is based on the non multi-domain model from <i>Milano, Federico. Power system modelling and scripting. Springer Science & Business Media, 2010.<\\p>
<p>In order to validate the multi-domain model, this validation example reproduces Example 11.7 Motor Startup from <i>Chow, Joe H., and Juan J. Sanchez-Gasca. Power system modeling, computation, and control. John Wiley & Sons, 2020.</i></p>
<p>The startup example from the book utilizes a <strong>three-phase type III induction motor </strong>. In this particular Modelica validation example, the idea is to test the model type III to compare results.</p>
<p>Simulate the system for 10 seconds. Variables of interest are:</p>
<ul>
<li><code>Motor3.s</code></li>
<li><code>Motor3.wr</code></li>
<li><code>Motor3.P</code></li>
<li><code>Motor3.Q</code></li>
<li><code>Motor3.Pmotor</code></li>
</ul>
</html>"));
    end MultiDomainTypeIII;

    model MultiDomainTypeV
      "Multi-Domain validation example for the Type V motor model"
      extends Modelica.Icons.Example;
      extends OpenIMDML.Examples.BaseClasses.ValidationPartial2(inf1(X_d=0));
      Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor5
        annotation (Placement(transformation(extent={{82,-10},{102,10}})));
      Modelica.Mechanics.Rotational.Sources.Torque torque5
        annotation (Placement(transformation(extent={{112,-50},{132,-30}})));
      Modelica.Mechanics.Rotational.Components.Inertia load_inertia5(J=1)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={122,0})));
      Modelica.Blocks.Sources.RealExpression Torque5(y=-(0.1*(15/100)*(Motor5.s) +
            0.5*(15/100)*(1 - Motor5.s)^2)*Motor5.T_b)
        annotation (Placement(transformation(extent={{72,-50},{92,-30}})));
      Modelica.Blocks.Sources.RealExpression SS5(y=2*Modelica.Constants.pi*(
            SysData.fn))
        annotation (Placement(transformation(extent={{20,-46},{40,-26}})));
      MultiDomainModels.Motors.ThreePhase.PSAT.MD_MotorTypeV Motor5(V_b=23000)
        annotation (Placement(transformation(extent={{68,-10},{48,10}})));
    equation
      connect(torqueSensor5.flange_b,load_inertia5. flange_a)
        annotation (Line(points={{102,0},{112,0}}, color={0,0,0}));
      connect(torque5.flange,load_inertia5. flange_b) annotation (Line(points={{132,-40},
              {142,-40},{142,0},{132,0}},      color={0,0,0}));
      connect(Torque5.y,torque5. tau)
        annotation (Line(points={{93,-40},{110,-40}},  color={0,0,127}));
      connect(Motor5.flange, torqueSensor5.flange_a)
        annotation (Line(points={{68,0},{82,0}}, color={0,0,0}));
      connect(torqueSensor5.tau, Motor5.mech_torque) annotation (Line(points={{84,
              -11},{84,-20},{64,-20},{64,-12}}, color={0,0,127}));
      connect(SS5.y, Motor5.we)
        annotation (Line(points={{41,-36},{58,-36},{58,-12}}, color={0,0,127}));
      connect(bus4_mt1.p, Motor5.p)
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
<p>The example MultiDomainTypeV contains a multi-domain three-phase type V induction motor model which drives a mechanical load.
The three-phase induction motor type V is based on the non multi-domain model from <i>Milano, Federico. Power system modelling and scripting. Springer Science & Business Media, 2010.<\\p>
<p>In order to validate the multi-domain model, this validation example reproduces Example 11.7 Motor Startup from <i>Chow, Joe H., and Juan J. Sanchez-Gasca. Power system modeling, computation, and control. John Wiley & Sons, 2020.</i>  </p>
<p>The startup example from the book utilizes a <strong>three-phase type III induction motor </strong>. In this particular Modelica validation example, the idea is to test the more detailed model type V to compare results.</p>
<p>Simulate the system for 10 seconds. Variables of interest are:</p>
<ul>
<li><code>Motor5.s</code></li>
<li><code>Motor5.wr</code></li>
<li><code>Motor5.P</code></li>
<li><code>Motor5.Q</code></li>
<li><code>Motor5.Pmotor</code></li>
</ul>
</html>"));
    end MultiDomainTypeV;

    model MultiDomainCIM
      "Multi-Domain validation example for the CIM family motor model"
      extends Modelica.Icons.Example;
      extends OpenIMDML.Examples.BaseClasses.ValidationPartial2(inf1(X_d=0));
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
      MultiDomainModels.Motors.ThreePhase.PSSE.MD_CIM CIM(V_b=23000)
        annotation (Placement(transformation(extent={{68,-10},{48,10}})));
    equation
      connect(torqueSensorCIM5.flange_b,load_inertiaCIM5. flange_a)
        annotation (Line(points={{102,0},{112,0}},       color={0,0,0}));
      connect(torqueCIM5.flange,load_inertiaCIM5. flange_b) annotation (Line(points={{132,-40},
              {142,-40},{142,0},{132,0}},                   color={0,0,0}));
      connect(TorqueCIM5.y,torqueCIM5. tau)
        annotation (Line(points={{93,-40},{110,-40}},    color={0,0,127}));
      connect(torqueSensorCIM5.tau, CIM.mech_torque) annotation (Line(points={{84,
              -11},{84,-20},{64,-20},{64,-12}}, color={0,0,127}));
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
<p>The example MultiDomainCIM5andCIM6 contains a multi-domain three-phase of the PSSE CIM family of induction motor models, which drives a mechanical load.
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
    end MultiDomainCIM;

    model MultiDomainMotor
      "Multi-Domain validation example for the ALL-IN-ONE motor model"
      extends Modelica.Icons.Example;
      extends OpenIMDML.Examples.BaseClasses.ValidationPartial2(inf1(X_d=0));

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
          redeclare
          OpenIMDML.MultiDomainModels.Motors.ThreePhase.PSAT.MD_MotorTypeIII
          motor)
        annotation (Placement(transformation(extent={{48,-10},{68,10}})));
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
<p>The example MultiDomainMotor contains an <strong>ALL-IN-ONE</strong> multi-domain three-phase that allows the user to select any three-phase multi-domain motor from the library.
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
    end MultiDomainMotor;
  end MultiDomainThreePhaseMotorValidation;

  package MultiDomainThreePhaseMotorVSDStartup
    "Simple systems to test out the variable speed drive model"
    extends Modelica.Icons.ExamplesPackage;

    model MultiDomainMotorTypeIStartUpComparison "Validation example for the variable speed drive driven type I motor"
      extends Modelica.Icons.Example;
      extends OpenIMDML.Examples.BaseClasses.ValidationPartial3;
      parameter Real Ro = 0.1;
      Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor1
        annotation (Placement(transformation(extent={{92,40},{112,60}})));
      Modelica.Mechanics.Rotational.Components.Inertia load_inertia1(J=1)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={130,50})));
      Modelica.Mechanics.Rotational.Sources.Torque torque1
        annotation (Placement(transformation(extent={{120,-10},{140,10}})));
      MultiDomainModels.Motors.ThreePhase.PSAT.MD_MotorTypeI Motor1VSD(V_b=23000)
        annotation (Placement(transformation(extent={{80,40},{60,60}})));
      Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC AC2DC_and_DC2AC(V_b=23000,
          v_0=1) annotation (Placement(transformation(extent={{28,40},{48,60}})));
      Controls.VariableSpeedDrive.Controls.VoltsHertz_Controller VfController(
        V_b=23000,
        f_max=60,
        f_min=0,
        VSDstart=0.1,
        Kp=1,
        Ki=1) annotation (Placement(transformation(extent={{28,-10},{46,10}})));
      Modelica.Blocks.Sources.Ramp Sync_Speed(
        height=0.9*(2*Modelica.Constants.pi*SysData.fn),
        duration=5,
        offset=0.1*(2*Modelica.Constants.pi*SysData.fn))
        annotation (Placement(transformation(extent={{-6,-10},{14,10}})));
      Modelica.Blocks.Sources.RealExpression TorqueEquation(y=-(0.1*(15/100)*(
            Motor1VSD.s) + 0.5*(15/100)*(1 - Motor1VSD.s)^2)*Motor1VSD.T_b)
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
      MultiDomainModels.Motors.ThreePhase.PSAT.MD_MotorTypeI Motor1(V_b=23000)
        annotation (Placement(transformation(extent={{80,-60},{60,-40}})));
    equation
      connect(torqueSensor1.flange_b,load_inertia1. flange_a)
        annotation (Line(points={{112,50},{120,50}}, color={0,0,0}));
      connect(torque1.flange,load_inertia1. flange_b) annotation (Line(points={{140,0},
              {150,0},{150,50},{140,50}},      color={0,0,0}));
      connect(Motor1VSD.flange, torqueSensor1.flange_a)
        annotation (Line(points={{80,50},{92,50}}, color={0,0,0}));
      connect(torqueSensor1.tau, Motor1VSD.mech_torque) annotation (Line(points={{94,
              39},{94,32},{76,32},{76,38}}, color={0,0,127}));
      connect(AC2DC_and_DC2AC.n, Motor1VSD.p)
        annotation (Line(points={{48,50},{60,50}}, color={0,0,255}));
      connect(VfController.we, Motor1VSD.we)
        annotation (Line(points={{50,-4},{70,-4},{70,38}}, color={0,0,127}));
      connect(Motor1VSD.wr, VfController.motor_speed)
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
          StopTime=10,
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

    model MultiDomainMotorTypeIIIStartUpComparison "Validation example for the variable speed drive driven type III motor"
      extends Modelica.Icons.Example;
      extends OpenIMDML.Examples.BaseClasses.ValidationPartial3;
      parameter Real Ro = 0.1;
      Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC AC2DC_and_DC2AC(V_b=23000,
          v_0=1) annotation (Placement(transformation(extent={{28,40},{48,60}})));
      Controls.VariableSpeedDrive.Controls.VoltsHertz_Controller VfController(
        V_b=23000,
        f_max=60,
        f_min=0,
        VSDstart=0.05,
        Kp=2,
        Ki=1)
        annotation (Placement(transformation(extent={{28,-10},{46,10}})));
      MultiDomainModels.Motors.ThreePhase.PSAT.MD_MotorTypeIII Motor3VSD(V_b=23000)
        annotation (Placement(transformation(extent={{80,40},{60,60}})));
      Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor3
        annotation (Placement(transformation(extent={{92,40},{112,60}})));
      Modelica.Mechanics.Rotational.Sources.Torque torque3
        annotation (Placement(transformation(extent={{120,-10},{140,10}})));
      Modelica.Mechanics.Rotational.Components.Inertia load_inertia3(J=100)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={130,50})));
      Modelica.Blocks.Sources.RealExpression TorqueEquation(y=-(0.1*(15/100)*(
            Motor3VSD.s) + 0.5*(15/100)*(1 - Motor3VSD.s)^2)*Motor3VSD.T_b)
        annotation (Placement(transformation(extent={{88,-10},{108,10}})));
      Modelica.Blocks.Sources.Ramp Sync_Speed(
        height=0.95*(2*Modelica.Constants.pi*SysData.fn),
        duration=5,
        offset=0.05*(2*Modelica.Constants.pi*SysData.fn))
        annotation (Placement(transformation(extent={{-6,-10},{14,10}})));
      Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor1
        annotation (Placement(transformation(extent={{90,-60},{110,-40}})));
      Modelica.Mechanics.Rotational.Sources.Torque torque1
        annotation (Placement(transformation(extent={{120,-100},{140,-80}})));
      Modelica.Mechanics.Rotational.Components.Inertia load_inertia1(J=1)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={130,-50})));
      Modelica.Blocks.Sources.RealExpression Torque_Equation(y=-(0.1*(15/100)*(
            Motor3.s) + 0.5*(15/100)*(1 - Motor3.s)^2)*Motor3.T_b)
        annotation (Placement(transformation(extent={{80,-100},{100,-80}})));
      MultiDomainModels.Motors.ThreePhase.PSAT.MD_MotorTypeIII Motor3(V_b=23000)
        annotation (Placement(transformation(extent={{80,-60},{60,-40}})));
      Modelica.Blocks.Sources.RealExpression SS3(y=2*Modelica.Constants.pi*(SysData.fn))
                    annotation (Placement(transformation(extent={{20,-86},{40,-66}})));
    equation
      connect(AC2DC_and_DC2AC.Vc, VfController.Vc)
        annotation (Line(points={{33.4545,38},{33.4,12}}, color={0,0,127}));
      connect(VfController.m, AC2DC_and_DC2AC.m_input) annotation (Line(points={{42.6,12},
              {42.5455,12},{42.5455,38}},     color={0,0,127}));
      connect(AC2DC_and_DC2AC.n, Motor3VSD.p)
        annotation (Line(points={{48,50},{60,50}}, color={0,0,255}));
      connect(VfController.we, Motor3VSD.we)
        annotation (Line(points={{50,-4},{70,-4},{70,38}}, color={0,0,127}));
      connect(Motor3VSD.wr, VfController.motor_speed)
        annotation (Line(points={{64,38},{64,4},{50,4}}, color={0,0,127}));
      connect(torqueSensor3.flange_b,load_inertia3. flange_a)
        annotation (Line(points={{112,50},{120,50}},
                                                   color={0,0,0}));
      connect(torque3.flange,load_inertia3. flange_b) annotation (Line(points={{140,0},
              {150,0},{150,50},{140,50}},      color={0,0,0}));
      connect(TorqueEquation.y, torque3.tau)
        annotation (Line(points={{109,0},{118,0}}, color={0,0,127}));
      connect(Motor3VSD.flange, torqueSensor3.flange_a)
        annotation (Line(points={{80,50},{92,50}}, color={0,0,0}));
      connect(torqueSensor3.tau, Motor3VSD.mech_torque) annotation (Line(points={{94,
              39},{94,32},{76,32},{76,38}}, color={0,0,127}));
      connect(bus4_mt1.p, AC2DC_and_DC2AC.p)
        annotation (Line(points={{20,50},{28,50}}, color={0,0,255}));
      connect(Sync_Speed.y, VfController.W_ref)
        annotation (Line(points={{15,0},{26,0}}, color={0,0,127}));
      connect(torqueSensor1.flange_b,load_inertia1. flange_a)
        annotation (Line(points={{110,-50},{120,-50}},
                                                   color={0,0,0}));
      connect(torque1.flange,load_inertia1. flange_b) annotation (Line(points={{140,-90},
              {150,-90},{150,-50},{140,-50}},  color={0,0,0}));
      connect(Torque_Equation.y, torque1.tau)
        annotation (Line(points={{101,-90},{118,-90}}, color={0,0,127}));
      connect(torqueSensor1.tau,Motor3. mech_torque) annotation (Line(points={{92,-61},
              {92,-68},{76,-68},{76,-62}}, color={0,0,127}));
      connect(SS3.y,Motor3. we)
        annotation (Line(points={{41,-76},{70,-76},{70,-62}}, color={0,0,127}));
      connect(Motor3.flange,torqueSensor1. flange_a)
        annotation (Line(points={{80,-50},{90,-50}},
                                                 color={0,0,0}));
      connect(bus4_mt2.p, Motor3.p)
        annotation (Line(points={{20,-50},{60,-50}}, color={0,0,255}));
      annotation (preferredView = "info",Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-120},
                {160,120}})), Diagram(coordinateSystem(preserveAspectRatio=false,
              extent={{-160,-120},{160,120}})),
        experiment(
          StopTime=10,
          __Dymola_NumberOfIntervals=5000,
          __Dymola_Algorithm="Dassl"),
        Documentation(info="<html>
<p>The example MultiDomainMotorTypeIIIStartUpComparison contains two separate systems meant for comparison. The top system is a multi-domain three-phase type III induction motor model controlled by a Variable Speed Drive while the bottom system is the same example but with no VSD controller.
The purpose of this example is to compare the startup current magnitude of both systems and to point out one of the major benefits of using a VSD in the startup process. Starting an induction motor from a halt condition
generates a peak starting current in the motor which is detrimental to the stability of the electrical system and to the magnitization coils of the machine itself. 
The user can compare both startup currents in one example and confirm that the startup current magnitude in the VSD example is substantially smaller in comparison to the non VSD example.</p>
 
<p>The Volts/Hertz scalar variable speed drive model is based on paper <i>Del Rosso, A. D., Mariano Anello, and E. Spittle. \"Stability Assessment of Isolated Power Systems with Large Induction Motor Drives.\" 2006 IEEE/PES Transmission & Distribution Conference and Exposition: Latin America. IEEE, 2006. </i><\\p>
<p>Simulate the system for 50 seconds. Variables of interest and comparison are:</p>
<ul>
<li><code>Motor3.Imag vs Motor3_with_VSD.Imag</code></li>
<li><code>Motor3.s vs Motor3_with_VSD.s</code></li>
<li><code>Motor3.wr vs Motor3_with_VSD.wr</code></li>
<li><code>Motor3.P vs Motor3_with_VSD.P</code></li>
<li><code>Motor3.Q vs Motor3_with_VSD.Q</code></li>
<li><code>VfController.m</code></li>
<li><code>VfController.we</code></li>
</ul>
</html>"));
    end MultiDomainMotorTypeIIIStartUpComparison;

    model MultiDomainMotorTypeVStartUpComparison "Validation example for the variable speed drive driven type V motor"
      extends Modelica.Icons.Example;
      extends OpenIMDML.Examples.BaseClasses.ValidationPartial3;
      parameter Real Ro = 0.1;
      Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC AC2DC_and_DC2AC(V_b=23000,
          v_0=1) annotation (Placement(transformation(extent={{28,40},{48,60}})));
      Controls.VariableSpeedDrive.Controls.VoltsHertz_Controller VfController(
        V_b=23000,
        f_max=60,
        f_min=0,
        VSDstart=0.1,
        Kp=1,
        Ki=0.01)
        annotation (Placement(transformation(extent={{28,-10},{46,10}})));
      Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor3
        annotation (Placement(transformation(extent={{92,40},{112,60}})));
      Modelica.Mechanics.Rotational.Sources.Torque torque3
        annotation (Placement(transformation(extent={{120,-10},{140,10}})));
      Modelica.Mechanics.Rotational.Components.Inertia load_inertia3(J=1)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={130,50})));
      Modelica.Blocks.Sources.RealExpression TorqueEquation(y=-(0.1*(15/100)*(
            Motor5VSD.s) + 0.5*(15/100)*(1 - Motor5VSD.s)^2)*Motor5VSD.T_b)
        annotation (Placement(transformation(extent={{88,-10},{108,10}})));
      Modelica.Blocks.Sources.Ramp Sync_Speed(
        height=0.9*(2*Modelica.Constants.pi*SysData.fn),
        duration=5,
        offset=0.1*(2*Modelica.Constants.pi*SysData.fn))
        annotation (Placement(transformation(extent={{-6,-10},{14,10}})));
      Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor1
        annotation (Placement(transformation(extent={{90,-60},{110,-40}})));
      Modelica.Mechanics.Rotational.Sources.Torque torque1
        annotation (Placement(transformation(extent={{120,-100},{140,-80}})));
      Modelica.Mechanics.Rotational.Components.Inertia load_inertia1(J=1)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={130,-50})));
      Modelica.Blocks.Sources.RealExpression Torque_Equation(y=-(0.1*(15/100)*(
            Motor5.s) + 0.5*(15/100)*(1 - Motor5.s)^2)*Motor5.T_b)
        annotation (Placement(transformation(extent={{80,-100},{100,-80}})));
      Modelica.Blocks.Sources.RealExpression SS3(y=2*Modelica.Constants.pi*(SysData.fn))
                    annotation (Placement(transformation(extent={{20,-86},{40,-66}})));
      MultiDomainModels.Motors.ThreePhase.PSAT.MD_MotorTypeV Motor5VSD(V_b=23000)
        annotation (Placement(transformation(extent={{80,40},{60,60}})));
      MultiDomainModels.Motors.ThreePhase.PSAT.MD_MotorTypeV Motor5(V_b=23000)
        annotation (Placement(transformation(extent={{80,-60},{60,-40}})));
    equation
      connect(AC2DC_and_DC2AC.Vc, VfController.Vc)
        annotation (Line(points={{33.4545,38},{33.4,12}}, color={0,0,127}));
      connect(VfController.m, AC2DC_and_DC2AC.m_input) annotation (Line(points={{42.6,12},
              {42.5455,12},{42.5455,38}},     color={0,0,127}));
      connect(torqueSensor3.flange_b,load_inertia3. flange_a)
        annotation (Line(points={{112,50},{120,50}},
                                                   color={0,0,0}));
      connect(torque3.flange,load_inertia3. flange_b) annotation (Line(points={{140,0},
              {150,0},{150,50},{140,50}},      color={0,0,0}));
      connect(TorqueEquation.y, torque3.tau)
        annotation (Line(points={{109,0},{118,0}}, color={0,0,127}));
      connect(bus4_mt1.p, AC2DC_and_DC2AC.p)
        annotation (Line(points={{20,50},{28,50}}, color={0,0,255}));
      connect(Sync_Speed.y, VfController.W_ref)
        annotation (Line(points={{15,0},{26,0}}, color={0,0,127}));
      connect(torqueSensor1.flange_b,load_inertia1. flange_a)
        annotation (Line(points={{110,-50},{120,-50}},
                                                   color={0,0,0}));
      connect(torque1.flange,load_inertia1. flange_b) annotation (Line(points={{140,-90},
              {150,-90},{150,-50},{140,-50}},  color={0,0,0}));
      connect(Torque_Equation.y, torque1.tau)
        annotation (Line(points={{101,-90},{118,-90}}, color={0,0,127}));
      connect(AC2DC_and_DC2AC.n, Motor5VSD.p)
        annotation (Line(points={{48,50},{60,50}}, color={0,0,255}));
      connect(torqueSensor3.tau, Motor5VSD.mech_torque) annotation (Line(points=
             {{94,39},{94,32},{76,32},{76,38}}, color={0,0,127}));
      connect(VfController.motor_speed, Motor5VSD.wr)
        annotation (Line(points={{50,4},{64,4},{64,38}}, color={0,0,127}));
      connect(VfController.we, Motor5VSD.we)
        annotation (Line(points={{50,-4},{70,-4},{70,38}}, color={0,0,127}));
      connect(Motor5VSD.flange, torqueSensor3.flange_a)
        annotation (Line(points={{80,50},{92,50}}, color={0,0,0}));
      connect(bus4_mt2.p, Motor5.p)
        annotation (Line(points={{20,-50},{60,-50}}, color={0,0,255}));
      connect(Motor5.flange, torqueSensor1.flange_a)
        annotation (Line(points={{80,-50},{90,-50}}, color={0,0,0}));
      connect(torqueSensor1.tau, Motor5.mech_torque) annotation (Line(points={{92,-61},
              {92,-68},{76,-68},{76,-62}}, color={0,0,127}));
      connect(SS3.y, Motor5.we)
        annotation (Line(points={{41,-76},{70,-76},{70,-62}}, color={0,0,127}));
      annotation (preferredView = "info",Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-120},
                {160,120}})), Diagram(coordinateSystem(preserveAspectRatio=false,
              extent={{-160,-120},{160,120}})),
        experiment(
          StopTime=10,
          __Dymola_NumberOfIntervals=5000,
          __Dymola_Algorithm="Dassl"),
        Documentation(info="<html>
<p>The example MultiDomainMotorTypeVStartUpComparison contains two separate systems meant for comparison. The top system is a multi-domain three-phase type V induction motor model controlled by a Variable Speed Drive while the bottom system is the same example but with no VSD controller.
The purpose of this example is to compare the startup current magnitude of both systems and to point out one of the major benefits of using a VSD in the startup process. Starting an induction motor from a halt condition
generates a peak starting current in the motor which is detrimental to the stability of the electrical system and to the magnitization coils of the machine itself. 
The user can compare both startup currents in one example and confirm that the startup current magnitude in the VSD example is substantially smaller in comparison to the non VSD example.</p>
 
<p>The Volts/Hertz scalar variable speed drive model is based on paper <i>Del Rosso, A. D., Mariano Anello, and E. Spittle. \"Stability Assessment of Isolated Power Systems with Large Induction Motor Drives.\" 2006 IEEE/PES Transmission & Distribution Conference and Exposition: Latin America. IEEE, 2006. </i><\\p>
<p>Simulate the system for 50 seconds. Variables of interest and comparison are:</p>
<ul>
<li><code>Motor5.Imag vs Motor5_with_VSD.Imag</code></li>
<li><code>Motor5.s vs Motor5_with_VSD.s</code></li>
<li><code>Motor5.wr vs Motor5_with_VSD.wr</code></li>
<li><code>Motor5.P vs Motor5_with_VSD.P</code></li>
<li><code>Motor5.Q vs Motor5_with_VSD.Q</code></li>
<li><code>VfController.m</code></li>
<li><code>VfController.we</code></li>
</ul>
</html>"));
    end MultiDomainMotorTypeVStartUpComparison;

    model MultiDomainMotorCIMStartUpComparison "Validation example for the variable speed drive driven CIM motor"
      extends Modelica.Icons.Example;
      extends OpenIMDML.Examples.BaseClasses.ValidationPartial3;
      parameter Real Ro = 0.1;
      Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC AC2DC_and_DC2AC(V_b=23000,
          v_0=1) annotation (Placement(transformation(extent={{28,40},{48,60}})));
      Controls.VariableSpeedDrive.Controls.VoltsHertz_Controller VfController(
        V_b=23000,
        f_max=60,
        f_min=0,
        VSDstart=0.1,
        Kp=0.1,
        Ki=0.01)
        annotation (Placement(transformation(extent={{28,-10},{46,10}})));
      Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor3
        annotation (Placement(transformation(extent={{92,40},{112,60}})));
      Modelica.Mechanics.Rotational.Sources.Torque torque3
        annotation (Placement(transformation(extent={{120,-10},{140,10}})));
      Modelica.Mechanics.Rotational.Components.Inertia load_inertia3(J=1)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={130,50})));
      Modelica.Blocks.Sources.RealExpression TorqueEquation(y=-(0.1*(15/100)*(
            CIMVSD.s) + 0.5*(15/100)*(1 - CIMVSD.s)^2)*CIMVSD.T_b)
        annotation (Placement(transformation(extent={{88,-10},{108,10}})));
      Modelica.Blocks.Sources.Ramp Sync_Speed(
        height=0.9*(2*Modelica.Constants.pi*SysData.fn),
        duration=5,
        offset=0.1*(2*Modelica.Constants.pi*SysData.fn))
        annotation (Placement(transformation(extent={{-6,-10},{14,10}})));
      Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor1
        annotation (Placement(transformation(extent={{90,-60},{110,-40}})));
      Modelica.Mechanics.Rotational.Sources.Torque torque1
        annotation (Placement(transformation(extent={{120,-100},{140,-80}})));
      Modelica.Mechanics.Rotational.Components.Inertia load_inertia1(J=1)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={130,-50})));
      Modelica.Blocks.Sources.RealExpression Torque_Equation(y=-(0.1*(15/100)*(CIM.s)
             + 0.5*(15/100)*(1 - CIM.s)^2)*CIM.T_b)
        annotation (Placement(transformation(extent={{80,-100},{100,-80}})));
      Modelica.Blocks.Sources.RealExpression SS3(y=2*Modelica.Constants.pi*(SysData.fn))
                    annotation (Placement(transformation(extent={{20,-86},{40,-66}})));
      MultiDomainModels.Motors.ThreePhase.PSSE.MD_CIM CIMVSD(
        V_b=23000,
        Mtype=2,
        E1=1,
        SE1=1,
        E2=1,
        SE2=1) annotation (Placement(transformation(extent={{80,40},{60,60}})));
      MultiDomainModels.Motors.ThreePhase.PSSE.MD_CIM CIM(
        V_b=23000,
        E1=1,
        SE1=1,
        E2=1,
        SE2=1)
        annotation (Placement(transformation(extent={{80,-60},{60,-40}})));
    equation
      connect(AC2DC_and_DC2AC.Vc, VfController.Vc)
        annotation (Line(points={{33.4545,38},{33.4,12}}, color={0,0,127}));
      connect(VfController.m, AC2DC_and_DC2AC.m_input) annotation (Line(points={{42.6,12},
              {42.5455,12},{42.5455,38}},     color={0,0,127}));
      connect(torqueSensor3.flange_b,load_inertia3. flange_a)
        annotation (Line(points={{112,50},{120,50}},
                                                   color={0,0,0}));
      connect(torque3.flange,load_inertia3. flange_b) annotation (Line(points={{140,0},
              {150,0},{150,50},{140,50}},      color={0,0,0}));
      connect(TorqueEquation.y, torque3.tau)
        annotation (Line(points={{109,0},{118,0}}, color={0,0,127}));
      connect(bus4_mt1.p, AC2DC_and_DC2AC.p)
        annotation (Line(points={{20,50},{28,50}}, color={0,0,255}));
      connect(Sync_Speed.y, VfController.W_ref)
        annotation (Line(points={{15,0},{26,0}}, color={0,0,127}));
      connect(torqueSensor1.flange_b,load_inertia1. flange_a)
        annotation (Line(points={{110,-50},{120,-50}},
                                                   color={0,0,0}));
      connect(torque1.flange,load_inertia1. flange_b) annotation (Line(points={{140,-90},
              {150,-90},{150,-50},{140,-50}},  color={0,0,0}));
      connect(Torque_Equation.y, torque1.tau)
        annotation (Line(points={{101,-90},{118,-90}}, color={0,0,127}));
      connect(AC2DC_and_DC2AC.n, CIMVSD.p)
        annotation (Line(points={{48,50},{60,50}}, color={0,0,255}));
      connect(CIMVSD.flange, torqueSensor3.flange_a)
        annotation (Line(points={{80,50},{92,50}}, color={0,0,0}));
      connect(torqueSensor3.tau, CIMVSD.mech_torque) annotation (Line(points={{94,39},
              {94,32},{76,32},{76,38}}, color={0,0,127}));
      connect(VfController.we, CIMVSD.we)
        annotation (Line(points={{50,-4},{70,-4},{70,38}}, color={0,0,127}));
      connect(CIMVSD.wr, VfController.motor_speed)
        annotation (Line(points={{64,38},{64,4},{50,4}}, color={0,0,127}));
      connect(bus4_mt2.p, CIM.p)
        annotation (Line(points={{20,-50},{60,-50}}, color={0,0,255}));
      connect(CIM.mech_torque, torqueSensor1.tau) annotation (Line(points={{76,-62},
              {76,-68},{92,-68},{92,-61}}, color={0,0,127}));
      connect(SS3.y, CIM.we)
        annotation (Line(points={{41,-76},{70,-76},{70,-62}}, color={0,0,127}));
      connect(CIM.flange, torqueSensor1.flange_a)
        annotation (Line(points={{80,-50},{90,-50}}, color={0,0,0}));
      annotation (preferredView = "info",Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-120},
                {160,120}})), Diagram(coordinateSystem(preserveAspectRatio=false,
              extent={{-160,-120},{160,120}})),
        experiment(
          StopTime=10,
          __Dymola_NumberOfIntervals=5000,
          __Dymola_Algorithm="Dassl"),
        Documentation(info="<html>
<p>The example MultiDomainMotorTypeCIMStartUpComparison contains two separate systems meant for comparison. The top system is a multi-domain three-phase type V induction motor model controlled by a Variable Speed Drive while the bottom system is the same example but with no VSD controller. The purpose of this example is to compare the startup current magnitude of both systems and to point out one of the major benefits of using a VSD in the startup process. Starting an induction motor from a halt condition generates a peak starting current in the motor which is detrimental to the stability of the electrical system and to the magnitization coils of the machine itself. The user can compare both startup currents in one example and confirm that the startup current magnitude in the VSD example is substantially smaller in comparison to the non VSD example. </p>
<p>The Volts/Hertz scalar variable speed drive model is based on paper <i>Del Rosso, A. D., Mariano Anello, and E. Spittle. &quot;Stability Assessment of Isolated Power Systems with Large Induction Motor Drives.&quot; 2006 IEEE/PES Transmission &amp; Distribution Conference and Exposition: Latin America. IEEE, 2006. </i></p>
<p>Simulate the system for 50 seconds. Variables of interest and comparison are:</p>
<ul>
<li><span style=\"font-family: Courier New;\">CIM.Imag vs CIMVSD.Imag</span></li>
<li><span style=\"font-family: Courier New;\">CIM.s vs CIMVSD.s</span></li>
<li><span style=\"font-family: Courier New;\">CIM.wr vs CIMVSD.wr</span></li>
<li><span style=\"font-family: Courier New;\">CIM.P vs CIMVSD.P</span></li>
<li><span style=\"font-family: Courier New;\">CIM.Q vs CIMVSD.Q</span></li>
<li><span style=\"font-family: Courier New;\">VfController.m</span></li>
<li><span style=\"font-family: Courier New;\">VfController.we</span></li>
</ul>
</html>"));
    end MultiDomainMotorCIMStartUpComparison;

    model MultiDomainMotorStartUpComparison
      "Validation example for the variable speed drive driven CIM motor"
      extends Modelica.Icons.Example;
      extends OpenIMDML.Examples.BaseClasses.ValidationPartial3(inf2(X_d=0), inf1(
            X_d=0));
      parameter Real Ro = 0.1;

      OpenIPSL.Types.PerUnit TorqVSD;
      OpenIPSL.Types.PerUnit Torq;
      Modelica.Units.SI.AngularVelocity sync_speed;
      Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC AC2DC_and_DC2AC(V_b=23000,
          v_0=1) annotation (Placement(transformation(extent={{28,40},{48,60}})));
      Controls.VariableSpeedDrive.Controls.VoltsHertz_Controller VfController(
        V_b=23000,
        f_max=60,
        f_min=0,
        VSDstart=0.1)
        annotation (Placement(transformation(extent={{28,-10},{46,10}})));
      Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor1
        annotation (Placement(transformation(extent={{92,40},{112,60}})));
      Modelica.Mechanics.Rotational.Sources.Torque torque1
        annotation (Placement(transformation(extent={{120,-10},{140,10}})));
      Modelica.Mechanics.Rotational.Components.Inertia load_inertia1(J=1)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={130,50})));
      Modelica.Blocks.Sources.RealExpression TorqueEquation1(y=TorqVSD)
        annotation (Placement(transformation(extent={{80,-10},{100,10}})));
      Modelica.Blocks.Sources.Ramp Sync_Speed(
        height=0.9*(2*Modelica.Constants.pi*SysData.fn),
        duration=5,
        offset=0.1*(2*Modelica.Constants.pi*SysData.fn))
        annotation (Placement(transformation(extent={{-6,-10},{14,10}})));
      Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor2
        annotation (Placement(transformation(extent={{90,-60},{110,-40}})));
      Modelica.Mechanics.Rotational.Sources.Torque torque2
        annotation (Placement(transformation(extent={{120,-100},{140,-80}})));
      Modelica.Mechanics.Rotational.Components.Inertia load_inertia2(J=1)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={130,-50})));
      Modelica.Blocks.Sources.RealExpression TorqueEquation2(y=Torq)
        annotation (Placement(transformation(extent={{80,-100},{100,-80}})));
      Modelica.Blocks.Sources.RealExpression Synchronous_Speed(y=sync_speed)
        annotation (Placement(transformation(extent={{20,-86},{40,-66}})));
      MultiDomainModels.Motors.MD_ALL_IN_ONE_ThreePhaseMotor motorVSD(M_b=15000000,
          redeclare MultiDomainModels.Motors.ThreePhase.PSAT.MD_MotorTypeIII motor)
        annotation (Placement(transformation(extent={{60,40},{80,60}})));
      MultiDomainModels.Motors.MD_ALL_IN_ONE_ThreePhaseMotor motor(M_b=15000000,
          redeclare MultiDomainModels.Motors.ThreePhase.PSAT.MD_MotorTypeIII motor)
        annotation (Placement(transformation(extent={{60,-60},{80,-40}})));
    equation

      TorqVSD = -(0.1*(15/100)*(motorVSD.motor.s) + 0.5*(15/100)*(1 - motorVSD.motor.s)^2)*motorVSD.motor.T_b;
      Torq = -(0.1*(15/100)*(motor.motor.s) + 0.5*(15/100)*(1 - motor.motor.s)^2)*motor.motor.T_b;
      sync_speed = 2*Modelica.Constants.pi*SysData.fn;
      connect(AC2DC_and_DC2AC.Vc, VfController.Vc)
        annotation (Line(points={{33.4545,38},{33.4,12}}, color={0,0,127}));
      connect(VfController.m, AC2DC_and_DC2AC.m_input) annotation (Line(points={{42.6,12},
              {42.5455,12},{42.5455,38}},     color={0,0,127}));
      connect(torqueSensor1.flange_b, load_inertia1.flange_a)
        annotation (Line(points={{112,50},{120,50}}, color={0,0,0}));
      connect(torque1.flange, load_inertia1.flange_b) annotation (Line(points={{140,
              0},{150,0},{150,50},{140,50}}, color={0,0,0}));
      connect(TorqueEquation1.y, torque1.tau)
        annotation (Line(points={{101,0},{118,0}}, color={0,0,127}));
      connect(bus4_mt1.p, AC2DC_and_DC2AC.p)
        annotation (Line(points={{20,50},{28,50}}, color={0,0,255}));
      connect(Sync_Speed.y, VfController.W_ref)
        annotation (Line(points={{15,0},{26,0}}, color={0,0,127}));
      connect(torqueSensor2.flange_b,load_inertia2. flange_a)
        annotation (Line(points={{110,-50},{120,-50}},
                                                   color={0,0,0}));
      connect(torque2.flange, load_inertia2.flange_b) annotation (Line(points={{140,
              -90},{150,-90},{150,-50},{140,-50}}, color={0,0,0}));
      connect(TorqueEquation2.y, torque2.tau)
        annotation (Line(points={{101,-90},{118,-90}}, color={0,0,127}));
      connect(AC2DC_and_DC2AC.n, motorVSD.pwpin)
        annotation (Line(points={{48,50},{60,50}}, color={0,0,255}));
      connect(motorVSD.flange1,torqueSensor1. flange_a)
        annotation (Line(points={{80,50},{92,50}}, color={0,0,0}));
      connect(VfController.motor_speed, motorVSD.wr)
        annotation (Line(points={{50,4},{64,4},{64,38.0006}}, color={0,0,127}));
      connect(VfController.we, motorVSD.we)
        annotation (Line(points={{50,-4},{70,-4},{70,38}}, color={0,0,127}));
      connect(torqueSensor1.tau, motorVSD.mech_torque) annotation (Line(points={{94,
              39},{94,32},{76,32},{76,38}}, color={0,0,127}));
      connect(motor.flange1,torqueSensor2. flange_a)
        annotation (Line(points={{80,-50},{90,-50}}, color={0,0,0}));
      connect(bus4_mt2.p, motor.pwpin)
        annotation (Line(points={{20,-50},{60,-50}}, color={0,0,255}));
      connect(Synchronous_Speed.y, motor.we)
        annotation (Line(points={{41,-76},{70,-76},{70,-62}}, color={0,0,127}));
      connect(torqueSensor2.tau, motor.mech_torque) annotation (Line(points={{92,-61},
              {92,-68},{76,-68},{76,-62}}, color={0,0,127}));
      annotation (preferredView = "info",Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-120},
                {160,120}})), Diagram(coordinateSystem(preserveAspectRatio=false,
              extent={{-160,-120},{160,120}})),
        experiment(
          StopTime=10,
          __Dymola_NumberOfIntervals=5000,
          __Dymola_Algorithm="Dassl"),
        Documentation(info="<html>
<p>The example MultiDomainMotorStartUpComparison contains two separate systems meant for comparison. The top system is an ALL-IN-ONE multi-domain induction motor model controlled by a Variable Speed Drive while the bottom system is the same example but with no VSD controller. The purpose of this example is to compare the startup current magnitude of both systems and to point out one of the major benefits of using a VSD in the startup process. Starting an induction motor from a halt condition generates a peak starting current in the motor which is detrimental to the stability of the electrical system and to the magnitization coils of the machine itself. The user can compare both startup currents in one example and confirm that the startup current magnitude in the VSD example is substantially smaller in comparison to the non VSD example. </p>
<p>The Volts/Hertz scalar variable speed drive model is based on paper <i>Del Rosso, A. D., Mariano Anello, and E. Spittle. &quot;Stability Assessment of Isolated Power Systems with Large Induction Motor Drives.&quot; 2006 IEEE/PES Transmission &amp; Distribution Conference and Exposition: Latin America. IEEE, 2006. </i></p>
<p>Simulate the system for 50 seconds. Variables of interest and comparison are:</p>
<ul>
<li><span style=\"font-family: Courier New;\">motor.motor.Imag vs motorVSD.motor.Imag</span></li>
<li><span style=\"font-family: Courier New;\">motor.motor.s vs motorVSD.motor.s</span></li>
<li><span style=\"font-family: Courier New;\">motor.motor.wr vs motorVSD.motor.wr</span></li>
<li><span style=\"font-family: Courier New;\">motor.motor.P vs motorVSD.motor.P</span></li>
<li><span style=\"font-family: Courier New;\">motor.motor.Q vs motorVSD.motor.Q</span></li>
<li><span style=\"font-family: Courier New;\">VfController.m</span></li>
<li><span style=\"font-family: Courier New;\">VfController.we</span></li>
</ul>
</html>"));
    end MultiDomainMotorStartUpComparison;
    annotation (Documentation(info="<html>
</html>"));
  end MultiDomainThreePhaseMotorVSDStartup;

  package MultiDomainControllableMotor
    extends Modelica.Icons.ExamplesPackage;

    model VariableVoltageFrequencyMotorTypeI
      extends BaseClasses.ValidationPartial2;
      parameter Real v_start = 1;
      parameter Real f_start = 1;
      Real Torq;


      Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor1
        annotation (Placement(transformation(extent={{94,-10},{114,10}})));
      Modelica.Mechanics.Rotational.Components.Inertia load_inertia1(J=1)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={132,0})));
      Modelica.Mechanics.Rotational.Sources.Torque torque1
        annotation (Placement(transformation(extent={{122,-60},{142,-40}})));
      MultiDomainModels.Motors.ThreePhase.PSAT.MD_MotorTypeI Motor1VSD(V_b=23000,
          R1=0.08)
        annotation (Placement(transformation(extent={{82,-10},{62,10}})));
      Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC AC2DC_and_DC2AC(V_b=23000,
          v_0=1) annotation (Placement(transformation(extent={{30,-10},{50,10}})));
      Modelica.Blocks.Sources.RealExpression TorqueEquation(y=Torq)
        annotation (Placement(transformation(extent={{90,-60},{110,-40}})));
      Modelica.Blocks.Sources.RealExpression SVR(y=v_start)
        annotation (Placement(transformation(extent={{0,-40},{20,-20}})));
      Modelica.Blocks.Sources.RealExpression SSR(y=f_start)
        annotation (Placement(transformation(extent={{0,-60},{20,-40}})));
      Modelica.Blocks.Math.Gain gain(k=2*Modelica.Constants.pi*SysData.fn)
        annotation (Placement(transformation(extent={{32,-56},{44,-44}})));
    equation

      Torq = -0.4*Motor1VSD.wr^2;
      //Torq = -(0.1*(15/100)*(Motor1VSD.s) + 0.5*(15/100)*(1 - Motor1VSD.s)^2)*Motor1VSD.T_b;



      connect(torqueSensor1.flange_b,load_inertia1. flange_a)
        annotation (Line(points={{114,0},{122,0}},   color={0,0,0}));
      connect(torque1.flange,load_inertia1. flange_b) annotation (Line(points={{142,-50},
              {152,-50},{152,0},{142,0}},      color={0,0,0}));
      connect(Motor1VSD.flange,torqueSensor1. flange_a)
        annotation (Line(points={{82,0},{94,0}},   color={0,0,0}));
      connect(torqueSensor1.tau,Motor1VSD. mech_torque) annotation (Line(points={{96,-11},
              {96,-18},{78,-18},{78,-12}},  color={0,0,127}));
      connect(AC2DC_and_DC2AC.n,Motor1VSD. p)
        annotation (Line(points={{50,0},{62,0}},   color={0,0,255}));
      connect(TorqueEquation.y,torque1. tau)
        annotation (Line(points={{111,-50},{120,-50}},
                                                   color={0,0,127}));
      connect(bus4_mt1.p, AC2DC_and_DC2AC.p)
        annotation (Line(points={{20,0},{30,0}}, color={0,0,255}));
      connect(SVR.y, AC2DC_and_DC2AC.m_input) annotation (Line(points={{21,-30},
              {44.5455,-30},{44.5455,-12}},
                                   color={0,0,127}));
      connect(SSR.y, gain.u)
        annotation (Line(points={{21,-50},{30.8,-50}}, color={0,0,127}));
      connect(gain.y, Motor1VSD.we)
        annotation (Line(points={{44.6,-50},{72,-50},{72,-12}}, color={0,0,127}));
      annotation (experiment(
          StopTime=50,
          __Dymola_NumberOfIntervals=10000,
          __Dymola_Algorithm="Dassl"), Diagram(graphics={
            Line(
              points={{66,-72},{88,-56}},
              color={28,108,200},
              arrow={Arrow.None,Arrow.Filled}),
            Rectangle(extent={{38,-72},{94,-92}}, lineColor={28,108,200}),
            Text(
              extent={{40,-72},{92,-92}},
              textColor={28,108,200},
              textString="Mechanical Torque emulates
a fan or blower-type load.
Check equation section.")}));
    end VariableVoltageFrequencyMotorTypeI;

    model VariableVoltageFrequencyMotorTypeIII
      extends BaseClasses.ValidationPartial2;
      parameter Real v_start = 0.1;
      parameter Real f_start = 0.1;
      Real Torq;


      Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor1
        annotation (Placement(transformation(extent={{94,-10},{114,10}})));
      Modelica.Mechanics.Rotational.Components.Inertia load_inertia1(J=1)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={132,0})));
      Modelica.Mechanics.Rotational.Sources.Torque torque1
        annotation (Placement(transformation(extent={{122,-60},{142,-40}})));
      MultiDomainModels.Motors.ThreePhase.PSAT.MD_MotorTypeIII Motor3VSD(V_b=23000)
        annotation (Placement(transformation(extent={{82,-10},{62,10}})));
      Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC AC2DC_and_DC2AC(V_b=23000,
          v_0=1) annotation (Placement(transformation(extent={{30,-10},{50,10}})));
      Modelica.Blocks.Sources.RealExpression TorqueEquation(y=Torq)
        annotation (Placement(transformation(extent={{90,-60},{110,-40}})));
      Modelica.Blocks.Sources.RealExpression SVR(y=v_start)
        annotation (Placement(transformation(extent={{0,-40},{20,-20}})));
      Modelica.Blocks.Sources.RealExpression SSR(y=f_start)
        annotation (Placement(transformation(extent={{0,-60},{20,-40}})));
      Modelica.Blocks.Math.Gain gain(k=2*Modelica.Constants.pi*SysData.fn)
        annotation (Placement(transformation(extent={{32,-56},{44,-44}})));
    equation

      Torq = -0.4*Motor3VSD.wr^2;
      //Torq = -(0.1*(15/100)*(Motor3VSD.s) + 0.5*(15/100)*(1 - Motor3VSD.s)^2)*Motor3VSD.T_b;



      connect(torqueSensor1.flange_b,load_inertia1. flange_a)
        annotation (Line(points={{114,0},{122,0}},   color={0,0,0}));
      connect(torque1.flange,load_inertia1. flange_b) annotation (Line(points={{142,-50},
              {152,-50},{152,0},{142,0}},      color={0,0,0}));
      connect(Motor3VSD.flange,torqueSensor1. flange_a)
        annotation (Line(points={{82,0},{94,0}},   color={0,0,0}));
      connect(torqueSensor1.tau,Motor3VSD. mech_torque) annotation (Line(points={{96,-11},
              {96,-18},{78,-18},{78,-12}},  color={0,0,127}));
      connect(AC2DC_and_DC2AC.n,Motor3VSD. p)
        annotation (Line(points={{50,0},{62,0}},   color={0,0,255}));
      connect(TorqueEquation.y,torque1. tau)
        annotation (Line(points={{111,-50},{120,-50}},
                                                   color={0,0,127}));
      connect(bus4_mt1.p, AC2DC_and_DC2AC.p)
        annotation (Line(points={{20,0},{30,0}}, color={0,0,255}));
      connect(SVR.y, AC2DC_and_DC2AC.m_input) annotation (Line(points={{21,-30},
              {44.5455,-30},{44.5455,-12}},
                                   color={0,0,127}));
      connect(SSR.y, gain.u)
        annotation (Line(points={{21,-50},{30.8,-50}}, color={0,0,127}));
      connect(gain.y, Motor3VSD.we)
        annotation (Line(points={{44.6,-50},{72,-50},{72,-12}}, color={0,0,127}));
      annotation (Diagram(graphics={
            Line(
              points={{66,-72},{88,-56}},
              color={28,108,200},
              arrow={Arrow.None,Arrow.Filled}),
            Rectangle(extent={{38,-72},{94,-92}}, lineColor={28,108,200}),
            Text(
              extent={{40,-72},{92,-92}},
              textColor={28,108,200},
              textString="Mechanical Torque emulates
a fan or blower-type load.
Check equation section.")}));
    end VariableVoltageFrequencyMotorTypeIII;

    model VariableVoltageFrequencyMotorTypeV
      extends BaseClasses.ValidationPartial2;
      parameter Real v_start = 0.1;
      parameter Real f_start = 0.1;
      Real Torq;

      Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor1
        annotation (Placement(transformation(extent={{94,-10},{114,10}})));
      Modelica.Mechanics.Rotational.Components.Inertia load_inertia1(J=1)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={132,0})));
      Modelica.Mechanics.Rotational.Sources.Torque torque1
        annotation (Placement(transformation(extent={{122,-60},{142,-40}})));
      MultiDomainModels.Motors.ThreePhase.PSAT.MD_MotorTypeV Motor5VSD(V_b=23000)
        annotation (Placement(transformation(extent={{82,-10},{62,10}})));
      Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC AC2DC_and_DC2AC(V_b=23000,
          v_0=1) annotation (Placement(transformation(extent={{30,-10},{50,10}})));
      Modelica.Blocks.Sources.RealExpression TorqueEquation(y=-(0.1*(15/100)*(
            Motor5VSD.s) + 0.5*(15/100)*(1 - Motor5VSD.s)^2)*Motor5VSD.T_b)
        annotation (Placement(transformation(extent={{90,-60},{110,-40}})));
      Modelica.Blocks.Sources.RealExpression SVR(y=v_start)
        annotation (Placement(transformation(extent={{0,-40},{20,-20}})));
      Modelica.Blocks.Sources.RealExpression SSR(y=f_start)
        annotation (Placement(transformation(extent={{0,-60},{20,-40}})));
      Modelica.Blocks.Math.Gain gain(k=2*Modelica.Constants.pi*SysData.fn)
        annotation (Placement(transformation(extent={{32,-56},{44,-44}})));
    equation

      Torq = -0.4*Motor5VSD.wr^2;
      //Torq = -(0.1*(15/100)*(Motor5VSD.s) + 0.5*(15/100)*(1 - Motor5VSD.s)^2)*Motor5VSD.T_b;

      connect(torqueSensor1.flange_b,load_inertia1. flange_a)
        annotation (Line(points={{114,0},{122,0}},   color={0,0,0}));
      connect(torque1.flange,load_inertia1. flange_b) annotation (Line(points={{142,-50},
              {152,-50},{152,0},{142,0}},      color={0,0,0}));
      connect(Motor5VSD.flange,torqueSensor1. flange_a)
        annotation (Line(points={{82,0},{94,0}},   color={0,0,0}));
      connect(torqueSensor1.tau,Motor5VSD. mech_torque) annotation (Line(points={{96,-11},
              {96,-18},{78,-18},{78,-12}},  color={0,0,127}));
      connect(AC2DC_and_DC2AC.n,Motor5VSD. p)
        annotation (Line(points={{50,0},{62,0}},   color={0,0,255}));
      connect(TorqueEquation.y,torque1. tau)
        annotation (Line(points={{111,-50},{120,-50}},
                                                   color={0,0,127}));
      connect(bus4_mt1.p, AC2DC_and_DC2AC.p)
        annotation (Line(points={{20,0},{30,0}}, color={0,0,255}));
      connect(SVR.y, AC2DC_and_DC2AC.m_input) annotation (Line(points={{21,-30},
              {44.5455,-30},{44.5455,-12}},
                                   color={0,0,127}));
      connect(SSR.y, gain.u)
        annotation (Line(points={{21,-50},{30.8,-50}}, color={0,0,127}));
      connect(gain.y, Motor5VSD.we)
        annotation (Line(points={{44.6,-50},{72,-50},{72,-12}}, color={0,0,127}));
      annotation (Diagram(graphics={
            Line(
              points={{66,-72},{88,-56}},
              color={28,108,200},
              arrow={Arrow.None,Arrow.Filled}),
            Rectangle(extent={{38,-72},{94,-92}}, lineColor={28,108,200}),
            Text(
              extent={{40,-72},{92,-92}},
              textColor={28,108,200},
              textString="Mechanical Torque emulates
a fan or blower-type load.
Check equation section.")}));
    end VariableVoltageFrequencyMotorTypeV;
  end MultiDomainControllableMotor;
end MultiDomainThreePhaseInductionMotors;
