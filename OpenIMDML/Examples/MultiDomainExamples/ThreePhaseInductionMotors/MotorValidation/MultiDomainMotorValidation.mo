within OpenIMDML.Examples.MultiDomainExamples.ThreePhaseInductionMotors.MotorValidation;
model MultiDomainMotorValidation
  extends Modelica.Icons.Example;

  OpenIPSL.Types.PerUnit Torq;
  Modelica.Units.SI.AngularVelocity sync_speed;

  OpenIPSL.Electrical.Machines.PSSE.GENCLS inf(
    V_b=16000,
    v_0=1.05,
    M_b=600000000,
    H=0,
    X_d=0)
         annotation (Placement(transformation(extent={{-116,-10},{-96,10}})));
  OpenIPSL.Electrical.Buses.Bus bus1(V_b=16000, v_0=1.05)
    annotation (Placement(transformation(extent={{-94,-10},{-74,10}})));
  OpenIPSL.Electrical.Buses.Bus bus2(V_b=230000, v_0=1.03)
    annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  OpenIPSL.Electrical.Branches.PwLine line_mt1(
    R=0,
    X=0.02,
    G=0,
    B=0) annotation (Placement(transformation(extent={{-42,-14},{-14,14}})));
  OpenIPSL.Electrical.Buses.Bus bus3(V_b=230000, v_0=1.023)
    annotation (Placement(transformation(extent={{-14,-10},{6,10}})));
  inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
    annotation (Placement(transformation(extent={{52,32},{100,60}})));
  OpenIPSL.Electrical.Buses.Bus bus4(V_b=23000, angle_0=0.017715091907742)
    annotation (Placement(transformation(extent={{22,-10},{42,10}})));
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor Torque_Sensor
    annotation (Placement(transformation(extent={{66,-10},{86,10}})));
  Modelica.Mechanics.Rotational.Components.Inertia Load_Inertia(J=1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={102,0})));
  Modelica.Mechanics.Rotational.Sources.Torque Torque
    annotation (Placement(transformation(extent={{94,-46},{114,-26}})));
  Modelica.Blocks.Sources.RealExpression Torque_Equation(y=Torq)
    annotation (Placement(transformation(extent={{62,-46},{82,-26}})));
  OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1(
    V_b=16000,
    Vn=16000,
    rT=0,
    xT=0.025)
    annotation (Placement(transformation(extent={{-76,-10},{-56,10}})));
  OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2(
    V_b=230000,
    Vn=230000,
    rT=0,
    xT=0.15) annotation (Placement(transformation(extent={{4,-10},{24,10}})));
  OpenIPSL.Electrical.Loads.PSAT.PQ Load(V_b=230000, P_0=500000000)
    annotation (Placement(transformation(extent={{-20,-40},{0,-20}})));
  Modelica.Blocks.Sources.RealExpression Synchronous_Speed(y=sync_speed)
    annotation (Placement(transformation(extent={{12,-46},{32,-26}})));
  MultiDomainModels.Motors.MD_ALL_IN_ONE_ThreePhaseMotor motor(M_b=15000000,
      redeclare OpenIMDML.MultiDomainModels.Motors.ThreePhase.PSAT.MotorTypeIII
      motor) annotation (Placement(transformation(extent={{40,-10},{60,10}})));
equation

  Torq = -(0.1*(15/100)*(motor.motor.s) + 0.5*(15/100)*(1 - motor.motor.s)^2)*motor.motor.T_b;
  //Torq = -(0.1*(motor.motor.s) + 0.5*(1 - motor.motor.s)^2);

  sync_speed = 2*Modelica.Constants.pi*SysData.fn;

  connect(inf.p, bus1.p)
    annotation (Line(points={{-96,0},{-84,0}},   color={0,0,255}));
  connect(bus2.p, line_mt1.p)
    annotation (Line(points={{-48,0},{-40.6,0}}, color={0,0,255}));
  connect(line_mt1.n, bus3.p)
    annotation (Line(points={{-15.4,0},{-4,0}}, color={0,0,255}));
  connect(Torque_Sensor.flange_b, Load_Inertia.flange_a)
    annotation (Line(points={{86,0},{92,0}}, color={0,0,0}));
  connect(Torque.flange,Load_Inertia. flange_b) annotation (Line(points={{114,-36},
          {118,-36},{118,0},{112,0}}, color={0,0,0}));
  connect(bus1.p, tf1.p)
    annotation (Line(points={{-84,0},{-77,0}},  color={0,0,255}));
  connect(tf1.n, bus2.p)
    annotation (Line(points={{-55,0},{-48,0}}, color={0,0,255}));
  connect(bus3.p, tf2.p)
    annotation (Line(points={{-4,0},{3,0}}, color={0,0,255}));
  connect(tf2.n, bus4.p)
    annotation (Line(points={{25,0},{32,0}}, color={0,0,255}));
  connect(Torque_Equation.y, Torque.tau)
    annotation (Line(points={{83,-36},{92,-36}}, color={0,0,127}));
  connect(Load.p, bus3.p)
    annotation (Line(points={{-10,-20},{-10,0},{-4,0}}, color={0,0,255}));
  connect(bus4.p, motor.pwpin)
    annotation (Line(points={{32,0},{40,0}}, color={0,0,255}));
  connect(motor.flange1, Torque_Sensor.flange_a)
    annotation (Line(points={{60,0},{66,0}}, color={0,0,0}));
  connect(Torque_Sensor.tau, motor.mech_torque) annotation (Line(points={
          {68,-11},{68,-20},{56,-20},{56,-12}}, color={0,0,127}));
  connect(Synchronous_Speed.y, motor.we) annotation (Line(points={{33,-36},
          {50,-36},{50,-12}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},
            {120,100}})), Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-120,-100},{120,100}})),
    experiment(
      StopTime=5,
      __Dymola_NumberOfIntervals=10000,
      Tolerance=1e-05,
      __Dymola_Algorithm="Dassl"));
end MultiDomainMotorValidation;
