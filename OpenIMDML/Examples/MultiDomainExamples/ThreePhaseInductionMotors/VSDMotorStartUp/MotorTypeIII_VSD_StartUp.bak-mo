within OpenIMDML.Examples.MultiDomainExamples.ThreePhaseInductionMotors.VSDMotorStartUp;
model MotorTypeIII_VSD_StartUp
  extends Modelica.Icons.Example;
  OpenIPSL.Electrical.Machines.PSSE.GENCLS inf1(
    V_b=16000,
    v_0=1.05,
    M_b=600000000,
    H=0)
    annotation (Placement(transformation(extent={{-146,40},{-126,60}})));
  OpenIPSL.Electrical.Buses.Bus bus1_mt1(V_b=16000, v_0=1.05)
    annotation (Placement(transformation(extent={{-116,40},{-96,60}})));
  OpenIPSL.Electrical.Buses.Bus bus2_mt1(V_b=230000)
    annotation (Placement(transformation(extent={{-76,40},{-56,60}})));
  OpenIPSL.Electrical.Branches.PwLine line_mt1(
    R=0,
    X=0.02,
    G=0,
    B=0) annotation (Placement(transformation(extent={{-50,36},{-22,64}})));
  OpenIPSL.Electrical.Buses.Bus bus3_mt1(V_b=230000)
    annotation (Placement(transformation(extent={{-16,40},{4,60}})));
  inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
    annotation (Placement(transformation(extent={{80,80},{140,120}})));
  OpenIPSL.Electrical.Buses.Bus bus4_mt1(V_b=23000)
    annotation (Placement(transformation(extent={{24,40},{44,60}})));
  OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1_mt1(
    V_b=16000,
    Vn=16000,
    rT=0,
    xT=0.025)
             annotation (Placement(transformation(extent={{-96,40},{-76,60}})));
  OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2_mt1(
    V_b=230000,
    Vn=230000,
    rT=0,
    xT=0.15) annotation (Placement(transformation(extent={{4,40},{24,60}})));
  Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC
    aC2DC_and_DC2AC_uninitialized(V_b=23000, v_0=1)
    annotation (Placement(transformation(extent={{-60,-50},{-40,-30}})));
  Controls.VariableSpeedDrive.Controls.VoltsHertz_Controller
    voltsHertz_Controller(
    V_b=23000,
    f_max=60,
    f_min=0)
    annotation (Placement(transformation(extent={{-60,-100},{-42,-80}})));
  OpenIPSL.Electrical.Loads.PSAT.PQ Load1(V_b=230000, P_0=500000000)
    annotation (Placement(transformation(extent={{-26,12},{-6,32}})));
  Modelica.Blocks.Sources.RealExpression FREQUENCY(y=2*Modelica.Constants.pi
        *SysData.fn) annotation (Placement(transformation(extent={{-132,-100},
            {-112,-80}})));
  MultiDomainModels.Motors.ThreePhase.PSAT.MotorTypeIII Motor3(V_b=23000)
    annotation (Placement(transformation(extent={{20,-50},{0,-30}})));
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor3
    annotation (Placement(transformation(extent={{34,-50},{54,-30}})));
  Modelica.Mechanics.Rotational.Sources.Torque torque3
    annotation (Placement(transformation(extent={{64,-90},{84,-70}})));
  Modelica.Mechanics.Rotational.Components.Inertia load_inertia3(J=1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={74,-40})));
  Modelica.Blocks.Sources.RealExpression Torque3(y=-(0.1*(15/100)*(Motor3.s)
         + 0.5*(15/100)*(1 - Motor3.s)^2)*Motor3.T_b)
    annotation (Placement(transformation(extent={{28,-90},{48,-70}})));
equation
  connect(inf1.p,bus1_mt1. p)
    annotation (Line(points={{-126,50},{-106,50}}, color={0,0,255}));
  connect(bus2_mt1.p,line_mt1. p)
    annotation (Line(points={{-66,50},{-48.6,50}}, color={0,0,255}));
  connect(line_mt1.n,bus3_mt1. p)
    annotation (Line(points={{-23.4,50},{-6,50}},color={0,0,255}));
  connect(bus1_mt1.p,tf1_mt1. p)
    annotation (Line(points={{-106,50},{-97,50}}, color={0,0,255}));
  connect(tf1_mt1.n,bus2_mt1. p)
    annotation (Line(points={{-75,50},{-66,50}}, color={0,0,255}));
  connect(bus3_mt1.p,tf2_mt1. p)
    annotation (Line(points={{-6,50},{3,50}},color={0,0,255}));
  connect(tf2_mt1.n,bus4_mt1. p)
    annotation (Line(points={{25,50},{34,50}}, color={0,0,255}));
  connect(bus4_mt1.p, aC2DC_and_DC2AC_uninitialized.p) annotation (Line(points={
          {34,50},{80,50},{80,0},{-100,0},{-100,-40},{-60,-40}}, color={0,0,255}));
  connect(aC2DC_and_DC2AC_uninitialized.Vc, voltsHertz_Controller.Vc)
    annotation (Line(points={{-54.5455,-52},{-54.6,-78}}, color={0,0,127}));
  connect(voltsHertz_Controller.m, aC2DC_and_DC2AC_uninitialized.m_input)
    annotation (Line(points={{-45.4,-78},{-45.4545,-78},{-45.4545,-52}}, color={
          0,0,127}));
  connect(Load1.p, bus3_mt1.p)
    annotation (Line(points={{-16,32},{-16,50},{-6,50}}, color={0,0,255}));
  connect(aC2DC_and_DC2AC_uninitialized.n, Motor3.p)
    annotation (Line(points={{-40,-40},{0,-40}}, color={0,0,255}));
  connect(voltsHertz_Controller.we, Motor3.we) annotation (Line(points={{
          -38,-94},{10,-94},{10,-52}}, color={0,0,127}));
  connect(Motor3.wr, voltsHertz_Controller.motor_speed) annotation (Line(
        points={{4,-52},{4,-86},{-38,-86}}, color={0,0,127}));
  connect(torqueSensor3.flange_b,load_inertia3. flange_a)
    annotation (Line(points={{54,-40},{64,-40}},
                                               color={0,0,0}));
  connect(torque3.flange,load_inertia3. flange_b) annotation (Line(points={{84,-80},
          {94,-80},{94,-40},{84,-40}},     color={0,0,0}));
  connect(Torque3.y,torque3. tau)
    annotation (Line(points={{49,-80},{62,-80}},   color={0,0,127}));
  connect(Motor3.flange, torqueSensor3.flange_a)
    annotation (Line(points={{20,-40},{34,-40}}, color={0,0,0}));
  connect(torqueSensor3.tau, Motor3.mech_torque) annotation (Line(points=
          {{36,-51},{36,-60},{16,-60},{16,-52}}, color={0,0,127}));
  connect(FREQUENCY.y, voltsHertz_Controller.W_ref)
    annotation (Line(points={{-111,-90},{-62,-90}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-160},
            {160,160}})), Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-160,-160},{160,160}})),
    experiment(
      StopTime=100,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end MotorTypeIII_VSD_StartUp;
