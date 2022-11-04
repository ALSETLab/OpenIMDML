within OpenIMDML.Examples.MultiDomainExamples.ThreePhaseInductionMotors.VSDMotorStartUp;
model MotorTypeV_VSD_Ramp_StartUp
  extends Modelica.Icons.Example;
  OpenIPSL.Electrical.Machines.PSSE.GENCLS inf5(
    V_b=16000,
    v_0=1.05,
    M_b=600000000,
    H=0)
    annotation (Placement(transformation(extent={{-140,40},{-120,60}})));
  OpenIPSL.Electrical.Buses.Bus bus1_mt5(V_b=16000, v_0=1.05)
    annotation (Placement(transformation(extent={{-110,40},{-90,60}})));
  OpenIPSL.Electrical.Buses.Bus bus2_mt5(V_b=230000)
    annotation (Placement(transformation(extent={{-70,40},{-50,60}})));
  OpenIPSL.Electrical.Branches.PwLine line_mt5(
    R=0,
    X=0.02,
    G=0,
    B=0) annotation (Placement(transformation(extent={{-44,36},{-16,64}})));
  OpenIPSL.Electrical.Buses.Bus bus3_mt5(V_b=230000)
    annotation (Placement(transformation(extent={{-10,40},{10,60}})));
  OpenIPSL.Electrical.Buses.Bus bus4_mt5(V_b=23000)
    annotation (Placement(transformation(extent={{30,40},{50,60}})));
  OpenIPSL.Electrical.Loads.PSAT.PQ Load5(V_b=230000, P_0=500000000)
    annotation (Placement(transformation(extent={{-20,12},{0,32}})));
  OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1_mt5(
    V_b=16000,
    Vn=16000,
    rT=0,
    xT=0.025)
             annotation (Placement(transformation(extent={{-90,40},{-70,60}})));
  OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2_mt5(
    V_b=230000,
    Vn=230000,
    rT=0,
    xT=0.15) annotation (Placement(transformation(extent={{10,40},{30,60}})));
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor5
    annotation (Placement(transformation(extent={{34,-50},{54,-30}})));
  Modelica.Mechanics.Rotational.Sources.Torque torque5
    annotation (Placement(transformation(extent={{64,-90},{84,-70}})));
  Modelica.Mechanics.Rotational.Components.Inertia load_inertia5(J=1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={74,-40})));
  inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
    annotation (Placement(transformation(extent={{80,80},{140,120}})));
  MultiDomainModels.Motors.ThreePhase.PSAT.MotorTypeV Motor5
    annotation (Placement(transformation(extent={{20,-50},{0,-30}})));
  Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC
    aC2DC_and_DC2AC_uninitialized(V_b=23000, v_0=1)
    annotation (Placement(transformation(extent={{-60,-50},{-40,-30}})));
  Modelica.Blocks.Sources.Ramp     m_const(
    height=0,
    duration=50,
    offset=1)
    annotation (Placement(transformation(extent={{-140,-108},{-120,-88}})));
  Modelica.Blocks.Math.Gain gain(k=1) annotation (Placement(
        transformation(extent={{-78,-108},{-58,-88}})));
  Modelica.Blocks.Math.Gain gain1(k=2*3.1415*SysData.fn) annotation (
      Placement(transformation(extent={{-78,-150},{-58,-130}})));
  Modelica.Blocks.Sources.RealExpression Torque5(y=-0.01*Motor5.T_b)
    annotation (Placement(transformation(extent={{28,-90},{48,-70}})));
  Modelica.Blocks.Sources.Ramp     m_const1(
    height=0,
    duration=50,
    offset=1.1)
    annotation (Placement(transformation(extent={{-140,-150},{-120,-130}})));
equation
  connect(inf5.p,bus1_mt5. p)
    annotation (Line(points={{-120,50},{-100,50}},   color={0,0,255}));
  connect(bus2_mt5.p,line_mt5. p)
    annotation (Line(points={{-60,50},{-42.6,50}},   color={0,0,255}));
  connect(line_mt5.n,bus3_mt5. p)
    annotation (Line(points={{-17.4,50},{0,50}},   color={0,0,255}));
  connect(Load5.p,bus3_mt5. p)
    annotation (Line(points={{-10,32},{-10,50},{0,50}},     color={0,0,255}));
  connect(bus1_mt5.p,tf1_mt5. p)
    annotation (Line(points={{-100,50},{-91,50}},   color={0,0,255}));
  connect(tf1_mt5.n,bus2_mt5. p)
    annotation (Line(points={{-69,50},{-60,50}},   color={0,0,255}));
  connect(bus3_mt5.p,tf2_mt5. p)
    annotation (Line(points={{0,50},{9,50}},   color={0,0,255}));
  connect(tf2_mt5.n,bus4_mt5. p)
    annotation (Line(points={{31,50},{40,50}},   color={0,0,255}));
  connect(torqueSensor5.flange_b,load_inertia5. flange_a)
    annotation (Line(points={{54,-40},{64,-40}},
                                               color={0,0,0}));
  connect(torque5.flange,load_inertia5. flange_b) annotation (Line(points={{84,-80},
          {94,-80},{94,-40},{84,-40}},     color={0,0,0}));
  connect(Motor5.flange,torqueSensor5. flange_a)
    annotation (Line(points={{20,-40},{34,-40}},
                                             color={0,0,0}));
  connect(torqueSensor5.tau,Motor5. mech_torque) annotation (Line(points={{36,-51},
          {36,-60},{16,-60},{16,-52}},      color={0,0,127}));
  connect(aC2DC_and_DC2AC_uninitialized.n, Motor5.p)
    annotation (Line(points={{-40,-40},{0,-40}}, color={0,0,255}));
  connect(aC2DC_and_DC2AC_uninitialized.p, bus4_mt5.p) annotation (Line(
        points={{-60,-40},{-100,-40},{-100,0},{80,0},{80,50},{40,50}},
        color={0,0,255}));
  connect(m_const.y, gain.u)
    annotation (Line(points={{-119,-98},{-80,-98}},   color={0,0,127}));
  connect(gain.y, aC2DC_and_DC2AC_uninitialized.m_input) annotation (Line(
        points={{-57,-98},{-45.4545,-98},{-45.4545,-52}},   color={0,0,
          127}));
  connect(gain1.y, Motor5.we) annotation (Line(points={{-57,-140},{10,
          -140},{10,-52}}, color={0,0,127}));
  connect(Torque5.y, torque5.tau)
    annotation (Line(points={{49,-80},{62,-80}}, color={0,0,127}));
  connect(m_const1.y, gain1.u)
    annotation (Line(points={{-119,-140},{-80,-140}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
            -160,-160},{160,160}})),                             Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-160,-160},{
            160,160}})),
    experiment(
      StopTime=100,
      __Dymola_NumberOfIntervals=10000,
      __Dymola_Algorithm="Dassl"));
end MotorTypeV_VSD_Ramp_StartUp;
