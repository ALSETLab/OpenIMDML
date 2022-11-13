within OpenIMDML.Examples.MultiDomainExamples.ThreePhaseInductionMotors.VSDMotorStartUp;
model MotorTypeIII_VSD_Ramp_StartUp2
  extends Modelica.Icons.Example;

  parameter Real start_point_v = 1;
  parameter Real start_point_f = 1;
  OpenIPSL.Electrical.Machines.PSSE.GENCLS inf3(
    V_b=16000,
    v_0=1.05,
    M_b=600000000,
    H=0)
    annotation (Placement(transformation(extent={{-140,40},{-120,60}})));
  OpenIPSL.Electrical.Buses.Bus bus1_mt3(V_b=16000, v_0=1.05)
    annotation (Placement(transformation(extent={{-110,40},{-90,60}})));
  OpenIPSL.Electrical.Buses.Bus bus2_mt3(V_b=230000)
    annotation (Placement(transformation(extent={{-70,40},{-50,60}})));
  OpenIPSL.Electrical.Branches.PwLine line_mt3(
    R=0,
    X=0.02,
    G=0,
    B=0) annotation (Placement(transformation(extent={{-44,36},{-16,64}})));
  OpenIPSL.Electrical.Buses.Bus bus3_mt3(V_b=230000)
    annotation (Placement(transformation(extent={{-10,40},{10,60}})));
  OpenIPSL.Electrical.Buses.Bus bus4_mt3(V_b=23000)
    annotation (Placement(transformation(extent={{30,40},{50,60}})));
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor3
    annotation (Placement(transformation(extent={{34,-50},{54,-30}})));
  Modelica.Mechanics.Rotational.Sources.Torque torque3
    annotation (Placement(transformation(extent={{64,-90},{84,-70}})));
  Modelica.Mechanics.Rotational.Components.Inertia load_inertia3(J=1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={74,-40})));
  OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1_mt3(
    V_b=16000,
    Vn=16000,
    rT=0,
    xT=0.025)
             annotation (Placement(transformation(extent={{-90,40},{-70,60}})));
  OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2_mt3(
    V_b=230000,
    Vn=230000,
    rT=0,
    xT=0.15) annotation (Placement(transformation(extent={{10,40},{30,60}})));
  OpenIPSL.Electrical.Loads.PSAT.PQ Load3(V_b=230000, P_0=500000000)
    annotation (Placement(transformation(extent={{-20,12},{0,32}})));
  inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
    annotation (Placement(transformation(extent={{80,80},{140,120}})));
  Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC aC2DC_and_DC2AC(V_b=23000)
    annotation (Placement(transformation(extent={{-60,-50},{-40,-30}})));
  Modelica.Blocks.Sources.Ramp change(
    height=0,
    duration=0,
    offset=start_point_v)
    annotation (Placement(transformation(extent={{-120,-90},{-100,-70}})));
  Modelica.Blocks.Math.Gain pwm_modulation_index(k=1)
    annotation (Placement(transformation(extent={{-80,-90},{-60,-70}})));
  Modelica.Blocks.Math.Gain synchronous_speed(k=2*Modelica.Constants.pi*SysData.fn)
    annotation (Placement(transformation(extent={{-80,-140},{-60,-120}})));
  Modelica.Blocks.Sources.Ramp change1(
    height=0,
    duration=0,
    offset=start_point_f)
    annotation (Placement(transformation(extent={{-120,-140},{-100,-120}})));
  Modelica.Blocks.Sources.RealExpression Torque3(y=-(0.1*(15/100)*(Motor3.s) + 0.5
        *(15/100)*(1 - Motor3.s)^2)*Motor3.T_b)
    annotation (Placement(transformation(extent={{24,-90},{44,-70}})));
equation
  connect(inf3.p,bus1_mt3. p)
    annotation (Line(points={{-120,50},{-100,50}},
                                                 color={0,0,255}));
  connect(bus2_mt3.p,line_mt3. p)
    annotation (Line(points={{-60,50},{-42.6,50}},
                                                 color={0,0,255}));
  connect(line_mt3.n,bus3_mt3. p)
    annotation (Line(points={{-17.4,50},{0,50}},
                                               color={0,0,255}));
  connect(torqueSensor3.flange_b,load_inertia3. flange_a)
    annotation (Line(points={{54,-40},{64,-40}},
                                               color={0,0,0}));
  connect(torque3.flange,load_inertia3. flange_b) annotation (Line(points={{84,-80},
          {94,-80},{94,-40},{84,-40}},     color={0,0,0}));
  connect(bus1_mt3.p,tf1_mt3. p)
    annotation (Line(points={{-100,50},{-91,50}},
                                                color={0,0,255}));
  connect(tf1_mt3.n,bus2_mt3. p)
    annotation (Line(points={{-69,50},{-60,50}},
                                               color={0,0,255}));
  connect(bus3_mt3.p,tf2_mt3. p)
    annotation (Line(points={{0,50},{9,50}},
                                           color={0,0,255}));
  connect(tf2_mt3.n,bus4_mt3. p)
    annotation (Line(points={{31,50},{40,50}},
                                             color={0,0,255}));
  connect(Load3.p,bus3_mt3. p)
    annotation (Line(points={{-10,32},{-10,50},{0,50}},color={0,0,255}));
  connect(torqueSensor3.tau,Motor3. mech_torque) annotation (Line(points={{36,-51},
          {36,-60},{16,-60},{16,-52}}, color={0,0,127}));
  connect(Motor3.flange,torqueSensor3. flange_a)
    annotation (Line(points={{20,-40},{34,-40}},
                                             color={0,0,0}));
  connect(aC2DC_and_DC2AC.n, Motor3.p)
    annotation (Line(points={{-40,-40},{0,-40}}, color={0,0,255}));
  connect(aC2DC_and_DC2AC.p, bus4_mt3.p) annotation (Line(points={{-60,-40},{-100,
          -40},{-100,0},{80,0},{80,50},{40,50}}, color={0,0,255}));
  connect(change.y, pwm_modulation_index.u)
    annotation (Line(points={{-99,-80},{-82,-80}}, color={0,0,127}));
  connect(pwm_modulation_index.y, aC2DC_and_DC2AC.m_input) annotation (Line(
        points={{-59,-80},{-45.4545,-80},{-45.4545,-52}}, color={0,0,127}));
  connect(synchronous_speed.y, Motor3.we)
    annotation (Line(points={{-59,-130},{10,-130},{10,-52}}, color={0,0,127}));
  connect(change1.y, synchronous_speed.u)
    annotation (Line(points={{-99,-130},{-82,-130}}, color={0,0,127}));
  connect(Torque3.y, torque3.tau)
    annotation (Line(points={{45,-80},{62,-80}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
            -160,-160},{160,160}})),                             Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-160,-160},{
            160,160}})),
    experiment(
      StopTime=10,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end MotorTypeIII_VSD_Ramp_StartUp2;
