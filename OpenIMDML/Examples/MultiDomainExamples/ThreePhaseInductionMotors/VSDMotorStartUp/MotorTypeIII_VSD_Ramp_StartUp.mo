within OpenIMDML.Examples.MultiDomainExamples.ThreePhaseInductionMotors.VSDMotorStartUp;
model MotorTypeIII_VSD_Ramp_StartUp
  extends Modelica.Icons.Example;
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
  Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC aC2DC_and_DC2AC(V_b=23000,
      v_0=3*sqrt(6)/(2*Modelica.Constants.pi*sqrt(2)))
    annotation (Placement(transformation(extent={{-58,-50},{-38,-30}})));
  Modelica.Blocks.Sources.Ramp change(
    height=0,
    duration=60,
    offset=0.52) annotation (Placement(transformation(extent={{-144,-90},
            {-124,-70}})));
  Modelica.Blocks.Sources.RealExpression Torque3(y=-(0.05)*Motor3.T_b)
    annotation (Placement(transformation(extent={{26,-90},{46,-70}})));
  Modelica.Blocks.Math.Gain gain(k=1) annotation (Placement(
        transformation(extent={{-80,-90},{-60,-70}})));
  Modelica.Blocks.Math.Gain gain1(k=2*Modelica.Constants.pi*SysData.fn)
                                                         annotation (
      Placement(transformation(extent={{-80,-152},{-60,-132}})));
  Modelica.Blocks.Sources.Ramp change1(
    height=0,
    duration=60,
    offset=0.41) annotation (Placement(transformation(extent={{-140,-152},
            {-120,-132}})));
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
    annotation (Line(points={{-38,-40},{0,-40}}, color={0,0,255}));
  connect(aC2DC_and_DC2AC.p, bus4_mt3.p) annotation (Line(points={{-58,
          -40},{-100,-40},{-100,0},{80,0},{80,50},{40,50}}, color={0,0,
          255}));
  connect(Torque3.y, torque3.tau)
    annotation (Line(points={{47,-80},{62,-80}}, color={0,0,127}));
  connect(change.y, gain.u)
    annotation (Line(points={{-123,-80},{-82,-80}}, color={0,0,127}));
  connect(gain.y, aC2DC_and_DC2AC.m_input) annotation (Line(points={{-59,-80},
          {-44,-80},{-44,-56},{-43.4545,-56},{-43.4545,-52}},      color=
          {0,0,127}));
  connect(gain1.y, Motor3.we) annotation (Line(points={{-59,-142},{10,
          -142},{10,-52}}, color={0,0,127}));
  connect(gain1.u, change1.y)
    annotation (Line(points={{-82,-142},{-119,-142}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
            -160,-160},{160,160}})),                             Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-160,-160},{
            160,160}})),
    experiment(
      StopTime=10,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end MotorTypeIII_VSD_Ramp_StartUp;
