within OpenIMDML.Examples.MultiDomainThreePhaseInductionMotors.VSDMotorStartUp;
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
  Modelica.Blocks.Sources.RealExpression Torque3(y=-(0.5*(15/100)*(1 - Motor3.s)
        ^2)*Motor3.T_b)
    annotation (Placement(transformation(extent={{28,-90},{48,-70}})));
  OpenIPSL.Electrical.Loads.PSAT.PQ Load3(V_b=230000, P_0=500000000)
    annotation (Placement(transformation(extent={{-20,12},{0,32}})));
  inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
    annotation (Placement(transformation(extent={{80,80},{140,120}})));
  MultiDomain.Motors.ThreePhase.PSAT.MotorTypeIII Motor3(V_b=23000)
    annotation (Placement(transformation(extent={{20,-50},{0,-30}})));
  Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC_uninitialized
    aC2DC_and_DC2AC_uninitialized(V_b=23000)
    annotation (Placement(transformation(extent={{-60,-50},{-40,-30}})));
  Modelica.Blocks.Sources.Ramp m_ramp(
    height=0.99,
    duration=100,
    offset=0.01)
    annotation (Placement(transformation(extent={{-76,-90},{-56,-70}})));
  Modelica.Blocks.Sources.Ramp f_ramp(
    height=0.9*2*Modelica.Constants.pi*SysData.fn,
    duration=100,
    offset=0.1*2*Modelica.Constants.pi*SysData.fn)
    annotation (Placement(transformation(extent={{-20,-90},{0,-70}})));
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
  connect(Torque3.y,torque3. tau)
    annotation (Line(points={{49,-80},{62,-80}},   color={0,0,127}));
  connect(Load3.p,bus3_mt3. p)
    annotation (Line(points={{-10,32},{-10,50},{0,50}},color={0,0,255}));
  connect(torqueSensor3.tau,Motor3. mech_torque) annotation (Line(points={{36,-51},
          {36,-60},{16,-60},{16,-52}}, color={0,0,127}));
  connect(Motor3.flange,torqueSensor3. flange_a)
    annotation (Line(points={{20,-40},{34,-40}},
                                             color={0,0,0}));
  connect(aC2DC_and_DC2AC_uninitialized.n, Motor3.p)
    annotation (Line(points={{-40,-40},{0,-40}}, color={0,0,255}));
  connect(aC2DC_and_DC2AC_uninitialized.p, bus4_mt3.p) annotation (Line(points=
          {{-60,-40},{-100,-40},{-100,0},{80,0},{80,50},{40,50}}, color={0,0,
          255}));
  connect(m_ramp.y, aC2DC_and_DC2AC_uninitialized.m_input) annotation (Line(
        points={{-55,-80},{-45.4545,-80},{-45.4545,-52}}, color={0,0,127}));
  connect(f_ramp.y, Motor3.we)
    annotation (Line(points={{1,-80},{10,-80},{10,-52}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
            -160},{160,160}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-160,-160},{160,
            160}})),
    experiment(
      StopTime=100,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end MotorTypeIII_VSD_Ramp_StartUp;
