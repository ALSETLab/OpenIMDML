within OpenIMDML.Examples.MultiDomainThreePhaseInductionMotors.VSDMotorStartUp;
model MotorTypeI_VSD_Ramp_StartUp
  extends Modelica.Icons.Example;
  OpenIPSL.Electrical.Machines.PSSE.GENCLS inf1(
    V_b=16000,
    v_0=1.05,
    M_b=600000000,
    H=0)
    annotation (Placement(transformation(extent={{-140,40},{-120,60}})));
  OpenIPSL.Electrical.Buses.Bus bus1_mt1(V_b=16000, v_0=1.05)
    annotation (Placement(transformation(extent={{-110,40},{-90,60}})));
  OpenIPSL.Electrical.Buses.Bus bus2_mt1(V_b=230000)
    annotation (Placement(transformation(extent={{-70,40},{-50,60}})));
  OpenIPSL.Electrical.Branches.PwLine line_mt1(
    R=0,
    X=0.02,
    G=0,
    B=0) annotation (Placement(transformation(extent={{-44,36},{-16,64}})));
  OpenIPSL.Electrical.Buses.Bus bus3_mt1(V_b=230000)
    annotation (Placement(transformation(extent={{-10,40},{10,60}})));
  inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
    annotation (Placement(transformation(extent={{80,80},{140,120}})));
  OpenIPSL.Electrical.Buses.Bus bus4_mt1(V_b=23000)
    annotation (Placement(transformation(extent={{30,40},{50,60}})));
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor1
    annotation (Placement(transformation(extent={{34,-50},{54,-30}})));
  Modelica.Mechanics.Rotational.Components.Inertia load_inertia1(J=1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={74,-40})));
  Modelica.Mechanics.Rotational.Sources.Torque torque1
    annotation (Placement(transformation(extent={{64,-90},{84,-70}})));
  OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1_mt1(
    V_b=16000,
    Vn=16000,
    rT=0,
    xT=0.025)
             annotation (Placement(transformation(extent={{-90,40},{-70,60}})));
  OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2_mt1(
    V_b=230000,
    Vn=230000,
    rT=0,
    xT=0.15) annotation (Placement(transformation(extent={{10,40},{30,60}})));
  MultiDomain.Motors.ThreePhase.PSAT.MotorTypeI Motor1(V_b=23000)
    annotation (Placement(transformation(extent={{20,-50},{0,-30}})));
  Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC
    aC2DC_and_DC2AC_uninitialized(V_b=23000)
    annotation (Placement(transformation(extent={{-60,-50},{-40,-30}})));
  OpenIPSL.Electrical.Loads.PSAT.PQ Load1(V_b=230000, P_0=500000000)
    annotation (Placement(transformation(extent={{-20,12},{0,32}})));
  Modelica.Blocks.Sources.RealExpression Torque1(y=-(0.5*(15/100)*(1 - Motor1.s)
        ^2)*Motor1.T_b)
    annotation (Placement(transformation(extent={{28,-90},{48,-70}})));
  Modelica.Blocks.Sources.Ramp f_ramp(
    height=0.99*2*Modelica.Constants.pi*SysData.fn,
    duration=100,
    offset=0.01*2*Modelica.Constants.pi*SysData.fn)
    annotation (Placement(transformation(extent={{-20,-90},{0,-70}})));
  Modelica.Blocks.Sources.Ramp m_ramp(
    height=0.99,
    duration=100,
    offset=0.01)
    annotation (Placement(transformation(extent={{-76,-90},{-56,-70}})));
equation
  connect(inf1.p,bus1_mt1. p)
    annotation (Line(points={{-120,50},{-100,50}}, color={0,0,255}));
  connect(bus2_mt1.p,line_mt1. p)
    annotation (Line(points={{-60,50},{-42.6,50}}, color={0,0,255}));
  connect(line_mt1.n,bus3_mt1. p)
    annotation (Line(points={{-17.4,50},{0,50}}, color={0,0,255}));
  connect(torqueSensor1.flange_b,load_inertia1. flange_a)
    annotation (Line(points={{54,-40},{64,-40}}, color={0,0,0}));
  connect(torque1.flange,load_inertia1. flange_b) annotation (Line(points={{84,-80},
          {94,-80},{94,-40},{84,-40}},     color={0,0,0}));
  connect(bus1_mt1.p,tf1_mt1. p)
    annotation (Line(points={{-100,50},{-91,50}}, color={0,0,255}));
  connect(tf1_mt1.n,bus2_mt1. p)
    annotation (Line(points={{-69,50},{-60,50}}, color={0,0,255}));
  connect(bus3_mt1.p,tf2_mt1. p)
    annotation (Line(points={{0,50},{9,50}}, color={0,0,255}));
  connect(tf2_mt1.n,bus4_mt1. p)
    annotation (Line(points={{31,50},{40,50}}, color={0,0,255}));
  connect(Motor1.flange,torqueSensor1. flange_a)
    annotation (Line(points={{20,-40},{34,-40}},
                                             color={0,0,0}));
  connect(torqueSensor1.tau,Motor1. mech_torque) annotation (Line(points={{36,-51},
          {36,-60},{16,-60},{16,-52}}, color={0,0,127}));
  connect(aC2DC_and_DC2AC_uninitialized.n, Motor1.p)
    annotation (Line(points={{-40,-40},{0,-40}}, color={0,0,255}));
  connect(bus4_mt1.p, aC2DC_and_DC2AC_uninitialized.p) annotation (Line(points={{40,50},
          {80,50},{80,0},{-100,0},{-100,-40},{-60,-40}},         color={0,0,255}));
  connect(Load1.p, bus3_mt1.p)
    annotation (Line(points={{-10,32},{-10,50},{0,50}},  color={0,0,255}));
  connect(Torque1.y, torque1.tau)
    annotation (Line(points={{49,-80},{62,-80}}, color={0,0,127}));
  connect(m_ramp.y, aC2DC_and_DC2AC_uninitialized.m_input) annotation (Line(
        points={{-55,-80},{-45.4545,-80},{-45.4545,-52}}, color={0,0,127}));
  connect(f_ramp.y, Motor1.we)
    annotation (Line(points={{1,-80},{10,-80},{10,-52}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-160},
            {160,160}})), Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-160,-160},{160,160}})),
    experiment(
      StopTime=15,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end MotorTypeI_VSD_Ramp_StartUp;
