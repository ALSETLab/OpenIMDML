within OpenIMDML.Examples.MultiDomainExamples.ThreePhaseInductionMotors.VSDMotorStartUp;
model MotorTypeI_VSD_StartUp
  extends Modelica.Icons.Example;
  OpenIPSL.Electrical.Machines.PSSE.GENCLS inf1(
    V_b=16000,
    v_0=1.05,
    M_b=600000000,
    H=0)
    annotation (Placement(transformation(extent={{-220,-10},{-200,10}})));
  OpenIPSL.Electrical.Buses.Bus bus1_mt1(V_b=16000, v_0=1.05)
    annotation (Placement(transformation(extent={{-190,-10},{-170,10}})));
  OpenIPSL.Electrical.Buses.Bus bus2_mt1(V_b=230000)
    annotation (Placement(transformation(extent={{-150,-10},{-130,10}})));
  OpenIPSL.Electrical.Branches.PwLine line_mt1(
    R=0,
    X=0.02,
    G=0,
    B=0) annotation (Placement(transformation(extent={{-124,-14},{-96,14}})));
  OpenIPSL.Electrical.Buses.Bus bus3_mt1(V_b=230000)
    annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
  inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
    annotation (Placement(transformation(extent={{80,80},{140,120}})));
  OpenIPSL.Electrical.Buses.Bus bus4_mt1(V_b=23000)
    annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor1
    annotation (Placement(transformation(extent={{122,-10},{142,10}})));
  Modelica.Mechanics.Rotational.Components.Inertia load_inertia1(J=1)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={162,0})));
  Modelica.Mechanics.Rotational.Sources.Torque torque1
    annotation (Placement(transformation(extent={{152,-50},{172,-30}})));
  OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1_mt1(
    V_b=16000,
    Vn=16000,
    rT=0,
    xT=0.025)
             annotation (Placement(transformation(extent={{-170,-10},{
            -150,10}})));
  OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2_mt1(
    V_b=230000,
    Vn=230000,
    rT=0,
    xT=0.15) annotation (Placement(transformation(extent={{-70,-10},{-50,
            10}})));
  MultiDomainModels.Motors.ThreePhase.PSAT.MotorTypeI Motor1(V_b=23000)
    annotation (Placement(transformation(extent={{108,-10},{88,10}})));
  Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC
    aC2DC_and_DC2AC_uninitialized(V_b=23000,
    v_0=1)
    annotation (Placement(transformation(extent={{28,-10},{48,10}})));
  Controls.VariableSpeedDrive.Controls.VoltsHertz_Controller
    voltsHertz_Controller(
    V_b=23000,
    f_max=60,
    f_min=0,
    Kp=2)
    annotation (Placement(transformation(extent={{28,-60},{46,-40}})));
  OpenIPSL.Electrical.Loads.PSAT.PQ Load1(V_b=230000, P_0=500000000)
    annotation (Placement(transformation(extent={{-100,-38},{-80,-18}})));
  Modelica.Blocks.Sources.Ramp           Torque2(
    height=0.9*2*3.1415*60,
    duration=40,
    offset=0.1*2*3.1415*60)
    annotation (Placement(transformation(extent={{-44,-60},{-24,-40}})));
  Modelica.Blocks.Sources.RealExpression Torque1(y=-(0.1*(15/100)*(Motor1.s)
         + 0.5*(15/100)*(1 - Motor1.s)^2)*Motor1.T_b)
    annotation (Placement(transformation(extent={{120,-50},{140,-30}})));
equation
  connect(inf1.p,bus1_mt1. p)
    annotation (Line(points={{-200,0},{-180,0}},   color={0,0,255}));
  connect(bus2_mt1.p,line_mt1. p)
    annotation (Line(points={{-140,0},{-122.6,0}}, color={0,0,255}));
  connect(line_mt1.n,bus3_mt1. p)
    annotation (Line(points={{-97.4,0},{-80,0}}, color={0,0,255}));
  connect(torqueSensor1.flange_b,load_inertia1. flange_a)
    annotation (Line(points={{142,0},{152,0}},   color={0,0,0}));
  connect(torque1.flange,load_inertia1. flange_b) annotation (Line(points={{172,-40},
          {200,-40},{200,0},{172,0}},      color={0,0,0}));
  connect(bus1_mt1.p,tf1_mt1. p)
    annotation (Line(points={{-180,0},{-171,0}},  color={0,0,255}));
  connect(tf1_mt1.n,bus2_mt1. p)
    annotation (Line(points={{-149,0},{-140,0}}, color={0,0,255}));
  connect(bus3_mt1.p,tf2_mt1. p)
    annotation (Line(points={{-80,0},{-71,0}},
                                             color={0,0,255}));
  connect(tf2_mt1.n,bus4_mt1. p)
    annotation (Line(points={{-49,0},{-40,0}}, color={0,0,255}));
  connect(Motor1.flange,torqueSensor1. flange_a)
    annotation (Line(points={{108,0},{122,0}},
                                             color={0,0,0}));
  connect(torqueSensor1.tau,Motor1. mech_torque) annotation (Line(points={{124,-11},
          {124,-20},{104,-20},{104,-12}},
                                       color={0,0,127}));
  connect(aC2DC_and_DC2AC_uninitialized.n, Motor1.p)
    annotation (Line(points={{48,0},{88,0}},     color={0,0,255}));
  connect(voltsHertz_Controller.we, Motor1.we)
    annotation (Line(points={{50,-54},{98,-54},{98,-12}},  color={0,0,127}));
  connect(Motor1.wr, voltsHertz_Controller.motor_speed)
    annotation (Line(points={{92,-12},{92,-46},{50,-46}},color={0,0,127}));
  connect(aC2DC_and_DC2AC_uninitialized.Vc, voltsHertz_Controller.Vc)
    annotation (Line(points={{33.4545,-12},{33.4,-38}},   color={0,0,127}));
  connect(voltsHertz_Controller.m, aC2DC_and_DC2AC_uninitialized.m_input)
    annotation (Line(points={{42.6,-38},{42.5455,-38},{42.5455,-12}},    color={
          0,0,127}));
  connect(Load1.p, bus3_mt1.p)
    annotation (Line(points={{-90,-18},{-90,0},{-80,0}}, color={0,0,255}));
  connect(Torque2.y, voltsHertz_Controller.W_ref)
    annotation (Line(points={{-23,-50},{26,-50}},   color={0,0,127}));
  connect(Torque1.y, torque1.tau)
    annotation (Line(points={{141,-40},{150,-40}}, color={0,0,127}));
  connect(bus4_mt1.p, aC2DC_and_DC2AC_uninitialized.p)
    annotation (Line(points={{-40,0},{28,0}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-240,
            -160},{240,160}})),
                          Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-240,-160},{240,160}})),
    experiment(
      StopTime=100,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end MotorTypeI_VSD_StartUp;
