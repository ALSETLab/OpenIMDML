within OpenIMDML.Examples.MultiDomainThreePhaseInductionMotors.MotorValidation;
model MotorTypeI_Validation
  extends Modelica.Icons.Example;
  OpenIPSL.Electrical.Machines.PSSE.GENCLS inf1(
    V_b=16000,
    v_0=1.05,
    M_b=600000000,
    H=0)
    annotation (Placement(transformation(extent={{-148,-10},{-128,10}})));
  OpenIPSL.Electrical.Buses.Bus bus1_mt1(V_b=16000, v_0=1.05)
    annotation (Placement(transformation(extent={{-118,-10},{-98,10}})));
  OpenIPSL.Electrical.Buses.Bus bus2_mt1(V_b=230000)
    annotation (Placement(transformation(extent={{-78,-10},{-58,10}})));
  OpenIPSL.Electrical.Branches.PwLine line_mt1(
    R=0,
    X=0.02,
    G=0,
    B=0) annotation (Placement(transformation(extent={{-52,-14},{-24,14}})));
  OpenIPSL.Electrical.Buses.Bus bus3_mt1(V_b=230000)
    annotation (Placement(transformation(extent={{-18,-10},{2,10}})));
  inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
    annotation (Placement(transformation(extent={{80,80},{140,120}})));
  OpenIPSL.Electrical.Buses.Bus bus4_mt1(V_b=23000)
    annotation (Placement(transformation(extent={{22,-10},{42,10}})));
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
  OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1_mt1(
    V_b=16000,
    Vn=16000,
    rT=0,
    xT=0.025)
             annotation (Placement(transformation(extent={{-98,-10},{-78,10}})));
  OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2_mt1(
    V_b=230000,
    Vn=230000,
    rT=0,
    xT=0.15) annotation (Placement(transformation(extent={{2,-10},{22,10}})));
  OpenIPSL.Electrical.Loads.PSAT.PQ Load1(V_b=230000, P_0=500000000)
    annotation (Placement(transformation(extent={{-28,-40},{-8,-20}})));
  Modelica.Blocks.Sources.RealExpression SS1(y=2*Modelica.Constants.pi*(SysData.fn))
                annotation (Placement(transformation(extent={{12,-46},{32,-26}})));
  MultiDomain.Motors.ThreePhase.PSAT.MotorTypeI Motor1(V_b=23000)
    annotation (Placement(transformation(extent={{68,-10},{48,10}})));
equation
  connect(inf1.p,bus1_mt1. p)
    annotation (Line(points={{-128,0},{-108,0}},   color={0,0,255}));
  connect(bus2_mt1.p,line_mt1. p)
    annotation (Line(points={{-68,0},{-50.6,0}},   color={0,0,255}));
  connect(line_mt1.n,bus3_mt1. p)
    annotation (Line(points={{-25.4,0},{-8,0}},  color={0,0,255}));
  connect(torqueSensor1.flange_b,load_inertia1. flange_a)
    annotation (Line(points={{102,0},{112,0}},   color={0,0,0}));
  connect(torque1.flange,load_inertia1. flange_b) annotation (Line(points={{132,-40},
          {142,-40},{142,0},{132,0}},      color={0,0,0}));
  connect(bus1_mt1.p,tf1_mt1. p)
    annotation (Line(points={{-108,0},{-99,0}},   color={0,0,255}));
  connect(tf1_mt1.n,bus2_mt1. p)
    annotation (Line(points={{-77,0},{-68,0}},   color={0,0,255}));
  connect(bus3_mt1.p,tf2_mt1. p)
    annotation (Line(points={{-8,0},{1,0}},  color={0,0,255}));
  connect(tf2_mt1.n,bus4_mt1. p)
    annotation (Line(points={{23,0},{32,0}},   color={0,0,255}));
  connect(Torque1.y,torque1. tau)
    annotation (Line(points={{93,-40},{110,-40}},color={0,0,127}));
  connect(Load1.p,bus3_mt1. p)
    annotation (Line(points={{-18,-20},{-18,0},{-8,0}}, color={0,0,255}));
  connect(bus4_mt1.p, Motor1.p)
    annotation (Line(points={{32,0},{48,0}}, color={0,0,255}));
  connect(Motor1.flange, torqueSensor1.flange_a)
    annotation (Line(points={{68,0},{82,0}}, color={0,0,0}));
  connect(torqueSensor1.tau, Motor1.mech_torque) annotation (Line(points={{84,-11},
          {84,-20},{64,-20},{64,-12}}, color={0,0,127}));
  connect(SS1.y, Motor1.we)
    annotation (Line(points={{33,-36},{58,-36},{58,-12}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-160},
            {160,160}})), Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-160,-160},{160,160}})),
    experiment(
      StopTime=15,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end MotorTypeI_Validation;
