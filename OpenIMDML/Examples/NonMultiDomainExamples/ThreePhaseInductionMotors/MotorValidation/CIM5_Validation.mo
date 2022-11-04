within OpenIMDML.Examples.NonMultiDomainExamples.ThreePhaseInductionMotors.MotorValidation;
model CIM5_Validation
  extends Modelica.Icons.Example;
  OpenIPSL.Electrical.Machines.PSSE.GENCLS inf1(
    V_b=16000,
    v_0=1.05,
    M_b=600000000,
    H=0)
    annotation (Placement(transformation(extent={{-120,-10},{-100,10}})));
  OpenIPSL.Electrical.Buses.Bus bus1_mt1(V_b=16000, v_0=1.05)
    annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
  OpenIPSL.Electrical.Buses.Bus bus2_mt1(V_b=230000)
    annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
  OpenIPSL.Electrical.Branches.PwLine line_mt1(
    R=0,
    X=0.02,
    G=0,
    B=0) annotation (Placement(transformation(extent={{-24,-14},{4,14}})));
  OpenIPSL.Electrical.Buses.Bus bus3_mt1(V_b=230000)
    annotation (Placement(transformation(extent={{10,-10},{30,10}})));
  inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
    annotation (Placement(transformation(extent={{80,40},{140,80}})));
  OpenIPSL.Electrical.Buses.Bus bus4_mt1(V_b=23000)
    annotation (Placement(transformation(extent={{50,-10},{70,10}})));
  OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1_mt1(
    V_b=16000,
    Vn=16000,
    rT=0,
    xT=0.025)
             annotation (Placement(transformation(extent={{-70,-10},{-50,
            10}})));
  OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2_mt1(
    V_b=230000,
    Vn=230000,
    rT=0,
    xT=0.15) annotation (Placement(transformation(extent={{30,-10},{50,10}})));
  OpenIPSL.Electrical.Loads.PSAT.PQ Load1(V_b=230000, P_0=500000000)
    annotation (Placement(transformation(extent={{0,-40},{20,-20}})));
  Modelica.Blocks.Sources.RealExpression SS1(y=2*Modelica.Constants.pi*(
        SysData.fn))
                annotation (Placement(transformation(extent={{40,-46},{60,
            -26}})));
  NonMultiDomain.Motors.ThreePhase.PSSE.NMD_CIM5 CIM5(
    V_b=23000,
    Ctrl=true,
    T_nom=0.5)
    annotation (Placement(transformation(extent={{100,-10},{80,10}})));
equation
  connect(inf1.p,bus1_mt1. p)
    annotation (Line(points={{-100,0},{-80,0}},    color={0,0,255}));
  connect(bus2_mt1.p,line_mt1. p)
    annotation (Line(points={{-40,0},{-22.6,0}},   color={0,0,255}));
  connect(line_mt1.n,bus3_mt1. p)
    annotation (Line(points={{2.6,0},{20,0}},    color={0,0,255}));
  connect(bus1_mt1.p,tf1_mt1. p)
    annotation (Line(points={{-80,0},{-71,0}},    color={0,0,255}));
  connect(tf1_mt1.n,bus2_mt1. p)
    annotation (Line(points={{-49,0},{-40,0}},   color={0,0,255}));
  connect(bus3_mt1.p,tf2_mt1. p)
    annotation (Line(points={{20,0},{29,0}}, color={0,0,255}));
  connect(tf2_mt1.n,bus4_mt1. p)
    annotation (Line(points={{51,0},{60,0}},   color={0,0,255}));
  connect(Load1.p,bus3_mt1. p)
    annotation (Line(points={{10,-20},{10,0},{20,0}},   color={0,0,255}));
  connect(bus4_mt1.p, CIM5.p)
    annotation (Line(points={{60,0},{80,0}}, color={0,0,255}));
  connect(SS1.y, CIM5.we)
    annotation (Line(points={{61,-36},{96,-36},{96,-12}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
            -100},{160,100}})),
                          Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-160,-100},{160,100}})),
    experiment(
      StopTime=10,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end CIM5_Validation;
