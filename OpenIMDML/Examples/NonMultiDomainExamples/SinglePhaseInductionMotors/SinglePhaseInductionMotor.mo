within OpenIMDML.Examples.NonMultiDomainExamples.SinglePhaseInductionMotors;
model SinglePhaseInductionMotor
  extends Modelica.Icons.Example;
  OpenIPSL.Electrical.Buses.Bus inf_bus(V_b=230)
    annotation (Placement(transformation(extent={{-36,-10},{-16,10}})));
  OpenIPSL.Electrical.Machines.PSSE.GENCLS gENCLS(V_b=230)
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  OpenIPSL.Electrical.Buses.Bus load_bus(V_b=230)
    annotation (Placement(transformation(extent={{32,-10},{52,10}})));
  OpenIPSL.Electrical.Branches.PwLine Line(
    G=0,
    B=0,
    R=2.50000E-3,
    X=2.50000E-3)
    annotation (Placement(transformation(extent={{-2,6},{18,26}})));
  inner OpenIPSL.Electrical.SystemBase SysData(fn=60, S_b=100000000) annotation (Placement(transformation(extent={{40,30},
            {80,50}})));
  OpenIMDML.NonMultiDomain.Motors.SinglePhase.NMD_SPIM SPIM(
    V_b=230,
    P_0=40000,
    Q_0=40000,
    v_0=1,
    angle_0=0,
    R1=0.001,
    R2=0.005,
    X1=0.01,
    X2=0.01,
    Xm=0.1,
    H=0.1,
    a=0.01,
    b=0.02,
    c=0.01) annotation (Placement(transformation(extent={{80,-10},{60,10}})));
  OpenIPSL.Electrical.Events.PwFault Fault(
    R=0.1,
    X=0.1,
    t1=5,
    t2=5.1) annotation (Placement(transformation(extent={{40,-46},{60,-26}})));
  OpenIPSL.Electrical.Branches.PwLine Line1(
    G=0,
    B=0,
    R=0.5*2.50000E-3,
    X=0.5*2.50000E-3)
    annotation (Placement(transformation(extent={{-18,-26},{2,-6}})));
  OpenIPSL.Electrical.Branches.PwLine Line2(
    G=0,
    B=0,
    R=0.5*2.50000E-3,
    X=0.5*2.50000E-3)
    annotation (Placement(transformation(extent={{10,-26},{30,-6}})));
equation
  connect(gENCLS.p, inf_bus.p)
    annotation (Line(points={{-40,0},{-26,0}}, color={0,0,255}));
  connect(Line.n, load_bus.p)
    annotation (Line(points={{17,16},{32,16},{32,0},{42,0}}, color={0,0,255}));
  connect(inf_bus.p, Line.p) annotation (Line(points={{-26,0},{-20,0},{-20,16},
          {-1,16}}, color={0,0,255}));
  connect(load_bus.p, SPIM.p)
    annotation (Line(points={{42,0},{60,0}}, color={0,0,255}));
  connect(Line1.p, Line.p) annotation (Line(points={{-17,-16},{-20,-16},{-20,16},
          {-1,16}}, color={0,0,255}));
  connect(Line1.n, Line2.p)
    annotation (Line(points={{1,-16},{11,-16}}, color={0,0,255}));
  connect(Line2.n, load_bus.p) annotation (Line(points={{29,-16},{32,-16},{32,0},
          {42,0}}, color={0,0,255}));
  connect(Fault.p, Line2.p) annotation (Line(points={{38.3333,-36},{6,-36},{6,
          -16},{11,-16}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=5,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end SinglePhaseInductionMotor;
