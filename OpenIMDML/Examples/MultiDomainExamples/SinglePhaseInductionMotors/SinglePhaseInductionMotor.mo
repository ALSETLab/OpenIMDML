within OpenIMDML.Examples.MultiDomainExamples.SinglePhaseInductionMotors;
model SinglePhaseInductionMotor
  extends Modelica.Icons.Example;
  OpenIPSL.Electrical.Buses.Bus inf_bus(V_b=230)
    annotation (Placement(transformation(extent={{-76,-10},{-56,10}})));
  OpenIPSL.Electrical.Machines.PSSE.GENCLS gENCLS(V_b=230)
    annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
  OpenIPSL.Electrical.Buses.Bus load_bus(V_b=230)
    annotation (Placement(transformation(extent={{-6,-10},{14,10}})));
  OpenIPSL.Electrical.Branches.PwLine Line(
    G=0,
    B=0,
    R=2.50000E-3,
    X=2.50000E-3)
    annotation (Placement(transformation(extent={{-42,6},{-22,26}})));
  inner OpenIPSL.Electrical.SystemBase SysData(fn=60, S_b=100000000) annotation (Placement(transformation(extent={{40,60},
            {80,80}})));
  OpenIMDML.MultiDomainModels.Motors.SinglePhase.MD_SPIM SPIM(
    M_b(displayUnit="V.A") = 3500,
    V_b=230,
    P_0=1000,
    Q_0=5000,
    v_0=1,
    angle_0=0,
    N=1,
    H=0.1,
    R1=0.01,
    R2=0.05,
    X1=0.01,
    X2=0.01,
    Xm=0.1)
         annotation (Placement(transformation(extent={{38,-10},{18,10}})));
  OpenIPSL.Electrical.Events.PwFault Fault(
    R=0.1,
    X=0.1,
    t1=100,
    t2=101) annotation (Placement(transformation(extent={{-20,-50},{0,-30}})));
  OpenIPSL.Electrical.Branches.PwLine Line1(
    G=0,
    B=0,
    R=0.5*2.50000E-3,
    X=0.5*2.50000E-3)
    annotation (Placement(transformation(extent={{-58,-26},{-38,-6}})));
  OpenIPSL.Electrical.Branches.PwLine Line2(
    G=0,
    B=0,
    R=0.5*2.50000E-3,
    X=0.5*2.50000E-3)
    annotation (Placement(transformation(extent={{-30,-26},{-10,-6}})));
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor1
    annotation (Placement(transformation(extent={{50,-10},{70,10}})));
  Modelica.Mechanics.Rotational.Components.Inertia load_inertia1(J=0.01)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,0})));
  Modelica.Mechanics.Rotational.Sources.Torque torque1
    annotation (Placement(transformation(extent={{80,-50},{100,-30}})));
  Modelica.Blocks.Sources.RealExpression Torque1(y=-0.0001)
    annotation (Placement(transformation(extent={{40,-50},{60,-30}})));
equation
  connect(gENCLS.p, inf_bus.p)
    annotation (Line(points={{-80,0},{-66,0}}, color={0,0,255}));
  connect(Line.n, load_bus.p)
    annotation (Line(points={{-23,16},{-8,16},{-8,0},{4,0}}, color={0,0,255}));
  connect(inf_bus.p, Line.p) annotation (Line(points={{-66,0},{-60,0},{-60,16},
          {-41,16}},color={0,0,255}));
  connect(load_bus.p, SPIM.p)
    annotation (Line(points={{4,0},{18,0}},  color={0,0,255}));
  connect(Line1.p, Line.p) annotation (Line(points={{-57,-16},{-60,-16},{-60,16},
          {-41,16}},color={0,0,255}));
  connect(Line1.n, Line2.p)
    annotation (Line(points={{-39,-16},{-29,-16}},
                                                color={0,0,255}));
  connect(Line2.n, load_bus.p) annotation (Line(points={{-11,-16},{-8,-16},{-8,
          0},{4,0}},
                   color={0,0,255}));
  connect(Fault.p, Line2.p) annotation (Line(points={{-21.6667,-40},{-34,-40},{
          -34,-16},{-29,-16}},
                          color={0,0,255}));
  connect(torqueSensor1.flange_b,load_inertia1. flange_a)
    annotation (Line(points={{70,0},{80,0}},     color={0,0,0}));
  connect(torque1.flange,load_inertia1. flange_b) annotation (Line(points={{100,-40},
          {110,-40},{110,0},{100,0}},      color={0,0,0}));
  connect(Torque1.y,torque1. tau)
    annotation (Line(points={{61,-40},{78,-40}}, color={0,0,127}));
  connect(SPIM.flange, torqueSensor1.flange_a)
    annotation (Line(points={{38,0},{50,0}}, color={0,0,0}));
  connect(torqueSensor1.tau, SPIM.mech_torque) annotation (Line(points={{52,-11},
          {52,-20},{34,-20},{34,-12}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,
            -100},{120,100}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},{120,
            100}})),
    experiment(
      StopTime=5,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end SinglePhaseInductionMotor;
