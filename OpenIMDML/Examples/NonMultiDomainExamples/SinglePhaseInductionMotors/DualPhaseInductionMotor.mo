within OpenIMDML.Examples.NonMultiDomainExamples.SinglePhaseInductionMotors;
model DualPhaseInductionMotor
  extends Modelica.Icons.Example;
  extends OpenIMDML.Examples.BaseClasses.ValidationPartial1(SysData(S_b=1000000));

  OpenIMDML.NonMultiDomain.Motors.SinglePhase.NMD_DPIM DPIM(
    M_b=15000000,
    V_b=230,
    init=2,
    switch_open_speed=0.2,
    Lmainr=0.000588,
    Lmain=0.0806,
    Lauxr=0.000909,
    Laux=0.196,
    Lr=0.0000047,
    Rmain=0.58,
    Rr=0.0000376,
    Raux=3.37,
    Cc(displayUnit="F") = 0.001,
    H=0.005,
    a=0.0039,
    b=0,
    c=0) annotation (Placement(transformation(extent={{40,-10},{20,10}})));
equation
  connect(load_bus.p, DPIM.p)
    annotation (Line(points={{-16,0},{20,0}}, color={0,0,255}));
  annotation (                                        experiment(
      StopTime=5,
      __Dymola_NumberOfIntervals=10000,
      __Dymola_Algorithm="Dassl"),
    Diagram(coordinateSystem(extent={{-120,-100},{120,100}})),
    Icon(coordinateSystem(extent={{-120,-100},{120,100}})));
end DualPhaseInductionMotor;
