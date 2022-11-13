within OpenIMDML.Examples.NonMultiDomainExamples.SinglePhaseInductionMotors;
model SinglePhaseInductionMotor
  extends Modelica.Icons.Example;
  extends OpenIMDML.Examples.BaseClasses.ValidationPartial1;
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
    c=0.01) annotation (Placement(transformation(extent={{40,-10},{20,10}})));
equation
  connect(SPIM.p, load_bus.p)
    annotation (Line(points={{20,0},{-16,0}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,
            -100},{120,100}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},{120,
            100}})),
    experiment(
      StopTime=5,
      __Dymola_NumberOfIntervals=5000,
      __Dymola_Algorithm="Dassl"));
end SinglePhaseInductionMotor;
