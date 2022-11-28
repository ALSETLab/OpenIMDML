within OpenIMDML.Examples.NonMultiDomainExamples;
package NonMultiDomainSinglePhaseInductionMotors "Simple systems to test out the non multi-domain single and dual phase motor models"
  extends Modelica.Icons.ExamplesPackage;

  model NonMultiDomainSPIM "Non Multi-Domain validation example for the single-phase induction motor (SPIM) model"
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
    annotation (preferredView = "info",Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},
              {120,100}})),                                        Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},{120,100}})),
      experiment(
        StopTime=5,
        __Dymola_NumberOfIntervals=5000,
        __Dymola_Algorithm="Dassl"),
      Documentation(info="<html>
<p>The example NonMultiDomainSPIM contains a non multi-domain single-phase induction motor (SPIM) model which drives a mechanical load that is described with a mechanical torque equation described in the equation section the model. The SPIM induction motor circuitry and operation are in steady-state, which explains the lack of start-up capability of this model. </p>
<p>The reader can find more details about the single-phase induction motor modeling in Chapter 09 - <i>Fitzgerald, Arthur Eugene, et al. Electric machinery. Vol. 5. New York: McGRAW-hill, 2003.</i> </p>
<p>Simulate the system for 5 seconds. Variables of interest are:</p>
<ul>
<li><code>SPIM.s</code></li>
<li><code>SPIM.wr</code></li>
<li><code>SPIM.P</code></li>
<li><code>SPIM.Q</code></li>
<li><code>SPIM.Pc</code></li>
<li><code>SPIM.Qc</code></li>
</ul>
</html>"));
  end NonMultiDomainSPIM;

  model NonMultiDomainDPIM "Non Multi-Domain validation example for the single-phase induction motor (SPIM) model"
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
    annotation (preferredView = "info",                                        experiment(
        StopTime=5,
        __Dymola_NumberOfIntervals=10000,
        __Dymola_Algorithm="Dassl"),
      Diagram(coordinateSystem(extent={{-120,-100},{120,100}})),
      Icon(coordinateSystem(extent={{-120,-100},{120,100}})),
      Documentation(info="<html>
<p>The example NonMultiDomainDPIM contains a non multi-domain dual-phase induction motor (SPIM) model which drives a mechanical load that is described with a mechanical torque equation described in the equation section the model. The DPIM induction motor circuitry is in steady-state, while the operation of the motor can be toggled between steady-state operation and motor start-up.</p>
<p>There are two different modes of operation: Start-up (Sup = 2), and Steady-state (Sup = 1). The user can also select the configuration of the auxiliary winding circuitry, either split-phase or capacity-start. </p>
<p>The reader can find more details about the dual-phase induction motor modeling in Chapter 09 - <i>Fitzgerald, Arthur Eugene, et al. Electric machinery. Vol. 5. New York: McGRAW-hill, 2003.</i> </p>
<p>Simulate the system for 10 seconds. Variables of interest are:</p>
<ul>
<li><code>DPIM.s</code></li>
<li><code>DPIM.wr</code></li>
<li><code>DPIM.P</code></li>
<li><code>DPIM.Q</code></li>
<li><code>DPIM.Pc</code></li>
<li><code>DPIM.Qc</code></li>
</ul>
</html>"));
  end NonMultiDomainDPIM;
end NonMultiDomainSinglePhaseInductionMotors;
