within OpenIMDML.Examples.NonMultiDomainExamples;
package NonMultiDomainThreePhaseInductionMotors "Simple systems to test out the non multi-domain three-phase motor models"
  extends Modelica.Icons.ExamplesPackage;

  package NonMultiDomainThreePhaseMotorValidation "Non Multi-domain three-phase motor validation examples"
    extends Modelica.Icons.ExamplesPackage;

    model NonMultiDomainTypeI "Non Multi-Domain validation example for the Type I motor model"
      extends Modelica.Icons.Example;
      extends OpenIMDML.Examples.BaseClasses.ValidationPartial2;
      Modelica.Blocks.Sources.RealExpression SS1(y = 2*Modelica.Constants.pi*(SysData.fn + 10)) annotation (
        Placement(transformation(extent = {{40, -46}, {60, -26}})));
      NonMultiDomain.Motors.ThreePhase.PSAT.NMD_MotorTypeI Motor1(N = 1,
        Sup= false,
        V_b=23000,                                                                                 a = 0.1, b = 0.1, c = 0.1) annotation (
        Placement(transformation(extent = {{100, -10}, {80, 10}})));
    equation
      connect(SS1.y, Motor1.we) annotation (
        Line(points = {{61, -36}, {96, -36}, {96, -12}}, color = {0, 0, 127}));
      connect(bus4_mt1.p, Motor1.p) annotation (
        Line(points = {{20, 0}, {80, 0}}, color = {0, 0, 255}));
      annotation (
        preferredView = "info",
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-160, -100}, {160, 100}})),
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-160, -100}, {160, 100}})),
        Documentation(info = "<html>
<p>The example NonMultiDomainTypeI contains a non multi-domain three-phase type I induction motor model which drives a mechanical load that is described with a mechanical torque equation described in the equation section the model. 
The three-phase induction motor type I is based on the non multi-domain model from <i>Milano, Federico. Power system modelling and scripting. Springer Science & Business Media, 2010. </i><\\p>
<p>In order to validate the non multi-domain model, this validation example reproduces Example 11.7 Motor Startup from <i>Chow, Joe H., and Juan J. Sanchez-Gasca. Power system modeling, computation, and control. John Wiley & Sons, 2020.</i> but with 
a different load torque profile because the mechanical torque equations do not match due to modeling differences. Although the mechanical torque is not the same, the system itsel if and the startup process is similar. </p>
<p>Simulate the system for 10 seconds. Variables of interest are:</p>
<ul>
<li><code>Motor1.s</code></li>
<li><code>Motor1.wr</code></li>
<li><code>Motor1.P</code></li>
<li><code>Motor1.Q</code></li>
<li><code>Motor1.Pmotor</code></li>
</ul>
</html>"),
        experiment(StopTime = 5, __Dymola_Algorithm = "Dassl"));
    end NonMultiDomainTypeI;

    model NonMultiDomainTypeIII "Non Multi-Domain validation example for the Type III motor model"
      extends Modelica.Icons.Example;
      extends OpenIMDML.Examples.BaseClasses.ValidationPartial2;
      Modelica.Blocks.Sources.RealExpression SS3(y = 2*Modelica.Constants.pi*(SysData.fn)) annotation (
        Placement(transformation(extent = {{40, -46}, {60, -26}})));
      NonMultiDomain.Motors.ThreePhase.PSAT.NMD_MotorTypeIII Motor3(
        V_b=23000,
        Sup=false,                                                                           a = 0.1, b = 0.1, c = 0.1) annotation (
        Placement(transformation(extent = {{100, -10}, {80, 10}})));
    equation
      connect(SS3.y, Motor3.we) annotation (
        Line(points = {{61, -36}, {96, -36}, {96, -12}}, color = {0, 0, 127}));
      connect(bus4_mt1.p, Motor3.p) annotation (
        Line(points = {{20, 0}, {80, 0}}, color = {0, 0, 255}));
      annotation (
        preferredView = "info",
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-160, -100}, {160, 100}})),
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-160, -100}, {160, 100}})),
        Documentation(info = "<html>
<p>The example NonMultiDomainTypeIII contains a non multi-domain three-phase type III induction motor model which drives a mechanical load that is described with a mechanical torque equation described in the equation section the model. 
The three-phase induction motor type I is based on the non multi-domain model from <i>Milano, Federico. Power system modelling and scripting. Springer Science & Business Media, 2010. </i><\\p>
<p>In order to validate the non multi-domain model, this validation example reproduces Example 11.7 Motor Startup from <i>Chow, Joe H., and Juan J. Sanchez-Gasca. Power system modeling, computation, and control. John Wiley & Sons, 2020.</i> but with 
a different load torque profile because the mechanical torque equations do not match due to modeling differences. Although the mechanical torque is not the same, the system itsel if and the startup process is similar. </p>
<p>Simulate the system for 10 seconds. Variables of interest are:</p>
<ul>
<li><code>Motor3.s</code></li>
<li><code>Motor3.wr</code></li>
<li><code>Motor3.P</code></li>
<li><code>Motor3.Q</code></li>
<li><code>Motor3.Pmotor</code></li>
</ul>
</html>"));
    end NonMultiDomainTypeIII;

    model NonMultiDomainTypeV "Non Multi-Domain validation example for the Type V motor model"
      extends Modelica.Icons.Example;
      extends OpenIMDML.Examples.BaseClasses.ValidationPartial2;
      Modelica.Blocks.Sources.RealExpression SS5(y = 2*Modelica.Constants.pi*(SysData.fn)) annotation (
        Placement(transformation(extent = {{40, -46}, {60, -26}})));
      NonMultiDomain.Motors.ThreePhase.PSAT.NMD_MotorTypeV Motor5(
        Sup= false,
        V_b=23000,                                                                          a = 0.1, b = 0.1, c = 0.1) annotation (
        Placement(transformation(extent = {{100, -10}, {80, 10}})));
    equation
      connect(SS5.y, Motor5.we) annotation (
        Line(points = {{61, -36}, {96, -36}, {96, -12}}, color = {0, 0, 127}));
      connect(bus4_mt1.p, Motor5.p) annotation (
        Line(points = {{20, 0}, {80, 0}}, color = {0, 0, 255}));
      annotation (
        preferredView = "info",
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-160, -100}, {160, 100}})),
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-160, -100}, {160, 100}})),
        Documentation(info = "<html>
<p>The example NonMultiDomainTypeV contains a non multi-domain three-phase type V induction motor model which drives a mechanical load that is described with a mechanical torque equation described in the equation section the model. 
The three-phase induction motor type I is based on the non multi-domain model from <i>Milano, Federico. Power system modelling and scripting. Springer Science & Business Media, 2010. </i><\\p>
<p>In order to validate the non multi-domain model, this validation example reproduces Example 11.7 Motor Startup from <i>Chow, Joe H., and Juan J. Sanchez-Gasca. Power system modeling, computation, and control. John Wiley & Sons, 2020.</i> but with 
a different load torque profile because the mechanical torque equations do not match due to modeling differences. Although the mechanical torque is not the same, the system itsel if and the startup process is similar. </p>
<p>Simulate the system for 10 seconds. Variables of interest are:</p>
<ul>
<li><code>Motor5.s</code></li>
<li><code>Motor5.wr</code></li>
<li><code>Motor5.P</code></li>
<li><code>Motor5.Q</code></li>
<li><code>Motor5.Pmotor</code></li>
</ul>
</html>"));
    end NonMultiDomainTypeV;

    model NonMultiDomainCIM5 "Non Multi-Domain validation example for the CIM5 motor model"
      extends Modelica.Icons.Example;
      extends OpenIMDML.Examples.BaseClasses.ValidationPartial2(bus2_mt1(v_0 = 1.031491, angle_0 = -0.11763519158442), bus3_mt1(v_0 = 1.026884, angle_0 = -0.21022037054666), Load1(v_0 = 1.026884, angle_0 = -0.21022037054666), bus4_mt1(v_0 = 1.026884, angle_0 = -0.21022037054666));
      Modelica.Blocks.Sources.RealExpression SSCIM5(y = 2*Modelica.Constants.pi*(SysData.fn)) annotation (
        Placement(transformation(extent = {{40, -46}, {60, -26}})));
      NonMultiDomain.Motors.ThreePhase.PSSE.NMD_CIM5 CIM5(Mtype = 2, Sup = false, T_nom = 0.5, V_b = 23000) annotation (
        Placement(transformation(extent = {{100, -10}, {80, 10}})));
    equation
      connect(SSCIM5.y, CIM5.we) annotation (
        Line(points = {{61, -36}, {96, -36}, {96, -12}}, color = {0, 0, 127}));
      connect(bus4_mt1.p, CIM5.p) annotation (
        Line(points = {{20, 0}, {80, 0}}, color = {0, 0, 255}));
      annotation (
        preferredView = "info",
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-160, -100}, {160, 100}})),
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-160, -100}, {160, 100}})),
        experiment(StopTime = 10, __Dymola_NumberOfIntervals = 5000, __Dymola_Algorithm = "Dassl"),
        Documentation(info = "<html>
<p>The example NonMultiDomainCIM5 contains a non multi-domain three-phase CIM5 induction motor model which drives a mechanical load that is described with a mechanical torque equation described in the equation section the model. 
The three-phase induction motor type I is based on the non multi-domain model from <i>Milano, Federico. Power system modelling and scripting. Springer Science & Business Media, 2010. </i><\\p>
<p>In order to validate the non multi-domain model, this validation example reproduces Example 11.7 Motor Startup from <i>Chow, Joe H., and Juan J. Sanchez-Gasca. Power system modeling, computation, and control. John Wiley & Sons, 2020.</i> but with 
a different load torque profile because the mechanical torque equations do not match due to modeling differences. Although the mechanical torque is not the same, the system itsel if and the startup process is similar. </p>
<p>Simulate the system for 10 seconds. Variables of interest are:</p>
<ul>
<li><code>CIM5.s</code></li>
<li><code>CIM5.wr</code></li>
<li><code>CIM5.P</code></li>
<li><code>CIM5.Q</code></li>
<li><code>CIM5.Pmotor</code></li>
</ul>
</html>"));
    end NonMultiDomainCIM5;

    model NonMultiDomainCIM6 "Non Multi-Domain validation example for the CIM6 motor model"
      extends Modelica.Icons.Example;
      extends OpenIMDML.Examples.BaseClasses.ValidationPartial2;
      Modelica.Blocks.Sources.RealExpression SSCIM6(y = 2*Modelica.Constants.pi*(SysData.fn)) annotation (
        Placement(transformation(extent = {{40, -46}, {60, -26}})));
      NonMultiDomain.Motors.ThreePhase.PSSE.NMD_CIM6 CIM6(Ctrl = true, Mtype = 2, Sup = false, T_nom = 0.1, V_b = 23000) annotation (
        Placement(transformation(extent = {{100, -10}, {80, 10}})));
    equation
      connect(SSCIM6.y, CIM6.we) annotation (
        Line(points = {{61, -36}, {96, -36}, {96, -12}}, color = {0, 0, 127}));
      connect(bus4_mt1.p, CIM6.p) annotation (
        Line(points = {{20, 0}, {80, 0}}, color = {0, 0, 255}));
      annotation (
        preferredView = "info",
        Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-160, -100}, {160, 100}})),
        Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-160, -100}, {160, 100}})),
        experiment(StopTime = 10, __Dymola_NumberOfIntervals = 5000, __Dymola_Algorithm = "Dassl"),
        Documentation(info = "<html>
<p>The example NonMultiDomainCIM6 contains a non multi-domain three-phase CIM6 induction motor model which drives a mechanical load that is described with a mechanical torque equation described in the equation section the model. 
The three-phase induction motor type I is based on the non multi-domain model from <i>Milano, Federico. Power system modelling and scripting. Springer Science & Business Media, 2010. </i><\\p>
<p>In order to validate the non multi-domain model, this validation example reproduces Example 11.7 Motor Startup from <i>Chow, Joe H., and Juan J. Sanchez-Gasca. Power system modeling, computation, and control. John Wiley & Sons, 2020.</i> but with 
a different load torque profile because the mechanical torque equations do not match due to modeling differences. Although the mechanical torque is not the same, the system itsel if and the startup process is similar. </p>
<p>Simulate the system for 10 seconds. Variables of interest are:</p>
<ul>
<li><code>CIM6.s</code></li>
<li><code>CIM6.wr</code></li>
<li><code>CIM6.P</code></li>
<li><code>CIM6.Q</code></li>
<li><code>CIM6.Pmotor</code></li>
</ul>
</html>"));
    end NonMultiDomainCIM6;
  end NonMultiDomainThreePhaseMotorValidation;
end NonMultiDomainThreePhaseInductionMotors;
