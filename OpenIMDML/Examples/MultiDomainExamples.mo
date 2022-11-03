within OpenIMDML.Examples;
package MultiDomainExamples
  extends Modelica.Icons.ExamplesPackage;
  package ThreePhaseInductionMotors
    extends Modelica.Icons.ExamplesPackage;
    package MotorValidation
      extends Modelica.Icons.ExamplesPackage;
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
          annotation (Placement(transformation(extent={{80,40},{140,80}})));
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
        Modelica.Blocks.Sources.RealExpression SS1(y=2*Modelica.Constants.pi*(
              SysData.fn))
                      annotation (Placement(transformation(extent={{20,-46},{40,
                  -26}})));
        MultiDomainModels.Motors.ThreePhase.PSAT.MotorTypeI Motor1(V_b=23000)
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
          annotation (Line(points={{41,-36},{58,-36},{58,-12}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
                  -100},{160,100}})),
                                Diagram(coordinateSystem(preserveAspectRatio=false,
                extent={{-160,-100},{160,100}})),
          experiment(
            StopTime=10,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end MotorTypeI_Validation;

      model MotorTypeIII_Validation
        extends Modelica.Icons.Example;
        OpenIPSL.Electrical.Machines.PSSE.GENCLS inf3(
          V_b=16000,
          v_0=1.05,
          M_b=600000000,
          H=0)
          annotation (Placement(transformation(extent={{-148,-10},{-128,10}})));
        OpenIPSL.Electrical.Buses.Bus bus1_mt3(V_b=16000, v_0=1.05)
          annotation (Placement(transformation(extent={{-118,-10},{-98,10}})));
        OpenIPSL.Electrical.Buses.Bus bus2_mt3(V_b=230000)
          annotation (Placement(transformation(extent={{-78,-10},{-58,10}})));
        OpenIPSL.Electrical.Branches.PwLine line_mt3(
          R=0,
          X=0.02,
          G=0,
          B=0) annotation (Placement(transformation(extent={{-52,-14},{-24,14}})));
        OpenIPSL.Electrical.Buses.Bus bus3_mt3(V_b=230000)
          annotation (Placement(transformation(extent={{-18,-10},{2,10}})));
        OpenIPSL.Electrical.Buses.Bus bus4_mt3(V_b=23000)
          annotation (Placement(transformation(extent={{22,-10},{42,10}})));
        Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor3
          annotation (Placement(transformation(extent={{82,-10},{102,10}})));
        Modelica.Mechanics.Rotational.Sources.Torque torque3
          annotation (Placement(transformation(extent={{112,-50},{132,-30}})));
        Modelica.Mechanics.Rotational.Components.Inertia load_inertia3(J=1)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={122,0})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1_mt3(
          V_b=16000,
          Vn=16000,
          rT=0,
          xT=0.025)
                   annotation (Placement(transformation(extent={{-98,-10},{-78,10}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2_mt3(
          V_b=230000,
          Vn=230000,
          rT=0,
          xT=0.15) annotation (Placement(transformation(extent={{2,-10},{22,10}})));
        Modelica.Blocks.Sources.RealExpression Torque3(y=-(0.1*(15/100)*(Motor3.s) + 0.5
              *(15/100)*(1 - Motor3.s)^2)*Motor3.T_b)
          annotation (Placement(transformation(extent={{72,-50},{92,-30}})));
        OpenIPSL.Electrical.Loads.PSAT.PQ Load3(V_b=230000, P_0=500000000)
          annotation (Placement(transformation(extent={{-28,-40},{-8,-20}})));
        Modelica.Blocks.Sources.RealExpression SS3(y=2*Modelica.Constants.pi*(
              SysData.fn))
                      annotation (Placement(transformation(extent={{20,-46},{40,
                  -26}})));
        inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
          annotation (Placement(transformation(extent={{80,40},{140,80}})));
        MultiDomainModels.Motors.ThreePhase.PSAT.MotorTypeIII Motor3(V_b=23000)
          annotation (Placement(transformation(extent={{68,-10},{48,10}})));
      equation
        connect(inf3.p,bus1_mt3. p)
          annotation (Line(points={{-128,0},{-108,0}}, color={0,0,255}));
        connect(bus2_mt3.p,line_mt3. p)
          annotation (Line(points={{-68,0},{-50.6,0}}, color={0,0,255}));
        connect(line_mt3.n,bus3_mt3. p)
          annotation (Line(points={{-25.4,0},{-8,0}},color={0,0,255}));
        connect(torqueSensor3.flange_b,load_inertia3. flange_a)
          annotation (Line(points={{102,0},{112,0}}, color={0,0,0}));
        connect(torque3.flange,load_inertia3. flange_b) annotation (Line(points={{132,-40},
                {142,-40},{142,0},{132,0}},      color={0,0,0}));
        connect(bus1_mt3.p,tf1_mt3. p)
          annotation (Line(points={{-108,0},{-99,0}}, color={0,0,255}));
        connect(tf1_mt3.n,bus2_mt3. p)
          annotation (Line(points={{-77,0},{-68,0}}, color={0,0,255}));
        connect(bus3_mt3.p,tf2_mt3. p)
          annotation (Line(points={{-8,0},{1,0}},color={0,0,255}));
        connect(tf2_mt3.n,bus4_mt3. p)
          annotation (Line(points={{23,0},{32,0}}, color={0,0,255}));
        connect(Torque3.y,torque3. tau)
          annotation (Line(points={{93,-40},{110,-40}},  color={0,0,127}));
        connect(Load3.p,bus3_mt3. p)
          annotation (Line(points={{-18,-20},{-18,0},{-8,0}},color={0,0,255}));
        connect(torqueSensor3.tau, Motor3.mech_torque) annotation (Line(points={{84,-11},
                {84,-20},{64,-20},{64,-12}}, color={0,0,127}));
        connect(SS3.y, Motor3.we)
          annotation (Line(points={{41,-36},{58,-36},{58,-12}}, color={0,0,127}));
        connect(bus4_mt3.p, Motor3.p)
          annotation (Line(points={{32,0},{48,0}}, color={0,0,255}));
        connect(Motor3.flange, torqueSensor3.flange_a)
          annotation (Line(points={{68,0},{82,0}}, color={0,0,0}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
                  -100},{160,100}})),                                  Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-160,-100},{
                  160,100}})),
          experiment(
            StopTime=5,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end MotorTypeIII_Validation;

      model MotorTypeV_Validation
        extends Modelica.Icons.Example;
        OpenIPSL.Electrical.Machines.PSSE.GENCLS inf5(
          V_b=16000,
          v_0=1.05,
          M_b=600000000,
          H=0)
          annotation (Placement(transformation(extent={{-148,-10},{-128,10}})));
        OpenIPSL.Electrical.Buses.Bus bus1_mt5(V_b=16000, v_0=1.05)
          annotation (Placement(transformation(extent={{-118,-10},{-98,10}})));
        OpenIPSL.Electrical.Buses.Bus bus2_mt5(V_b=230000)
          annotation (Placement(transformation(extent={{-78,-10},{-58,10}})));
        OpenIPSL.Electrical.Branches.PwLine line_mt5(
          R=0,
          X=0.02,
          G=0,
          B=0) annotation (Placement(transformation(extent={{-52,-14},{-24,14}})));
        OpenIPSL.Electrical.Buses.Bus bus3_mt5(V_b=230000)
          annotation (Placement(transformation(extent={{-18,-10},{2,10}})));
        OpenIPSL.Electrical.Buses.Bus bus4_mt5(V_b=23000)
          annotation (Placement(transformation(extent={{22,-10},{42,10}})));
        OpenIPSL.Electrical.Loads.PSAT.PQ Load5(V_b=230000, P_0=500000000)
          annotation (Placement(transformation(extent={{-28,-40},{-8,-20}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1_mt5(
          V_b=16000,
          Vn=16000,
          rT=0,
          xT=0.025)
                   annotation (Placement(transformation(extent={{-98,-10},{-78,10}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2_mt5(
          V_b=230000,
          Vn=230000,
          rT=0,
          xT=0.15) annotation (Placement(transformation(extent={{2,-10},{22,10}})));
        Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor5
          annotation (Placement(transformation(extent={{82,-10},{102,10}})));
        Modelica.Mechanics.Rotational.Sources.Torque torque5
          annotation (Placement(transformation(extent={{112,-50},{132,-30}})));
        Modelica.Mechanics.Rotational.Components.Inertia load_inertia5(J=1)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={122,0})));
        Modelica.Blocks.Sources.RealExpression Torque5(y=-(0.1*(15/100)*(Motor5.s) +
              0.5*(15/100)*(1 - Motor5.s)^2)*Motor5.T_b)
          annotation (Placement(transformation(extent={{72,-50},{92,-30}})));
        Modelica.Blocks.Sources.RealExpression SS5(y=2*Modelica.Constants.pi*(
              SysData.fn))
          annotation (Placement(transformation(extent={{20,-46},{40,-26}})));
        inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
          annotation (Placement(transformation(extent={{80,40},{140,80}})));
        MultiDomainModels.Motors.ThreePhase.PSAT.MotorTypeV Motor5(V_b=23000)
          annotation (Placement(transformation(extent={{68,-10},{48,10}})));
      equation
        connect(inf5.p,bus1_mt5. p)
          annotation (Line(points={{-128,0},{-108,0}},     color={0,0,255}));
        connect(bus2_mt5.p,line_mt5. p)
          annotation (Line(points={{-68,0},{-50.6,0}},     color={0,0,255}));
        connect(line_mt5.n,bus3_mt5. p)
          annotation (Line(points={{-25.4,0},{-8,0}},    color={0,0,255}));
        connect(Load5.p,bus3_mt5. p)
          annotation (Line(points={{-18,-20},{-18,0},{-8,0}},     color={0,0,255}));
        connect(bus1_mt5.p,tf1_mt5. p)
          annotation (Line(points={{-108,0},{-99,0}},     color={0,0,255}));
        connect(tf1_mt5.n,bus2_mt5. p)
          annotation (Line(points={{-77,0},{-68,0}},     color={0,0,255}));
        connect(bus3_mt5.p,tf2_mt5. p)
          annotation (Line(points={{-8,0},{1,0}},    color={0,0,255}));
        connect(tf2_mt5.n,bus4_mt5. p)
          annotation (Line(points={{23,0},{32,0}},     color={0,0,255}));
        connect(torqueSensor5.flange_b,load_inertia5. flange_a)
          annotation (Line(points={{102,0},{112,0}}, color={0,0,0}));
        connect(torque5.flange,load_inertia5. flange_b) annotation (Line(points={{132,-40},
                {142,-40},{142,0},{132,0}},      color={0,0,0}));
        connect(Torque5.y,torque5. tau)
          annotation (Line(points={{93,-40},{110,-40}},  color={0,0,127}));
        connect(bus4_mt5.p, Motor5.p)
          annotation (Line(points={{32,0},{48,0}}, color={0,0,255}));
        connect(Motor5.flange, torqueSensor5.flange_a)
          annotation (Line(points={{68,0},{82,0}}, color={0,0,0}));
        connect(torqueSensor5.tau, Motor5.mech_torque) annotation (Line(points={{84,
                -11},{84,-20},{64,-20},{64,-12}}, color={0,0,127}));
        connect(SS5.y, Motor5.we)
          annotation (Line(points={{41,-36},{58,-36},{58,-12}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
                  -100},{160,100}})),                                  Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-160,-100},{
                  160,100}})),
          experiment(
            StopTime=10,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end MotorTypeV_Validation;

      model CIM5_and_CIM6_Validation
        extends Modelica.Icons.Example;
        OpenIPSL.Electrical.Machines.PSSE.GENCLS infCIM5(
          V_b=16000,
          v_0=1.05,
          M_b=600000000,
          H=0)
          annotation (Placement(transformation(extent={{-148,-10},{-128,10}})));
        OpenIPSL.Electrical.Buses.Bus bus1_CIM5(V_b=16000, v_0=1.05)
          annotation (Placement(transformation(extent={{-118,-10},{-98,10}})));
        OpenIPSL.Electrical.Buses.Bus bus2_CIM5(V_b=230000)
          annotation (Placement(transformation(extent={{-78,-10},{-58,10}})));
        OpenIPSL.Electrical.Branches.PwLine line_CIM5(
          R=0,
          X=0.02,
          G=0,
          B=0) annotation (Placement(transformation(extent={{-52,-14},{-24,14}})));
        OpenIPSL.Electrical.Buses.Bus bus3_CIM5(V_b=230000)
          annotation (Placement(transformation(extent={{-18,-10},{2,10}})));
        OpenIPSL.Electrical.Buses.Bus bus4_CIM5(V_b=23000)
          annotation (Placement(transformation(extent={{22,-10},{42,10}})));
        OpenIPSL.Electrical.Loads.PSAT.PQ LoadCIM5(V_b=230000, P_0=500000000)
          annotation (Placement(transformation(extent={{-28,-40},{-8,-20}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1_CIM5(
          V_b=16000,
          Vn=16000,
          rT=0,
          xT=0.025)
          annotation (Placement(transformation(extent={{-98,-10},{-78,10}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2_CIM5(
          V_b=230000,
          Vn=230000,
          rT=0,
          xT=0.15)
          annotation (Placement(transformation(extent={{2,-10},{22,10}})));
        Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensorCIM5
          annotation (Placement(transformation(extent={{82,-10},{102,10}})));
        Modelica.Mechanics.Rotational.Sources.Torque torqueCIM5
          annotation (Placement(transformation(extent={{112,-50},{132,-30}})));
        Modelica.Mechanics.Rotational.Components.Inertia load_inertiaCIM5(J=1)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={122,0})));
        Modelica.Blocks.Sources.RealExpression TorqueCIM5(y=-(0.1*(15/100)*(CIM5.s) +
              0.5*(15/100)*(1 - CIM5.s)^2)*CIM5.T_b)
          annotation (Placement(transformation(extent={{72,-50},{92,-30}})));
        Modelica.Blocks.Sources.RealExpression SSCIM(y=2*Modelica.Constants.pi*(
              SysData.fn))
          annotation (Placement(transformation(extent={{20,-46},{40,-26}})));
        inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
          annotation (Placement(transformation(extent={{80,40},{140,80}})));
        MultiDomainModels.Motors.ThreePhase.PSSE.CIM5_CIM6 CIM5(
        V_b = 23000)
          annotation (Placement(transformation(extent={{68,-10},{48,10}})));
      equation
        connect(infCIM5.p,bus1_CIM5. p)
          annotation (Line(points={{-128,0},{-108,0}},       color={0,0,255}));
        connect(bus2_CIM5.p,line_CIM5. p)
          annotation (Line(points={{-68,0},{-50.6,0}},       color={0,0,255}));
        connect(line_CIM5.n,bus3_CIM5. p)
          annotation (Line(points={{-25.4,0},{-8,0}},      color={0,0,255}));
        connect(LoadCIM5.p,bus3_CIM5. p) annotation (Line(points={{-18,-20},{-18,0},
                {-8,0}},   color={0,0,255}));
        connect(bus1_CIM5.p,tf1_CIM5. p)
          annotation (Line(points={{-108,0},{-99,0}},       color={0,0,255}));
        connect(tf1_CIM5.n,bus2_CIM5. p)
          annotation (Line(points={{-77,0},{-68,0}},       color={0,0,255}));
        connect(bus3_CIM5.p,tf2_CIM5. p)
          annotation (Line(points={{-8,0},{1,0}},      color={0,0,255}));
        connect(tf2_CIM5.n,bus4_CIM5. p)
          annotation (Line(points={{23,0},{32,0}},       color={0,0,255}));
        connect(torqueSensorCIM5.flange_b,load_inertiaCIM5. flange_a)
          annotation (Line(points={{102,0},{112,0}},       color={0,0,0}));
        connect(torqueCIM5.flange,load_inertiaCIM5. flange_b) annotation (Line(points={{132,-40},
                {142,-40},{142,0},{132,0}},                   color={0,0,0}));
        connect(TorqueCIM5.y,torqueCIM5. tau)
          annotation (Line(points={{93,-40},{110,-40}},    color={0,0,127}));
        connect(torqueSensorCIM5.tau, CIM5.mech_torque) annotation (Line(points={{84,
                -11},{84,-20},{64,-20},{64,-12}}, color={0,0,127}));
        connect(SSCIM.y, CIM5.we)
          annotation (Line(points={{41,-36},{58,-36},{58,-12}}, color={0,0,127}));
        connect(bus4_CIM5.p, CIM5.p)
          annotation (Line(points={{32,0},{48,0}}, color={0,0,255}));
        connect(CIM5.flange, torqueSensorCIM5.flange_a)
          annotation (Line(points={{68,0},{82,0}}, color={0,0,0}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
                  -100},{160,100}})),                                  Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-160,-100},{
                  160,100}})),
          experiment(
            StopTime=10,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end CIM5_and_CIM6_Validation;

      model MotorTypeI_Validation_test
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
          annotation (Placement(transformation(extent={{90,48},{150,88}})));
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
        Modelica.Blocks.Sources.RealExpression Torque1(y=-(0.1*(15/100)*(motor.motor.s)
               + 0.5*(15/100)*(1 - motor.motor.s)^2)*motor.motor.T_b)
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
        Modelica.Blocks.Sources.RealExpression SS1(y=2*Modelica.Constants.pi*(
              SysData.fn))
                      annotation (Placement(transformation(extent={{20,-46},{40,
                  -26}})));
        MultiDomainModels.Motors.ThreePhase.MultiDomainMotor motor(
          M_b=15000000,
          N=1,
          H=0.4,
          redeclare
            OpenIMDML.MultiDomainModels.Motors.ThreePhase.PSSE.CIM5_CIM6 motor)
          annotation (Placement(transformation(extent={{50,-10},{70,10}})));
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
        connect(bus4_mt1.p, motor.pwpin)
          annotation (Line(points={{32,0},{50,0}}, color={0,0,255}));
        connect(motor.flange1, torqueSensor1.flange_a)
          annotation (Line(points={{70,0},{82,0}}, color={0,0,0}));
        connect(torqueSensor1.tau, motor.mech_torque) annotation (Line(points={
                {84,-11},{84,-20},{66,-20},{66,-12}}, color={0,0,127}));
        connect(SS1.y, motor.we) annotation (Line(points={{41,-36},{60,-36},{60,
                -12}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
                  -100},{160,100}})),
                                Diagram(coordinateSystem(preserveAspectRatio=false,
                extent={{-160,-100},{160,100}})),
          experiment(
            StopTime=10,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end MotorTypeI_Validation_test;

      model MultiDomainMotorValidation
        extends Modelica.Icons.Example;

        OpenIPSL.Types.PerUnit Torq;
        Modelica.Units.SI.AngularVelocity sync_speed;

        OpenIPSL.Electrical.Machines.PSSE.GENCLS inf(
          V_b=16000,
          v_0=1.05,
          M_b=600000000,
          H=0,
          X_d=0)
               annotation (Placement(transformation(extent={{-116,-10},{-96,10}})));
        OpenIPSL.Electrical.Buses.Bus bus1(V_b=16000, v_0=1.05)
          annotation (Placement(transformation(extent={{-94,-10},{-74,10}})));
        OpenIPSL.Electrical.Buses.Bus bus2(V_b=230000, v_0=1.03)
          annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
        OpenIPSL.Electrical.Branches.PwLine line_mt1(
          R=0,
          X=0.02,
          G=0,
          B=0) annotation (Placement(transformation(extent={{-42,-14},{-14,14}})));
        OpenIPSL.Electrical.Buses.Bus bus3(V_b=230000, v_0=1.023)
          annotation (Placement(transformation(extent={{-14,-10},{6,10}})));
        inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
          annotation (Placement(transformation(extent={{52,32},{100,60}})));
        OpenIPSL.Electrical.Buses.Bus bus4(V_b=23000, angle_0=0.017715091907742)
          annotation (Placement(transformation(extent={{22,-10},{42,10}})));
        Modelica.Mechanics.Rotational.Sensors.TorqueSensor Torque_Sensor
          annotation (Placement(transformation(extent={{66,-10},{86,10}})));
        Modelica.Mechanics.Rotational.Components.Inertia Load_Inertia(J=1)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={102,0})));
        Modelica.Mechanics.Rotational.Sources.Torque Torque
          annotation (Placement(transformation(extent={{94,-46},{114,-26}})));
        Modelica.Blocks.Sources.RealExpression Torque_Equation(y=Torq)
          annotation (Placement(transformation(extent={{62,-46},{82,-26}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1(
          V_b=16000,
          Vn=16000,
          rT=0,
          xT=0.025)
          annotation (Placement(transformation(extent={{-76,-10},{-56,10}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2(
          V_b=230000,
          Vn=230000,
          rT=0,
          xT=0.15) annotation (Placement(transformation(extent={{4,-10},{24,10}})));
        OpenIPSL.Electrical.Loads.PSAT.PQ Load(V_b=230000, P_0=500000000)
          annotation (Placement(transformation(extent={{-20,-40},{0,-20}})));
        Modelica.Blocks.Sources.RealExpression Synchronous_Speed(y=sync_speed)
          annotation (Placement(transformation(extent={{12,-46},{32,-26}})));
        MultiDomainModels.Motors.ThreePhase.MultiDomainMotor motor(M_b=15000000,
            redeclare
            OpenIMDML.MultiDomainModels.Motors.ThreePhase.PSAT.MotorTypeIII
            motor)                                                               annotation (Placement(transformation(extent={{40,-10},{60,10}})));
      equation

        Torq = -(0.1*(15/100)*(motor.motor.s) + 0.5*(15/100)*(1 - motor.motor.s)^2)*motor.motor.T_b;
        //Torq = -(0.1*(motor.motor.s) + 0.5*(1 - motor.motor.s)^2);

        sync_speed = 2*Modelica.Constants.pi*SysData.fn;

        connect(inf.p, bus1.p)
          annotation (Line(points={{-96,0},{-84,0}},   color={0,0,255}));
        connect(bus2.p, line_mt1.p)
          annotation (Line(points={{-48,0},{-40.6,0}}, color={0,0,255}));
        connect(line_mt1.n, bus3.p)
          annotation (Line(points={{-15.4,0},{-4,0}}, color={0,0,255}));
        connect(Torque_Sensor.flange_b, Load_Inertia.flange_a)
          annotation (Line(points={{86,0},{92,0}}, color={0,0,0}));
        connect(Torque.flange,Load_Inertia. flange_b) annotation (Line(points={{114,-36},
                {118,-36},{118,0},{112,0}}, color={0,0,0}));
        connect(bus1.p, tf1.p)
          annotation (Line(points={{-84,0},{-77,0}},  color={0,0,255}));
        connect(tf1.n, bus2.p)
          annotation (Line(points={{-55,0},{-48,0}}, color={0,0,255}));
        connect(bus3.p, tf2.p)
          annotation (Line(points={{-4,0},{3,0}}, color={0,0,255}));
        connect(tf2.n, bus4.p)
          annotation (Line(points={{25,0},{32,0}}, color={0,0,255}));
        connect(Torque_Equation.y, Torque.tau)
          annotation (Line(points={{83,-36},{92,-36}}, color={0,0,127}));
        connect(Load.p, bus3.p)
          annotation (Line(points={{-10,-20},{-10,0},{-4,0}}, color={0,0,255}));
        connect(bus4.p, motor.pwpin)
          annotation (Line(points={{32,0},{40,0}}, color={0,0,255}));
        connect(motor.flange1, Torque_Sensor.flange_a)
          annotation (Line(points={{60,0},{66,0}}, color={0,0,0}));
        connect(Torque_Sensor.tau, motor.mech_torque) annotation (Line(points={
                {68,-11},{68,-20},{56,-20},{56,-12}}, color={0,0,127}));
        connect(Synchronous_Speed.y, motor.we) annotation (Line(points={{33,-36},
                {50,-36},{50,-12}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},
                  {120,100}})), Diagram(coordinateSystem(preserveAspectRatio=false,
                extent={{-120,-100},{120,100}})),
          experiment(
            StopTime=5,
            __Dymola_NumberOfIntervals=10000,
            Tolerance=1e-05,
            __Dymola_Algorithm="Dassl"));
      end MultiDomainMotorValidation;
    end MotorValidation;

    package VSDMotorStartUp
      extends Modelica.Icons.ExamplesPackage;
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

      model MotorTypeIII_VSD_StartUp
        extends Modelica.Icons.Example;
        OpenIPSL.Electrical.Machines.PSSE.GENCLS inf1(
          V_b=16000,
          v_0=1.05,
          M_b=600000000,
          H=0)
          annotation (Placement(transformation(extent={{-146,40},{-126,60}})));
        OpenIPSL.Electrical.Buses.Bus bus1_mt1(V_b=16000, v_0=1.05)
          annotation (Placement(transformation(extent={{-116,40},{-96,60}})));
        OpenIPSL.Electrical.Buses.Bus bus2_mt1(V_b=230000)
          annotation (Placement(transformation(extent={{-76,40},{-56,60}})));
        OpenIPSL.Electrical.Branches.PwLine line_mt1(
          R=0,
          X=0.02,
          G=0,
          B=0) annotation (Placement(transformation(extent={{-50,36},{-22,64}})));
        OpenIPSL.Electrical.Buses.Bus bus3_mt1(V_b=230000)
          annotation (Placement(transformation(extent={{-16,40},{4,60}})));
        inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
          annotation (Placement(transformation(extent={{80,80},{140,120}})));
        OpenIPSL.Electrical.Buses.Bus bus4_mt1(V_b=23000)
          annotation (Placement(transformation(extent={{24,40},{44,60}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1_mt1(
          V_b=16000,
          Vn=16000,
          rT=0,
          xT=0.025)
                   annotation (Placement(transformation(extent={{-96,40},{-76,60}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2_mt1(
          V_b=230000,
          Vn=230000,
          rT=0,
          xT=0.15) annotation (Placement(transformation(extent={{4,40},{24,60}})));
        Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC
          aC2DC_and_DC2AC_uninitialized(V_b=23000, v_0=1)
          annotation (Placement(transformation(extent={{-60,-50},{-40,-30}})));
        Controls.VariableSpeedDrive.Controls.VoltsHertz_Controller
          voltsHertz_Controller(
          V_b=23000,
          f_max=60,
          f_min=0)
          annotation (Placement(transformation(extent={{-60,-100},{-42,-80}})));
        OpenIPSL.Electrical.Loads.PSAT.PQ Load1(V_b=230000, P_0=500000000)
          annotation (Placement(transformation(extent={{-26,12},{-6,32}})));
        Modelica.Blocks.Sources.RealExpression FREQUENCY(y=2*Modelica.Constants.pi
              *SysData.fn) annotation (Placement(transformation(extent={{-132,-100},
                  {-112,-80}})));
        MultiDomainModels.Motors.ThreePhase.PSAT.MotorTypeIII Motor3(V_b=23000)
          annotation (Placement(transformation(extent={{20,-50},{0,-30}})));
        Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor3
          annotation (Placement(transformation(extent={{34,-50},{54,-30}})));
        Modelica.Mechanics.Rotational.Sources.Torque torque3
          annotation (Placement(transformation(extent={{64,-90},{84,-70}})));
        Modelica.Mechanics.Rotational.Components.Inertia load_inertia3(J=1)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={74,-40})));
        Modelica.Blocks.Sources.RealExpression Torque3(y=-(0.1*(15/100)*(Motor3.s)
               + 0.5*(15/100)*(1 - Motor3.s)^2)*Motor3.T_b)
          annotation (Placement(transformation(extent={{28,-90},{48,-70}})));
      equation
        connect(inf1.p,bus1_mt1. p)
          annotation (Line(points={{-126,50},{-106,50}}, color={0,0,255}));
        connect(bus2_mt1.p,line_mt1. p)
          annotation (Line(points={{-66,50},{-48.6,50}}, color={0,0,255}));
        connect(line_mt1.n,bus3_mt1. p)
          annotation (Line(points={{-23.4,50},{-6,50}},color={0,0,255}));
        connect(bus1_mt1.p,tf1_mt1. p)
          annotation (Line(points={{-106,50},{-97,50}}, color={0,0,255}));
        connect(tf1_mt1.n,bus2_mt1. p)
          annotation (Line(points={{-75,50},{-66,50}}, color={0,0,255}));
        connect(bus3_mt1.p,tf2_mt1. p)
          annotation (Line(points={{-6,50},{3,50}},color={0,0,255}));
        connect(tf2_mt1.n,bus4_mt1. p)
          annotation (Line(points={{25,50},{34,50}}, color={0,0,255}));
        connect(bus4_mt1.p, aC2DC_and_DC2AC_uninitialized.p) annotation (Line(points={
                {34,50},{80,50},{80,0},{-100,0},{-100,-40},{-60,-40}}, color={0,0,255}));
        connect(aC2DC_and_DC2AC_uninitialized.Vc, voltsHertz_Controller.Vc)
          annotation (Line(points={{-54.5455,-52},{-54.6,-78}}, color={0,0,127}));
        connect(voltsHertz_Controller.m, aC2DC_and_DC2AC_uninitialized.m_input)
          annotation (Line(points={{-45.4,-78},{-45.4545,-78},{-45.4545,-52}}, color={
                0,0,127}));
        connect(Load1.p, bus3_mt1.p)
          annotation (Line(points={{-16,32},{-16,50},{-6,50}}, color={0,0,255}));
        connect(aC2DC_and_DC2AC_uninitialized.n, Motor3.p)
          annotation (Line(points={{-40,-40},{0,-40}}, color={0,0,255}));
        connect(voltsHertz_Controller.we, Motor3.we) annotation (Line(points={{
                -38,-94},{10,-94},{10,-52}}, color={0,0,127}));
        connect(Motor3.wr, voltsHertz_Controller.motor_speed) annotation (Line(
              points={{4,-52},{4,-86},{-38,-86}}, color={0,0,127}));
        connect(torqueSensor3.flange_b,load_inertia3. flange_a)
          annotation (Line(points={{54,-40},{64,-40}},
                                                     color={0,0,0}));
        connect(torque3.flange,load_inertia3. flange_b) annotation (Line(points={{84,-80},
                {94,-80},{94,-40},{84,-40}},     color={0,0,0}));
        connect(Torque3.y,torque3. tau)
          annotation (Line(points={{49,-80},{62,-80}},   color={0,0,127}));
        connect(Motor3.flange, torqueSensor3.flange_a)
          annotation (Line(points={{20,-40},{34,-40}}, color={0,0,0}));
        connect(torqueSensor3.tau, Motor3.mech_torque) annotation (Line(points=
                {{36,-51},{36,-60},{16,-60},{16,-52}}, color={0,0,127}));
        connect(FREQUENCY.y, voltsHertz_Controller.W_ref)
          annotation (Line(points={{-111,-90},{-62,-90}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-160},
                  {160,160}})), Diagram(coordinateSystem(preserveAspectRatio=false,
                extent={{-160,-160},{160,160}})),
          experiment(
            StopTime=100,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end MotorTypeIII_VSD_StartUp;

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
          annotation (Placement(transformation(extent={{100,40},{120,60}})));
        Modelica.Mechanics.Rotational.Components.Inertia load_inertia1(J=1)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={138,50})));
        Modelica.Mechanics.Rotational.Sources.Torque torque1
          annotation (Placement(transformation(extent={{130,0},{150,20}})));
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
        MultiDomainModels.Motors.ThreePhase.PSAT.MotorTypeI Motor1(V_b=23000)
          annotation (Placement(transformation(extent={{92,40},{72,60}})));
        Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC aC2DC_and_DC2AC(V_b=23000,
            v_0=0.8)
          annotation (Placement(transformation(extent={{46,40},{66,60}})));
        OpenIPSL.Electrical.Loads.PSAT.PQ Load1(V_b=230000, P_0=500000000)
          annotation (Placement(transformation(extent={{-20,12},{0,32}})));
        Modelica.Blocks.Sources.Ramp     m_const(
          height=0.9,
          duration=50,
          offset=0.1)
          annotation (Placement(transformation(extent={{-82,-70},{-62,-50}})));
        Modelica.Blocks.Math.Gain gain(k=1)
          annotation (Placement(transformation(extent={{-20,-70},{0,-50}})));
        Modelica.Blocks.Math.Gain gain1(k=2*3.1415*SysData.fn)
          annotation (Placement(transformation(extent={{-20,-100},{0,-80}})));
        Modelica.Blocks.Sources.RealExpression Torque1(y=-(0.1*(15/100)*(Motor1.s)
               + 0.5*(15/100)*(1 - Motor1.s)^2)*Motor1.T_b)
          annotation (Placement(transformation(extent={{98,0},{118,20}})));
      equation
        connect(inf1.p,bus1_mt1. p)
          annotation (Line(points={{-120,50},{-100,50}}, color={0,0,255}));
        connect(bus2_mt1.p,line_mt1. p)
          annotation (Line(points={{-60,50},{-42.6,50}}, color={0,0,255}));
        connect(line_mt1.n,bus3_mt1. p)
          annotation (Line(points={{-17.4,50},{0,50}}, color={0,0,255}));
        connect(torqueSensor1.flange_b,load_inertia1. flange_a)
          annotation (Line(points={{120,50},{128,50}}, color={0,0,0}));
        connect(torque1.flange,load_inertia1. flange_b) annotation (Line(points={{150,10},
                {156,10},{156,50},{148,50}},     color={0,0,0}));
        connect(bus1_mt1.p,tf1_mt1. p)
          annotation (Line(points={{-100,50},{-91,50}}, color={0,0,255}));
        connect(tf1_mt1.n,bus2_mt1. p)
          annotation (Line(points={{-69,50},{-60,50}}, color={0,0,255}));
        connect(bus3_mt1.p,tf2_mt1. p)
          annotation (Line(points={{0,50},{9,50}}, color={0,0,255}));
        connect(tf2_mt1.n,bus4_mt1. p)
          annotation (Line(points={{31,50},{40,50}}, color={0,0,255}));
        connect(Motor1.flange,torqueSensor1. flange_a)
          annotation (Line(points={{92,50},{100,50}},
                                                   color={0,0,0}));
        connect(torqueSensor1.tau,Motor1. mech_torque) annotation (Line(points={{102,39},
                {102,38},{88,38}},           color={0,0,127}));
        connect(aC2DC_and_DC2AC.n, Motor1.p)
          annotation (Line(points={{66,50},{72,50}}, color={0,0,255}));
        connect(Load1.p, bus3_mt1.p)
          annotation (Line(points={{-10,32},{-10,50},{0,50}},  color={0,0,255}));
        connect(bus4_mt1.p, aC2DC_and_DC2AC.p)
          annotation (Line(points={{40,50},{46,50}}, color={0,0,255}));
        connect(m_const.y, gain.u)
          annotation (Line(points={{-61,-60},{-22,-60}}, color={0,0,127}));
        connect(gain1.u, gain.u) annotation (Line(points={{-22,-90},{-40,-90},{
                -40,-60},{-22,-60}}, color={0,0,127}));
        connect(gain.y, aC2DC_and_DC2AC.m_input) annotation (Line(points={{1,-60},
                {60.5455,-60},{60.5455,38}},      color={0,0,127}));
        connect(gain1.y, Motor1.we) annotation (Line(points={{1,-90},{82,-90},{
                82,38}}, color={0,0,127}));
        connect(Torque1.y, torque1.tau)
          annotation (Line(points={{119,10},{128,10}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-160},
                  {160,160}})), Diagram(coordinateSystem(preserveAspectRatio=false,
                extent={{-160,-160},{160,160}})),
          experiment(
            StopTime=100,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end MotorTypeI_VSD_Ramp_StartUp;

      model MotorTypeIII_VSD_Ramp_StartUp
        extends Modelica.Icons.Example;
        OpenIPSL.Electrical.Machines.PSSE.GENCLS inf3(
          V_b=16000,
          v_0=1.05,
          M_b=600000000,
          H=0)
          annotation (Placement(transformation(extent={{-140,40},{-120,60}})));
        OpenIPSL.Electrical.Buses.Bus bus1_mt3(V_b=16000, v_0=1.05)
          annotation (Placement(transformation(extent={{-110,40},{-90,60}})));
        OpenIPSL.Electrical.Buses.Bus bus2_mt3(V_b=230000)
          annotation (Placement(transformation(extent={{-70,40},{-50,60}})));
        OpenIPSL.Electrical.Branches.PwLine line_mt3(
          R=0,
          X=0.02,
          G=0,
          B=0) annotation (Placement(transformation(extent={{-44,36},{-16,64}})));
        OpenIPSL.Electrical.Buses.Bus bus3_mt3(V_b=230000)
          annotation (Placement(transformation(extent={{-10,40},{10,60}})));
        OpenIPSL.Electrical.Buses.Bus bus4_mt3(V_b=23000)
          annotation (Placement(transformation(extent={{30,40},{50,60}})));
        Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor3
          annotation (Placement(transformation(extent={{34,-50},{54,-30}})));
        Modelica.Mechanics.Rotational.Sources.Torque torque3
          annotation (Placement(transformation(extent={{64,-90},{84,-70}})));
        Modelica.Mechanics.Rotational.Components.Inertia load_inertia3(J=1)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={74,-40})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1_mt3(
          V_b=16000,
          Vn=16000,
          rT=0,
          xT=0.025)
                   annotation (Placement(transformation(extent={{-90,40},{-70,60}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2_mt3(
          V_b=230000,
          Vn=230000,
          rT=0,
          xT=0.15) annotation (Placement(transformation(extent={{10,40},{30,60}})));
        OpenIPSL.Electrical.Loads.PSAT.PQ Load3(V_b=230000, P_0=500000000)
          annotation (Placement(transformation(extent={{-20,12},{0,32}})));
        inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
          annotation (Placement(transformation(extent={{80,80},{140,120}})));
        MultiDomainModels.Motors.ThreePhase.PSAT.MotorTypeIII_different Motor3(V_b=23000,
            H=0.4)
          annotation (Placement(transformation(extent={{20,-50},{0,-30}})));
        Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC aC2DC_and_DC2AC(V_b=23000,
            v_0=3*sqrt(6)/(2*Modelica.Constants.pi*sqrt(2)))
          annotation (Placement(transformation(extent={{-58,-50},{-38,-30}})));
        Modelica.Blocks.Sources.Ramp change(
          height=0,
          duration=60,
          offset=0.52) annotation (Placement(transformation(extent={{-144,-90},
                  {-124,-70}})));
        Modelica.Blocks.Sources.RealExpression Torque3(y=-(0.05)*Motor3.T_b)
          annotation (Placement(transformation(extent={{26,-90},{46,-70}})));
        Modelica.Blocks.Math.Gain gain(k=1) annotation (Placement(
              transformation(extent={{-80,-90},{-60,-70}})));
        Modelica.Blocks.Math.Gain gain1(k=2*Modelica.Constants.pi*SysData.fn)
                                                               annotation (
            Placement(transformation(extent={{-80,-152},{-60,-132}})));
        Modelica.Blocks.Sources.Ramp change1(
          height=0,
          duration=60,
          offset=0.41) annotation (Placement(transformation(extent={{-140,-152},
                  {-120,-132}})));
      equation
        connect(inf3.p,bus1_mt3. p)
          annotation (Line(points={{-120,50},{-100,50}},
                                                       color={0,0,255}));
        connect(bus2_mt3.p,line_mt3. p)
          annotation (Line(points={{-60,50},{-42.6,50}},
                                                       color={0,0,255}));
        connect(line_mt3.n,bus3_mt3. p)
          annotation (Line(points={{-17.4,50},{0,50}},
                                                     color={0,0,255}));
        connect(torqueSensor3.flange_b,load_inertia3. flange_a)
          annotation (Line(points={{54,-40},{64,-40}},
                                                     color={0,0,0}));
        connect(torque3.flange,load_inertia3. flange_b) annotation (Line(points={{84,-80},
                {94,-80},{94,-40},{84,-40}},     color={0,0,0}));
        connect(bus1_mt3.p,tf1_mt3. p)
          annotation (Line(points={{-100,50},{-91,50}},
                                                      color={0,0,255}));
        connect(tf1_mt3.n,bus2_mt3. p)
          annotation (Line(points={{-69,50},{-60,50}},
                                                     color={0,0,255}));
        connect(bus3_mt3.p,tf2_mt3. p)
          annotation (Line(points={{0,50},{9,50}},
                                                 color={0,0,255}));
        connect(tf2_mt3.n,bus4_mt3. p)
          annotation (Line(points={{31,50},{40,50}},
                                                   color={0,0,255}));
        connect(Load3.p,bus3_mt3. p)
          annotation (Line(points={{-10,32},{-10,50},{0,50}},color={0,0,255}));
        connect(torqueSensor3.tau,Motor3. mech_torque) annotation (Line(points={{36,-51},
                {36,-60},{16,-60},{16,-52}}, color={0,0,127}));
        connect(Motor3.flange,torqueSensor3. flange_a)
          annotation (Line(points={{20,-40},{34,-40}},
                                                   color={0,0,0}));
        connect(aC2DC_and_DC2AC.n, Motor3.p)
          annotation (Line(points={{-38,-40},{0,-40}}, color={0,0,255}));
        connect(aC2DC_and_DC2AC.p, bus4_mt3.p) annotation (Line(points={{-58,
                -40},{-100,-40},{-100,0},{80,0},{80,50},{40,50}}, color={0,0,
                255}));
        connect(Torque3.y, torque3.tau)
          annotation (Line(points={{47,-80},{62,-80}}, color={0,0,127}));
        connect(change.y, gain.u)
          annotation (Line(points={{-123,-80},{-82,-80}}, color={0,0,127}));
        connect(gain.y, aC2DC_and_DC2AC.m_input) annotation (Line(points={{-59,-80},
                {-44,-80},{-44,-56},{-43.4545,-56},{-43.4545,-52}},      color=
                {0,0,127}));
        connect(gain1.y, Motor3.we) annotation (Line(points={{-59,-142},{10,
                -142},{10,-52}}, color={0,0,127}));
        connect(gain1.u, change1.y)
          annotation (Line(points={{-82,-142},{-119,-142}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                  -160,-160},{160,160}})),                             Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-160,-160},{
                  160,160}})),
          experiment(
            StopTime=10,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end MotorTypeIII_VSD_Ramp_StartUp;

      model MotorTypeV_VSD_Ramp_StartUp
        extends Modelica.Icons.Example;
        OpenIPSL.Electrical.Machines.PSSE.GENCLS inf5(
          V_b=16000,
          v_0=1.05,
          M_b=600000000,
          H=0)
          annotation (Placement(transformation(extent={{-140,40},{-120,60}})));
        OpenIPSL.Electrical.Buses.Bus bus1_mt5(V_b=16000, v_0=1.05)
          annotation (Placement(transformation(extent={{-110,40},{-90,60}})));
        OpenIPSL.Electrical.Buses.Bus bus2_mt5(V_b=230000)
          annotation (Placement(transformation(extent={{-70,40},{-50,60}})));
        OpenIPSL.Electrical.Branches.PwLine line_mt5(
          R=0,
          X=0.02,
          G=0,
          B=0) annotation (Placement(transformation(extent={{-44,36},{-16,64}})));
        OpenIPSL.Electrical.Buses.Bus bus3_mt5(V_b=230000)
          annotation (Placement(transformation(extent={{-10,40},{10,60}})));
        OpenIPSL.Electrical.Buses.Bus bus4_mt5(V_b=23000)
          annotation (Placement(transformation(extent={{30,40},{50,60}})));
        OpenIPSL.Electrical.Loads.PSAT.PQ Load5(V_b=230000, P_0=500000000)
          annotation (Placement(transformation(extent={{-20,12},{0,32}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1_mt5(
          V_b=16000,
          Vn=16000,
          rT=0,
          xT=0.025)
                   annotation (Placement(transformation(extent={{-90,40},{-70,60}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2_mt5(
          V_b=230000,
          Vn=230000,
          rT=0,
          xT=0.15) annotation (Placement(transformation(extent={{10,40},{30,60}})));
        Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor5
          annotation (Placement(transformation(extent={{34,-50},{54,-30}})));
        Modelica.Mechanics.Rotational.Sources.Torque torque5
          annotation (Placement(transformation(extent={{64,-90},{84,-70}})));
        Modelica.Mechanics.Rotational.Components.Inertia load_inertia5(J=1)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={74,-40})));
        inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
          annotation (Placement(transformation(extent={{80,80},{140,120}})));
        MultiDomainModels.Motors.ThreePhase.PSAT.MotorTypeV Motor5
          annotation (Placement(transformation(extent={{20,-50},{0,-30}})));
        Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC
          aC2DC_and_DC2AC_uninitialized(V_b=23000, v_0=1)
          annotation (Placement(transformation(extent={{-60,-50},{-40,-30}})));
        Modelica.Blocks.Sources.Ramp     m_const(
          height=0,
          duration=50,
          offset=1)
          annotation (Placement(transformation(extent={{-140,-108},{-120,-88}})));
        Modelica.Blocks.Math.Gain gain(k=1) annotation (Placement(
              transformation(extent={{-78,-108},{-58,-88}})));
        Modelica.Blocks.Math.Gain gain1(k=2*3.1415*SysData.fn) annotation (
            Placement(transformation(extent={{-78,-150},{-58,-130}})));
        Modelica.Blocks.Sources.RealExpression Torque5(y=-0.01*Motor5.T_b)
          annotation (Placement(transformation(extent={{28,-90},{48,-70}})));
        Modelica.Blocks.Sources.Ramp     m_const1(
          height=0,
          duration=50,
          offset=1.1)
          annotation (Placement(transformation(extent={{-140,-150},{-120,-130}})));
      equation
        connect(inf5.p,bus1_mt5. p)
          annotation (Line(points={{-120,50},{-100,50}},   color={0,0,255}));
        connect(bus2_mt5.p,line_mt5. p)
          annotation (Line(points={{-60,50},{-42.6,50}},   color={0,0,255}));
        connect(line_mt5.n,bus3_mt5. p)
          annotation (Line(points={{-17.4,50},{0,50}},   color={0,0,255}));
        connect(Load5.p,bus3_mt5. p)
          annotation (Line(points={{-10,32},{-10,50},{0,50}},     color={0,0,255}));
        connect(bus1_mt5.p,tf1_mt5. p)
          annotation (Line(points={{-100,50},{-91,50}},   color={0,0,255}));
        connect(tf1_mt5.n,bus2_mt5. p)
          annotation (Line(points={{-69,50},{-60,50}},   color={0,0,255}));
        connect(bus3_mt5.p,tf2_mt5. p)
          annotation (Line(points={{0,50},{9,50}},   color={0,0,255}));
        connect(tf2_mt5.n,bus4_mt5. p)
          annotation (Line(points={{31,50},{40,50}},   color={0,0,255}));
        connect(torqueSensor5.flange_b,load_inertia5. flange_a)
          annotation (Line(points={{54,-40},{64,-40}},
                                                     color={0,0,0}));
        connect(torque5.flange,load_inertia5. flange_b) annotation (Line(points={{84,-80},
                {94,-80},{94,-40},{84,-40}},     color={0,0,0}));
        connect(Motor5.flange,torqueSensor5. flange_a)
          annotation (Line(points={{20,-40},{34,-40}},
                                                   color={0,0,0}));
        connect(torqueSensor5.tau,Motor5. mech_torque) annotation (Line(points={{36,-51},
                {36,-60},{16,-60},{16,-52}},      color={0,0,127}));
        connect(aC2DC_and_DC2AC_uninitialized.n, Motor5.p)
          annotation (Line(points={{-40,-40},{0,-40}}, color={0,0,255}));
        connect(aC2DC_and_DC2AC_uninitialized.p, bus4_mt5.p) annotation (Line(
              points={{-60,-40},{-100,-40},{-100,0},{80,0},{80,50},{40,50}},
              color={0,0,255}));
        connect(m_const.y, gain.u)
          annotation (Line(points={{-119,-98},{-80,-98}},   color={0,0,127}));
        connect(gain.y, aC2DC_and_DC2AC_uninitialized.m_input) annotation (Line(
              points={{-57,-98},{-45.4545,-98},{-45.4545,-52}},   color={0,0,
                127}));
        connect(gain1.y, Motor5.we) annotation (Line(points={{-57,-140},{10,
                -140},{10,-52}}, color={0,0,127}));
        connect(Torque5.y, torque5.tau)
          annotation (Line(points={{49,-80},{62,-80}}, color={0,0,127}));
        connect(m_const1.y, gain1.u)
          annotation (Line(points={{-119,-140},{-80,-140}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                  -160,-160},{160,160}})),                             Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-160,-160},{
                  160,160}})),
          experiment(
            StopTime=100,
            __Dymola_NumberOfIntervals=10000,
            __Dymola_Algorithm="Dassl"));
      end MotorTypeV_VSD_Ramp_StartUp;

      model CIM5_and_CIM6_VSD_Ramp_StartUp
        extends Modelica.Icons.Example;
        OpenIPSL.Electrical.Machines.PSSE.GENCLS infCIM5(
          V_b=16000,
          v_0=1.05,
          M_b=600000000,
          H=0)
          annotation (Placement(transformation(extent={{-140,40},{-120,60}})));
        OpenIPSL.Electrical.Buses.Bus bus1_CIM5(V_b=16000, v_0=1.05)
          annotation (Placement(transformation(extent={{-110,40},{-90,60}})));
        OpenIPSL.Electrical.Buses.Bus bus2_CIM5(V_b=230000)
          annotation (Placement(transformation(extent={{-70,40},{-50,60}})));
        OpenIPSL.Electrical.Branches.PwLine line_CIM5(
          R=0,
          X=0.02,
          G=0,
          B=0) annotation (Placement(transformation(extent={{-44,36},{-16,64}})));
        OpenIPSL.Electrical.Buses.Bus bus3_CIM5(V_b=230000)
          annotation (Placement(transformation(extent={{-10,40},{10,60}})));
        OpenIPSL.Electrical.Buses.Bus bus4_CIM5(V_b=23000)
          annotation (Placement(transformation(extent={{30,40},{50,60}})));
        OpenIPSL.Electrical.Loads.PSAT.PQ LoadCIM5(V_b=230000, P_0=500000000)
          annotation (Placement(transformation(extent={{-20,12},{0,32}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1_CIM5(
          V_b=16000,
          Vn=16000,
          rT=0,
          xT=0.025)
          annotation (Placement(transformation(extent={{-90,40},{-70,60}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2_CIM5(
          V_b=230000,
          Vn=230000,
          rT=0,
          xT=0.15)
          annotation (Placement(transformation(extent={{10,40},{30,60}})));
        Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensorCIM5
          annotation (Placement(transformation(extent={{34,-50},{54,-30}})));
        Modelica.Mechanics.Rotational.Sources.Torque torqueCIM5
          annotation (Placement(transformation(extent={{64,-90},{84,-70}})));
        Modelica.Mechanics.Rotational.Components.Inertia load_inertiaCIM5(J=1)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={74,-40})));
        Modelica.Blocks.Sources.RealExpression TorqueCIM5(y=-(0.5*(15/100)*(1 -
              CIM5.s)^2)*CIM5.T_b)
          annotation (Placement(transformation(extent={{28,-90},{48,-70}})));
        inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
          annotation (Placement(transformation(extent={{80,80},{140,120}})));
        MultiDomainModels.Motors.ThreePhase.PSSE.CIM5_CIM6 CIM5
          annotation (Placement(transformation(extent={{20,-50},{0,-30}})));
        Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC
          aC2DC_and_DC2AC_uninitialized(V_b=23000)
          annotation (Placement(transformation(extent={{-60,-50},{-40,-30}})));
        Modelica.Blocks.Sources.Ramp m_ramp(
          height=0.99,
          duration=100,
          offset=0.01)
          annotation (Placement(transformation(extent={{-76,-90},{-56,-70}})));
        Modelica.Blocks.Sources.Ramp f_ramp(
          height=0.9*2*Modelica.Constants.pi*SysData.fn,
          duration=100,
          offset=0.1*2*Modelica.Constants.pi*SysData.fn)
          annotation (Placement(transformation(extent={{-20,-90},{0,-70}})));
      equation
        connect(infCIM5.p,bus1_CIM5. p)
          annotation (Line(points={{-120,50},{-100,50}},     color={0,0,255}));
        connect(bus2_CIM5.p,line_CIM5. p)
          annotation (Line(points={{-60,50},{-42.6,50}},     color={0,0,255}));
        connect(line_CIM5.n,bus3_CIM5. p)
          annotation (Line(points={{-17.4,50},{0,50}},     color={0,0,255}));
        connect(LoadCIM5.p,bus3_CIM5. p) annotation (Line(points={{-10,32},{-10,
                50},{0,50}},
                           color={0,0,255}));
        connect(bus1_CIM5.p,tf1_CIM5. p)
          annotation (Line(points={{-100,50},{-91,50}},     color={0,0,255}));
        connect(tf1_CIM5.n,bus2_CIM5. p)
          annotation (Line(points={{-69,50},{-60,50}},     color={0,0,255}));
        connect(bus3_CIM5.p,tf2_CIM5. p)
          annotation (Line(points={{0,50},{9,50}},     color={0,0,255}));
        connect(tf2_CIM5.n,bus4_CIM5. p)
          annotation (Line(points={{31,50},{40,50}},     color={0,0,255}));
        connect(torqueSensorCIM5.flange_b,load_inertiaCIM5. flange_a)
          annotation (Line(points={{54,-40},{64,-40}},     color={0,0,0}));
        connect(torqueCIM5.flange,load_inertiaCIM5. flange_b) annotation (Line(points={{84,-80},
                {94,-80},{94,-40},{84,-40}},                  color={0,0,0}));
        connect(TorqueCIM5.y,torqueCIM5. tau)
          annotation (Line(points={{49,-80},{62,-80}},     color={0,0,127}));
        connect(torqueSensorCIM5.tau,CIM5. mech_torque) annotation (Line(points={{36,-51},
                {36,-60},{16,-60},{16,-52}},      color={0,0,127}));
        connect(CIM5.flange,torqueSensorCIM5. flange_a)
          annotation (Line(points={{20,-40},{34,-40}},
                                                   color={0,0,0}));
        connect(aC2DC_and_DC2AC_uninitialized.n, CIM5.p)
          annotation (Line(points={{-40,-40},{0,-40}}, color={0,0,255}));
        connect(aC2DC_and_DC2AC_uninitialized.p, bus4_CIM5.p) annotation (Line(
              points={{-60,-40},{-100,-40},{-100,0},{80,0},{80,50},{40,50}},
              color={0,0,255}));
        connect(m_ramp.y, aC2DC_and_DC2AC_uninitialized.m_input) annotation (Line(
              points={{-55,-80},{-45.4545,-80},{-45.4545,-52}}, color={0,0,127}));
        connect(f_ramp.y, CIM5.we)
          annotation (Line(points={{1,-80},{10,-80},{10,-52}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                  -160,-160},{160,160}})),                             Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-160,-160},{
                  160,160}})));
      end CIM5_and_CIM6_VSD_Ramp_StartUp;

      model MotorTypeIII_VSD_Ramp_StartUp2
        extends Modelica.Icons.Example;


        parameter Real start_point_v = 1;
        parameter Real start_point_f = 1;
        OpenIPSL.Electrical.Machines.PSSE.GENCLS inf3(
          V_b=16000,
          v_0=1.05,
          M_b=600000000,
          H=0)
          annotation (Placement(transformation(extent={{-140,40},{-120,60}})));
        OpenIPSL.Electrical.Buses.Bus bus1_mt3(V_b=16000, v_0=1.05)
          annotation (Placement(transformation(extent={{-110,40},{-90,60}})));
        OpenIPSL.Electrical.Buses.Bus bus2_mt3(V_b=230000)
          annotation (Placement(transformation(extent={{-70,40},{-50,60}})));
        OpenIPSL.Electrical.Branches.PwLine line_mt3(
          R=0,
          X=0.02,
          G=0,
          B=0) annotation (Placement(transformation(extent={{-44,36},{-16,64}})));
        OpenIPSL.Electrical.Buses.Bus bus3_mt3(V_b=230000)
          annotation (Placement(transformation(extent={{-10,40},{10,60}})));
        OpenIPSL.Electrical.Buses.Bus bus4_mt3(V_b=23000)
          annotation (Placement(transformation(extent={{30,40},{50,60}})));
        Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor3
          annotation (Placement(transformation(extent={{34,-50},{54,-30}})));
        Modelica.Mechanics.Rotational.Sources.Torque torque3
          annotation (Placement(transformation(extent={{64,-90},{84,-70}})));
        Modelica.Mechanics.Rotational.Components.Inertia load_inertia3(J=1)
          annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={74,-40})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1_mt3(
          V_b=16000,
          Vn=16000,
          rT=0,
          xT=0.025)
                   annotation (Placement(transformation(extent={{-90,40},{-70,60}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2_mt3(
          V_b=230000,
          Vn=230000,
          rT=0,
          xT=0.15) annotation (Placement(transformation(extent={{10,40},{30,60}})));
        OpenIPSL.Electrical.Loads.PSAT.PQ Load3(V_b=230000, P_0=500000000)
          annotation (Placement(transformation(extent={{-20,12},{0,32}})));
        inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
          annotation (Placement(transformation(extent={{80,80},{140,120}})));
        MultiDomainModels.Motors.ThreePhase.PSAT.MotorTypeIII_different Motor3(
          V_b=23000,
            H=0.4,
          Rs=0,
          R1=0.01)
          annotation (Placement(transformation(extent={{20,-50},{0,-30}})));
        Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC aC2DC_and_DC2AC(V_b=23000)
          annotation (Placement(transformation(extent={{-60,-50},{-40,-30}})));
        Modelica.Blocks.Sources.Ramp change(
          height=0,
          duration=0,
          offset=start_point_v)
          annotation (Placement(transformation(extent={{-120,-90},{-100,-70}})));
        Modelica.Blocks.Math.Gain pwm_modulation_index(k=1)
          annotation (Placement(transformation(extent={{-80,-90},{-60,-70}})));
        Modelica.Blocks.Math.Gain synchronous_speed(k=2*Modelica.Constants.pi*SysData.fn)
          annotation (Placement(transformation(extent={{-80,-140},{-60,-120}})));
        Modelica.Blocks.Sources.Ramp change1(
          height=0,
          duration=0,
          offset=start_point_f)
          annotation (Placement(transformation(extent={{-120,-140},{-100,-120}})));
        Modelica.Blocks.Sources.RealExpression Torque3(y=-(0.1*(15/100)*(Motor3.s) + 0.5
              *(15/100)*(1 - Motor3.s)^2)*Motor3.T_b)
          annotation (Placement(transformation(extent={{24,-90},{44,-70}})));
      equation
        connect(inf3.p,bus1_mt3. p)
          annotation (Line(points={{-120,50},{-100,50}},
                                                       color={0,0,255}));
        connect(bus2_mt3.p,line_mt3. p)
          annotation (Line(points={{-60,50},{-42.6,50}},
                                                       color={0,0,255}));
        connect(line_mt3.n,bus3_mt3. p)
          annotation (Line(points={{-17.4,50},{0,50}},
                                                     color={0,0,255}));
        connect(torqueSensor3.flange_b,load_inertia3. flange_a)
          annotation (Line(points={{54,-40},{64,-40}},
                                                     color={0,0,0}));
        connect(torque3.flange,load_inertia3. flange_b) annotation (Line(points={{84,-80},
                {94,-80},{94,-40},{84,-40}},     color={0,0,0}));
        connect(bus1_mt3.p,tf1_mt3. p)
          annotation (Line(points={{-100,50},{-91,50}},
                                                      color={0,0,255}));
        connect(tf1_mt3.n,bus2_mt3. p)
          annotation (Line(points={{-69,50},{-60,50}},
                                                     color={0,0,255}));
        connect(bus3_mt3.p,tf2_mt3. p)
          annotation (Line(points={{0,50},{9,50}},
                                                 color={0,0,255}));
        connect(tf2_mt3.n,bus4_mt3. p)
          annotation (Line(points={{31,50},{40,50}},
                                                   color={0,0,255}));
        connect(Load3.p,bus3_mt3. p)
          annotation (Line(points={{-10,32},{-10,50},{0,50}},color={0,0,255}));
        connect(torqueSensor3.tau,Motor3. mech_torque) annotation (Line(points={{36,-51},
                {36,-60},{16,-60},{16,-52}}, color={0,0,127}));
        connect(Motor3.flange,torqueSensor3. flange_a)
          annotation (Line(points={{20,-40},{34,-40}},
                                                   color={0,0,0}));
        connect(aC2DC_and_DC2AC.n, Motor3.p)
          annotation (Line(points={{-40,-40},{0,-40}}, color={0,0,255}));
        connect(aC2DC_and_DC2AC.p, bus4_mt3.p) annotation (Line(points={{-60,-40},{-100,
                -40},{-100,0},{80,0},{80,50},{40,50}}, color={0,0,255}));
        connect(change.y, pwm_modulation_index.u)
          annotation (Line(points={{-99,-80},{-82,-80}}, color={0,0,127}));
        connect(pwm_modulation_index.y, aC2DC_and_DC2AC.m_input) annotation (Line(
              points={{-59,-80},{-45.4545,-80},{-45.4545,-52}}, color={0,0,127}));
        connect(synchronous_speed.y, Motor3.we)
          annotation (Line(points={{-59,-130},{10,-130},{10,-52}}, color={0,0,127}));
        connect(change1.y, synchronous_speed.u)
          annotation (Line(points={{-99,-130},{-82,-130}}, color={0,0,127}));
        connect(Torque3.y, torque3.tau)
          annotation (Line(points={{45,-80},{62,-80}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
                  -160,-160},{160,160}})),                             Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-160,-160},{
                  160,160}})),
          experiment(
            StopTime=10,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end MotorTypeIII_VSD_Ramp_StartUp2;

    end VSDMotorStartUp;
  end ThreePhaseInductionMotors;
end MultiDomainExamples;
