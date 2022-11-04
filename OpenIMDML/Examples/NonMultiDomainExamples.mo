within OpenIMDML.Examples;
package NonMultiDomainExamples
  extends Modelica.Icons.ExamplesPackage;
  package SinglePhaseInductionMotors
    extends Modelica.Icons.ExamplesPackage;

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
        c=0.01)
        annotation (Placement(transformation(extent={{80,-10},{60,10}})));
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
      connect(Fault.p, Line2.p) annotation (Line(points={{38.3333,-36},{6,-36},
              {6,-16},{11,-16}},
                              color={0,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=5,
          __Dymola_NumberOfIntervals=5000,
          __Dymola_Algorithm="Dassl"));
    end SinglePhaseInductionMotor;

    model DualPhaseInductionMotor
      extends Modelica.Icons.Example;

      inner OpenIPSL.Electrical.SystemBase SysData(fn=60, S_b=100000000) annotation (Placement(transformation(extent={{40,30},
                {80,50}})));
      OpenIMDML.NonMultiDomain.Motors.SinglePhase.NMD_DPIM DPIM(
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
        H=0.00005,
        a=0.000039,
        b=0,
        c=0) annotation (Placement(transformation(extent={{80,-10},{60,10}})));
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
      OpenIPSL.Electrical.Events.PwFault Fault(
        R=0.001,
        X=0.001,
        t1=50,
        t2=50.3)
                annotation (Placement(transformation(extent={{40,-46},{60,-26}})));
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
      connect(inf_bus.p, Line.p) annotation (Line(points={{-26,0},{-20,0},{-20,16},{
              -1,16}}, color={0,0,255}));
      connect(Line1.p, Line.p) annotation (Line(points={{-17,-16},{-20,-16},{-20,16},
              {-1,16}}, color={0,0,255}));
      connect(Line1.n, Line2.p)
        annotation (Line(points={{1,-16},{11,-16}}, color={0,0,255}));
      connect(Line2.n, load_bus.p) annotation (Line(points={{29,-16},{32,-16},{32,0},
              {42,0}}, color={0,0,255}));
      connect(Fault.p, Line2.p) annotation (Line(points={{38.3333,-36},{6,-36},
              {6,-16},{11,-16}},
                         color={0,0,255}));
      connect(load_bus.p, DPIM.p)
        annotation (Line(points={{42,0},{60,0}}, color={0,0,255}));
      annotation (                                        experiment(
          StopTime=5,
          __Dymola_NumberOfIntervals=10000,
          __Dymola_Algorithm="Dassl"));
    end DualPhaseInductionMotor;

  end SinglePhaseInductionMotors;

  package ThreePhaseInductionMotors
    extends Modelica.Icons.ExamplesPackage;

    package MotorValidation
      model MotorTypeI_Validation
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
              SysData.fn + 10))
                      annotation (Placement(transformation(extent={{40,-46},{60,
                  -26}})));
        NonMultiDomain.Motors.ThreePhase.PSAT.NMD_MotorTypeI Motor1(
          V_b=23000,
          Sup=true,
          N=2,
          a=0.1,
          b=0.1,
          c=0.1) annotation (Placement(transformation(extent={{100,-10},{80,10}})));
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
        connect(bus4_mt1.p, Motor1.p)
          annotation (Line(points={{60,0},{80,0}}, color={0,0,255}));
        connect(SS1.y, Motor1.we)
          annotation (Line(points={{61,-36},{96,-36},{96,-12}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
                  -100},{160,100}})),
                                Diagram(coordinateSystem(preserveAspectRatio=false,
                extent={{-160,-100},{160,100}})));
      end MotorTypeI_Validation;

      model MotorTypeIII_Validation
        extends Modelica.Icons.Example;
        OpenIPSL.Electrical.Machines.PSSE.GENCLS inf3(
          V_b=16000,
          v_0=1.05,
          M_b=600000000,
          H=0)
          annotation (Placement(transformation(extent={{-120,-10},{-100,10}})));
        OpenIPSL.Electrical.Buses.Bus bus1_mt3(V_b=16000, v_0=1.05)
          annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
        OpenIPSL.Electrical.Buses.Bus bus2_mt3(V_b=230000)
          annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
        OpenIPSL.Electrical.Branches.PwLine line_mt3(
          R=0,
          X=0.02,
          G=0,
          B=0) annotation (Placement(transformation(extent={{-24,-14},{4,14}})));
        OpenIPSL.Electrical.Buses.Bus bus3_mt3(V_b=230000)
          annotation (Placement(transformation(extent={{10,-10},{30,10}})));
        OpenIPSL.Electrical.Buses.Bus bus4_mt3(V_b=23000)
          annotation (Placement(transformation(extent={{50,-10},{70,10}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1_mt3(
          V_b=16000,
          Vn=16000,
          rT=0,
          xT=0.025)
                   annotation (Placement(transformation(extent={{-70,-10},{-50,
                  10}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2_mt3(
          V_b=230000,
          Vn=230000,
          rT=0,
          xT=0.15) annotation (Placement(transformation(extent={{30,-10},{50,10}})));
        OpenIPSL.Electrical.Loads.PSAT.PQ Load3(V_b=230000, P_0=500000000)
          annotation (Placement(transformation(extent={{0,-40},{20,-20}})));
        Modelica.Blocks.Sources.RealExpression SS3(y=2*Modelica.Constants.pi*(SysData.fn))
                      annotation (Placement(transformation(extent={{40,-46},{60,
                  -26}})));
        inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
          annotation (Placement(transformation(extent={{80,40},{140,80}})));
        NonMultiDomain.Motors.ThreePhase.PSAT.NMD_MotorTypeIII Motor3(
          V_b = 23000,
          a=0.1,
          b=0.1,
          c=0.1) annotation (Placement(transformation(extent={{100,-10},{80,10}})));
      equation
        connect(inf3.p,bus1_mt3. p)
          annotation (Line(points={{-100,0},{-80,0}},  color={0,0,255}));
        connect(bus2_mt3.p,line_mt3. p)
          annotation (Line(points={{-40,0},{-22.6,0}}, color={0,0,255}));
        connect(line_mt3.n,bus3_mt3. p)
          annotation (Line(points={{2.6,0},{20,0}},  color={0,0,255}));
        connect(bus1_mt3.p,tf1_mt3. p)
          annotation (Line(points={{-80,0},{-71,0}},  color={0,0,255}));
        connect(tf1_mt3.n,bus2_mt3. p)
          annotation (Line(points={{-49,0},{-40,0}}, color={0,0,255}));
        connect(bus3_mt3.p,tf2_mt3. p)
          annotation (Line(points={{20,0},{29,0}},
                                                 color={0,0,255}));
        connect(tf2_mt3.n,bus4_mt3. p)
          annotation (Line(points={{51,0},{60,0}}, color={0,0,255}));
        connect(Load3.p,bus3_mt3. p)
          annotation (Line(points={{10,-20},{10,0},{20,0}},  color={0,0,255}));
        connect(bus4_mt3.p, Motor3.p)
          annotation (Line(points={{60,0},{80,0}}, color={0,0,255}));
        connect(SS3.y, Motor3.we)
          annotation (Line(points={{61,-36},{96,-36},{96,-12}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
                  -100},{160,100}})),                                  Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-160,-100},{
                  160,100}})));
      end MotorTypeIII_Validation;
      extends Modelica.Icons.ExamplesPackage;
      model MotorTypeV_Validation
        extends Modelica.Icons.Example;
        OpenIPSL.Electrical.Machines.PSSE.GENCLS inf5(
          V_b=16000,
          v_0=1.05,
          M_b=600000000,
          H=0)
          annotation (Placement(transformation(extent={{-120,-10},{-100,10}})));
        OpenIPSL.Electrical.Buses.Bus bus1_mt5(V_b=16000, v_0=1.05)
          annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
        OpenIPSL.Electrical.Buses.Bus bus2_mt5(V_b=230000)
          annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
        OpenIPSL.Electrical.Branches.PwLine line_mt5(
          R=0,
          X=0.02,
          G=0,
          B=0) annotation (Placement(transformation(extent={{-24,-14},{4,14}})));
        OpenIPSL.Electrical.Buses.Bus bus3_mt5(V_b=230000)
          annotation (Placement(transformation(extent={{10,-10},{30,10}})));
        OpenIPSL.Electrical.Buses.Bus bus4_mt5(V_b=23000)
          annotation (Placement(transformation(extent={{50,-10},{70,10}})));
        OpenIPSL.Electrical.Loads.PSAT.PQ Load5(V_b=230000, P_0=500000000)
          annotation (Placement(transformation(extent={{0,-40},{20,-20}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1_mt5(
          V_b=16000,
          Vn=16000,
          rT=0,
          xT=0.025)
                   annotation (Placement(transformation(extent={{-70,-10},{-50,
                  10}})));
        OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2_mt5(
          V_b=230000,
          Vn=230000,
          rT=0,
          xT=0.15) annotation (Placement(transformation(extent={{30,-10},{50,10}})));
        Modelica.Blocks.Sources.RealExpression SS5(y=2*Modelica.Constants.pi*(SysData.fn))
          annotation (Placement(transformation(extent={{40,-46},{60,-26}})));
        inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
          annotation (Placement(transformation(extent={{80,40},{140,80}})));
        NonMultiDomain.Motors.ThreePhase.PSAT.NMD_MotorTypeV Motor5(
          V_b=23000,
        a=0.1,b=0.1,c=0.1)
          annotation (Placement(transformation(extent={{100,-10},{80,10}})));
      equation
        connect(inf5.p,bus1_mt5. p)
          annotation (Line(points={{-100,0},{-80,0}},      color={0,0,255}));
        connect(bus2_mt5.p,line_mt5. p)
          annotation (Line(points={{-40,0},{-22.6,0}},     color={0,0,255}));
        connect(line_mt5.n,bus3_mt5. p)
          annotation (Line(points={{2.6,0},{20,0}},      color={0,0,255}));
        connect(Load5.p,bus3_mt5. p)
          annotation (Line(points={{10,-20},{10,0},{20,0}},       color={0,0,255}));
        connect(bus1_mt5.p,tf1_mt5. p)
          annotation (Line(points={{-80,0},{-71,0}},      color={0,0,255}));
        connect(tf1_mt5.n,bus2_mt5. p)
          annotation (Line(points={{-49,0},{-40,0}},     color={0,0,255}));
        connect(bus3_mt5.p,tf2_mt5. p)
          annotation (Line(points={{20,0},{29,0}},   color={0,0,255}));
        connect(tf2_mt5.n,bus4_mt5. p)
          annotation (Line(points={{51,0},{60,0}},     color={0,0,255}));
        connect(bus4_mt5.p, Motor5.p)
          annotation (Line(points={{60,0},{80,0}}, color={0,0,255}));
        connect(SS5.y, Motor5.we)
          annotation (Line(points={{61,-36},{96,-36},{96,-12}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
                  -100},{160,100}})),                                  Diagram(
              coordinateSystem(preserveAspectRatio=false, extent={{-160,-100},{
                  160,100}})));
      end MotorTypeV_Validation;

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

      model CIM6_Validation
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
        NonMultiDomain.Motors.ThreePhase.PSSE.NMD_CIM6 CIM6(
          V_b=23000,
          Ctrl=true,
          T_nom=0.1)
          annotation (Placement(transformation(extent={{70,-10},{50,10}})));
      equation
        connect(inf1.p,bus1_mt1. p)
          annotation (Line(points={{-128,0},{-108,0}},   color={0,0,255}));
        connect(bus2_mt1.p,line_mt1. p)
          annotation (Line(points={{-68,0},{-50.6,0}},   color={0,0,255}));
        connect(line_mt1.n,bus3_mt1. p)
          annotation (Line(points={{-25.4,0},{-8,0}},  color={0,0,255}));
        connect(bus1_mt1.p,tf1_mt1. p)
          annotation (Line(points={{-108,0},{-99,0}},   color={0,0,255}));
        connect(tf1_mt1.n,bus2_mt1. p)
          annotation (Line(points={{-77,0},{-68,0}},   color={0,0,255}));
        connect(bus3_mt1.p,tf2_mt1. p)
          annotation (Line(points={{-8,0},{1,0}},  color={0,0,255}));
        connect(tf2_mt1.n,bus4_mt1. p)
          annotation (Line(points={{23,0},{32,0}},   color={0,0,255}));
        connect(Load1.p,bus3_mt1. p)
          annotation (Line(points={{-18,-20},{-18,0},{-8,0}}, color={0,0,255}));
        connect(bus4_mt1.p, CIM6.p)
          annotation (Line(points={{32,0},{50,0}}, color={0,0,255}));
        connect(SS1.y, CIM6.we)
          annotation (Line(points={{33,-36},{66,-36},{66,-12}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-160},
                  {160,160}})), Diagram(coordinateSystem(preserveAspectRatio=false,
                extent={{-160,-160},{160,160}})),
          experiment(
            StopTime=10,
            __Dymola_NumberOfIntervals=5000,
            __Dymola_Algorithm="Dassl"));
      end CIM6_Validation;
    end MotorValidation;
  end ThreePhaseInductionMotors;
end NonMultiDomainExamples;
