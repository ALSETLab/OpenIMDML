within OpenIMDML;
package Sandbox
  model MotorTypeIII_VSD_Ramp_StartUp2
    extends Modelica.Icons.Example;

    parameter Real start_point_v = 1;
    parameter Real start_point_f = 1;
    OpenIPSL.Electrical.Machines.PSSE.GENCLS inf3(
      V_b=16000,
      v_0=1.05,
      M_b=600000000,
      H=0)
      annotation (Placement(transformation(extent={{-154,40},{-134,60}})));
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
      Ctrl=false,
      Rs=0)
      annotation (Placement(transformation(extent={{20,-50},{0,-30}})));
    Controls.VariableSpeedDrive.Power_Electronics.AC2DC_and_DC2AC vsd(V_b=23000)
      annotation (Placement(transformation(extent={{-60,-50},{-40,-30}})));
    Modelica.Blocks.Sources.RealExpression Torque3(y=-(0.1*(15/100)*(Motor3.s) + 0.5
          *(15/100)*(1 - Motor3.s)^2)*Motor3.T_b)
      annotation (Placement(transformation(extent={{24,-90},{44,-70}})));
    Modelica.Blocks.Interfaces.RealInput dm
      annotation (Placement(transformation(extent={{-200,-20},{-160,20}})));
    Modelica.Blocks.Interfaces.RealOutput y
      annotation (Placement(transformation(extent={{140,-150},{160,-130}})));
    Modelica.Blocks.Interfaces.RealOutput y1
      annotation (Placement(transformation(extent={{140,-130},{160,-110}})));
    Modelica.Blocks.Math.Add add
      annotation (Placement(transformation(extent={{-122,-16},{-102,4}})));
    Modelica.Blocks.Sources.Constant m0(k=1)
      annotation (Placement(transformation(extent={{-154,-50},{-134,-30}})));
    Modelica.Blocks.Interfaces.RealOutput y2
      annotation (Placement(transformation(extent={{140,-106},{160,-86}})));
    Modelica.Blocks.Sources.RealExpression realExpression(y=Motor3.s)
      annotation (Placement(transformation(extent={{110,-106},{130,-86}})));
    Modelica.Blocks.Sources.RealExpression realExpression1(y=bus3_mt3.V)
      annotation (Placement(transformation(extent={{112,-70},{132,-50}})));
    Modelica.Blocks.Interfaces.RealOutput y3
      annotation (Placement(transformation(extent={{142,-70},{162,-50}})));
    OpenIPSL.Electrical.Events.PwFault pwFault(
      R=0.1,
      X=1,
      t1=10,
      t2=10.5)
      annotation (Placement(transformation(extent={{-50,20},{-38,32}})));
  equation
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
    connect(vsd.n, Motor3.p)
      annotation (Line(points={{-40,-40},{0,-40}}, color={0,0,255}));
    connect(vsd.p, bus4_mt3.p) annotation (Line(points={{-60,-40},{-80,-40},{
            -80,0},{100,0},{100,50},{40,50}}, color={0,0,255}));
    connect(Torque3.y, torque3.tau)
      annotation (Line(points={{45,-80},{62,-80}}, color={0,0,127}));
    connect(y, vsd.Vc) annotation (Line(points={{150,-140},{-54.5455,-140},{
            -54.5455,-52}}, color={0,0,127}));
    connect(y1, Motor3.wr) annotation (Line(points={{150,-120},{78,-120},{78,
            -130},{4,-130},{4,-52}}, color={0,0,127}));
    connect(m0.y, add.u2) annotation (Line(points={{-133,-40},{-128,-40},{-128,
            -12},{-124,-12}}, color={0,0,127}));
    connect(dm, add.u1)
      annotation (Line(points={{-180,0},{-124,0}}, color={0,0,127}));
    connect(add.y, vsd.m_input) annotation (Line(points={{-101,-6},{-90,-6},{
            -90,-60},{-45.4545,-60},{-45.4545,-52}}, color={0,0,127}));
    connect(realExpression.y, y2)
      annotation (Line(points={{131,-96},{150,-96}}, color={0,0,127}));
    connect(realExpression1.y, y3)
      annotation (Line(points={{133,-60},{152,-60}}, color={0,0,127}));
    connect(inf3.p, bus1_mt3.p)
      annotation (Line(points={{-134,50},{-100,50}}, color={0,0,255}));
    connect(pwFault.p, bus2_mt3.p) annotation (Line(points={{-51,26},{-54,26},{
            -54,36},{-48,36},{-48,50},{-60,50}}, color={0,0,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{
              -160,-160},{160,160}})),                             Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-160,-160},{
              160,160}})),
      experiment(
        StopTime=40,
        __Dymola_NumberOfIntervals=20000,
        __Dymola_Algorithm="Dassl"));
  end MotorTypeIII_VSD_Ramp_StartUp2;

  function Linearize_Model

  end Linearize_Model;

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
        StopTime=10,
        __Dymola_NumberOfIntervals=5000,
        __Dymola_Algorithm="Dassl"));
  end MotorTypeIII_Validation;
end Sandbox;
