within OpenIMDML.Examples.BaseClasses;
model ValidationPartial3 "Partial model 2 for validation purposes"
  extends Modelica.Icons.Example;
  OpenIPSL.Electrical.Machines.PSSE.GENCLS inf1(
    V_b=16000,
    v_0=1.05,
    M_b=600000000,
    H=0,
    X_d=0)
    annotation (Placement(transformation(extent={{-154,40},{-134,60}})));
  OpenIPSL.Electrical.Buses.Bus bus1_mt1(V_b=16000, v_0=1.05)
    annotation (Placement(transformation(extent={{-130,40},{-110,60}})));
  OpenIPSL.Electrical.Buses.Bus bus2_mt1(V_b=230000)
    annotation (Placement(transformation(extent={{-90,40},{-70,60}})));
  OpenIPSL.Electrical.Branches.PwLine line_mt1(
    R=0,
    X=0.02,
    G=0,
    B=0) annotation (Placement(transformation(extent={{-72,36},{-44,64}})));
  OpenIPSL.Electrical.Buses.Bus bus3_mt1(V_b=230000)
    annotation (Placement(transformation(extent={{-30,40},{-10,60}})));
  OpenIPSL.Electrical.Buses.Bus bus4_mt1(V_b=23000)
    annotation (Placement(transformation(extent={{10,40},{30,60}})));
  OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1_mt1(
    V_b=16000,
    Vn=16000,
    rT=0,
    xT=0.025)
             annotation (Placement(transformation(extent={{-110,40},{-90,60}})));
  OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2_mt1(
    V_b=230000,
    Vn=230000,
    rT=0,
    xT=0.15) annotation (Placement(transformation(extent={{-10,40},{10,60}})));
  OpenIPSL.Electrical.Loads.PSAT.PQ Load1(V_b=230000, P_0=500000000)
    annotation (Placement(transformation(extent={{-40,10},{-20,30}})));
  inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
    annotation (Placement(transformation(extent={{96,80},{150,104}})));
  OpenIPSL.Electrical.Machines.PSSE.GENCLS inf2(
    V_b=16000,
    v_0=1.05,
    M_b=600000000,
    H=0,
    X_d=0)
    annotation (Placement(transformation(extent={{-154,-60},{-134,-40}})));
  OpenIPSL.Electrical.Buses.Bus bus1_mt2(V_b=16000, v_0=1.05)
    annotation (Placement(transformation(extent={{-130,-60},{-110,-40}})));
  OpenIPSL.Electrical.Buses.Bus bus2_mt2(V_b=230000)
    annotation (Placement(transformation(extent={{-90,-60},{-70,-40}})));
  OpenIPSL.Electrical.Branches.PwLine line_mt2(
    R=0,
    X=0.02,
    G=0,
    B=0) annotation (Placement(transformation(extent={{-72,-64},{-44,-36}})));
  OpenIPSL.Electrical.Buses.Bus bus3_mt2(V_b=230000)
    annotation (Placement(transformation(extent={{-30,-60},{-10,-40}})));
  OpenIPSL.Electrical.Buses.Bus bus4_mt2(V_b=23000)
    annotation (Placement(transformation(extent={{10,-60},{30,-40}})));
  OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1_mt2(
    V_b=16000,
    Vn=16000,
    rT=0,
    xT=0.025)
             annotation (Placement(transformation(extent={{-110,-60},{-90,-40}})));
  OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2_mt2(
    V_b=230000,
    Vn=230000,
    rT=0,
    xT=0.15) annotation (Placement(transformation(extent={{-10,-60},{10,-40}})));
  OpenIPSL.Electrical.Loads.PSAT.PQ Load2(V_b=230000, P_0=500000000)
    annotation (Placement(transformation(extent={{-40,-90},{-20,-70}})));
equation
  connect(inf1.p,bus1_mt1. p)
    annotation (Line(points={{-134,50},{-120,50}}, color={0,0,255}));
  connect(bus2_mt1.p,line_mt1. p)
    annotation (Line(points={{-80,50},{-70.6,50}}, color={0,0,255}));
  connect(line_mt1.n,bus3_mt1. p)
    annotation (Line(points={{-45.4,50},{-20,50}},
                                                 color={0,0,255}));
  connect(bus1_mt1.p,tf1_mt1. p)
    annotation (Line(points={{-120,50},{-111,50}},color={0,0,255}));
  connect(tf1_mt1.n,bus2_mt1. p)
    annotation (Line(points={{-89,50},{-80,50}}, color={0,0,255}));
  connect(bus3_mt1.p,tf2_mt1. p)
    annotation (Line(points={{-20,50},{-11,50}},
                                             color={0,0,255}));
  connect(tf2_mt1.n,bus4_mt1. p)
    annotation (Line(points={{11,50},{20,50}}, color={0,0,255}));
  connect(Load1.p,bus3_mt1. p)
    annotation (Line(points={{-30,30},{-30,50},{-20,50}},
                                                        color={0,0,255}));
  connect(inf2.p,bus1_mt2. p)
    annotation (Line(points={{-134,-50},{-120,-50}},
                                                   color={0,0,255}));
  connect(bus2_mt2.p,line_mt2. p)
    annotation (Line(points={{-80,-50},{-70.6,-50}},
                                                   color={0,0,255}));
  connect(line_mt2.n,bus3_mt2. p)
    annotation (Line(points={{-45.4,-50},{-20,-50}},
                                                 color={0,0,255}));
  connect(bus1_mt2.p,tf1_mt2. p)
    annotation (Line(points={{-120,-50},{-111,-50}},
                                                  color={0,0,255}));
  connect(tf1_mt2.n,bus2_mt2. p)
    annotation (Line(points={{-89,-50},{-80,-50}},
                                                 color={0,0,255}));
  connect(bus3_mt2.p,tf2_mt2. p)
    annotation (Line(points={{-20,-50},{-11,-50}},
                                             color={0,0,255}));
  connect(tf2_mt2.n,bus4_mt2. p)
    annotation (Line(points={{11,-50},{20,-50}},
                                               color={0,0,255}));
  connect(Load2.p,bus3_mt2. p)
    annotation (Line(points={{-30,-70},{-30,-50},{-20,-50}},
                                                        color={0,0,255}));
  annotation (preferredView = "info",Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
            -120},{160,120}})),
                          Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-160,-120},{160,120}})),
    Documentation(info="<html>
<p>Validation Partial System 3. </p>
</html>"));
end ValidationPartial3;
