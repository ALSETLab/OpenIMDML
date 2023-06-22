within OpenIMDML.Examples.BaseClasses;
partial model ValidationPartial1
  "Partial model 1 for validation purposes"

  extends Modelica.Icons.Example;
  OpenIPSL.Electrical.Buses.Bus inf_bus(V_b=230)
    annotation (Placement(transformation(extent={{-96,-10},{-76,10}})));
  OpenIPSL.Electrical.Machines.PSSE.GENCLS Inf(V_b=230, X_d=0)
    annotation (Placement(transformation(extent={{-114,-10},{-94,10}})));
  OpenIPSL.Electrical.Buses.Bus load_bus(V_b=230)
    annotation (Placement(transformation(extent={{-26,-10},{-6,10}})));
  OpenIPSL.Electrical.Branches.PwLine Line(
    G=0,
    B=0,
    R=2.50000E-3,
    X=2.50000E-3)
    annotation (Placement(transformation(extent={{-62,6},{-42,26}})));
  OpenIPSL.Electrical.Branches.PwLine Line1(
    G=0,
    B=0,
    R=0.5*2.50000E-3,
    X=0.5*2.50000E-3)
    annotation (Placement(transformation(extent={{-78,-26},{-58,-6}})));
  OpenIPSL.Electrical.Branches.PwLine Line2(
    G=0,
    B=0,
    R=0.5*2.50000E-3,
    X=0.5*2.50000E-3)
    annotation (Placement(transformation(extent={{-50,-26},{-30,-6}})));
  inner OpenIPSL.Electrical.SystemBase SysData(fn=60, S_b=100000)    annotation (Placement(transformation(extent={{-100,40},
            {-60,60}})));
equation
  connect(Inf.p, inf_bus.p)
    annotation (Line(points={{-94,0},{-86,0}}, color={0,0,255}));
  connect(Line.n,load_bus. p)
    annotation (Line(points={{-43,16},{-28,16},{-28,0},{-16,0}},
                                                             color={0,0,255}));
  connect(inf_bus.p,Line. p) annotation (Line(points={{-86,0},{-80,0},{-80,16},{
          -61,16}}, color={0,0,255}));
  connect(Line1.p,Line. p) annotation (Line(points={{-77,-16},{-80,-16},{-80,16},
          {-61,16}},color={0,0,255}));
  connect(Line1.n,Line2. p)
    annotation (Line(points={{-59,-16},{-49,-16}},
                                                color={0,0,255}));
  connect(Line2.n,load_bus. p) annotation (Line(points={{-31,-16},{-28,-16},{-28,
          0},{-16,0}},
                   color={0,0,255}));
  annotation (preferredView = "info",Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},
            {120,100}})),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},{120,100}})),
    Documentation(info="<html>
<p>Validation Partial System 1. </p>
</html>"));
end ValidationPartial1;
