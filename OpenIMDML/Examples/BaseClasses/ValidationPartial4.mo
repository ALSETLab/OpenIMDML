within OpenIMDML.Examples.BaseClasses;
partial model ValidationPartial4 "Partial model 4 for validation purposes"

  extends Modelica.Icons.Example;
  OpenIPSL.Electrical.Buses.Bus inf_bus(V_b=230)
    annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
  OpenIPSL.Electrical.Machines.PSSE.GENCLS Inf(V_b=230, X_d=0)
    annotation (Placement(transformation(extent={{-114,-10},{-94,10}})));
  OpenIPSL.Electrical.Buses.Bus load_bus(V_b=230)
    annotation (Placement(transformation(extent={{-30,-10},{-10,10}})));
  OpenIPSL.Electrical.Branches.PwLine Line(
    G=0,
    B=0,
    R=1.250000E-3,
    X=1.250000E-3)
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  inner OpenIPSL.Electrical.SystemBase SysData(fn=60, S_b=100000)    annotation (Placement(transformation(extent={{-100,40},
            {-60,60}})));
equation
  connect(Inf.p, inf_bus.p)
    annotation (Line(points={{-94,0},{-80,0}}, color={0,0,255}));
  connect(Line.n,load_bus. p)
    annotation (Line(points={{-41,0},{-20,0}},               color={0,0,255}));
  connect(inf_bus.p,Line. p) annotation (Line(points={{-80,0},{-59,0}},
                    color={0,0,255}));
  annotation (preferredView = "info", Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},
            {120,100}})),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},{120,100}})),
    Documentation(info="<html>
<p>Validation Partial System 4. </p>
</html>"));
end ValidationPartial4;
