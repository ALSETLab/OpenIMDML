within OpenIMDML.Examples.MultiDomainExamples;
package MultiDomainApplications "Coupling OpenIMDML + other library examples"
  extends Modelica.Icons.ExamplesPackage;

  model TankExample "Motor filling a tank example"
    extends BaseClasses.ValidationPartial2;
    Modelica.Fluid.Sources.FixedBoundary SOURCE(
      p=system.p_ambient,
      T=303.15,
      redeclare package Medium = Modelica.Media.Water.WaterIF97OnePhase_ph,
      nPorts=1)
      annotation (Placement(transformation(extent={{28,-50},{48,-30}})));
    Modelica.Fluid.Machines.Pump pump(
      redeclare package Medium = Modelica.Media.Water.WaterIF97OnePhase_ph,
      redeclare function flowCharacteristic =
          Modelica.Fluid.Machines.BaseClasses.PumpCharacteristics.linearFlow (
            V_flow_nominal={0,0.1}, head_nominal={10,0}),
      N_nominal(displayUnit="rad/s") = 3500,
      use_powerCharacteristic=false,
      redeclare function powerCharacteristic =
          Modelica.Fluid.Machines.BaseClasses.PumpCharacteristics.quadraticPower,
      V=5)
      annotation (Placement(transformation(extent={{60,-50},{80,-30}})));

    Modelica.Fluid.Pipes.StaticPipe pipe(
      allowFlowReversal=true,
      length=500,
      height_ab=0,
      diameter=1,
      redeclare package Medium = Modelica.Media.Water.WaterIF97OnePhase_ph)
      annotation (Placement(transformation(
          origin={121,-40},
          extent={{-9,-10},{11,10}},
          rotation=0)));
    Modelica.Fluid.Vessels.OpenTank reservoir(
      T_start=Modelica.Units.Conversions.from_degC(20),
      use_portsData=true,
      crossArea=1,
      level_start=0,
      height=5,
      portsData={Modelica.Fluid.Vessels.BaseClasses.VesselPortsData(diameter=
          0.1)},
      redeclare package Medium = Modelica.Media.Water.WaterIF97OnePhase_ph,
      nPorts=1)
      annotation (Placement(transformation(extent={{120,0},{152,32}})));
    MultiDomain.Motors.MD_All_In_One_ThreePhaseMotor motor(
      M_b=10000,
      Sup=true,
      Ctrl=false,
      H=0.4,
      redeclare OpenIMDML.MultiDomain.Motors.ThreePhase.PSSE.MD_CIM motor)
               annotation (Placement(transformation(extent={{30,-10},{50,10}})));
    Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor1
      annotation (Placement(transformation(extent={{56,-10},{76,10}})));
    Modelica.Mechanics.Rotational.Components.Inertia load_inertia1(J=0.5)
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={94,0})));
    inner Modelica.Fluid.System system(
      T_ambient=323.15,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      m_flow_start=0)                annotation (Placement(transformation(extent={{-88,30},
              {-68,50}})));
    Modelica.Blocks.Sources.Ramp valveClosing(
      height=-0.9,
      duration=1,
      startTime=25,
      offset=1)
      annotation (Placement(transformation(extent={{60,-90},{80,-70}})));
    Modelica.Fluid.Valves.ValveIncompressible clogged_pipe(
      redeclare package Medium = Modelica.Media.Water.WaterIF97OnePhase_ph,
      dp_nominal=10000,
      m_flow_nominal=243.912)
      annotation (Placement(transformation(extent={{86,-30},{106,-50}})));
      OpenIPSL.Electrical.Events.PwFault fault(
      t1=35,
      t2=35.3,
      R=0.05,
      X=0.05) annotation (Placement(transformation(extent={{-80,-50},{-60,-30}})));
  equation
    connect(bus4_mt1.p, motor.pwpin)
      annotation (Line(points={{20,0},{30,0}}, color={0,0,255}));
    connect(SOURCE.ports[1], pump.port_a)
      annotation (Line(points={{48,-40},{60,-40}}, color={0,127,255}));
    connect(pipe.port_b, reservoir.ports[1]) annotation (Line(points={{132,-40},
            {136,-40},{136,0}}, color={0,127,255}));
    connect(load_inertia1.flange_b, pump.shaft) annotation (Line(points={{104,0},
            {110,0},{110,-18},{70,-18},{70,-30}}, color={0,0,0}));
    connect(motor.flange1, torqueSensor1.flange_a)
      annotation (Line(points={{50,0},{56,0}}, color={0,0,0}));
    connect(torqueSensor1.flange_b, load_inertia1.flange_a)
      annotation (Line(points={{76,0},{84,0}}, color={0,0,0}));
    connect(torqueSensor1.tau, motor.mech_torque) annotation (Line(points={{58,
            -11},{58,-18},{46,-18},{46,-12}}, color={0,0,127}));
    connect(clogged_pipe.port_b, pipe.port_a)
      annotation (Line(points={{106,-40},{112,-40}}, color={0,127,255}));
    connect(pump.port_b, clogged_pipe.port_a)
      annotation (Line(points={{80,-40},{86,-40}}, color={0,127,255}));
    connect(valveClosing.y, clogged_pipe.opening)
      annotation (Line(points={{81,-80},{96,-80},{96,-48}}, color={0,0,127}));
    connect(fault.p, tf1_mt1.n) annotation (Line(points={{-81.6667,-40},{-90,-40},{-90,0},{-89,0}},
                              color={0,0,255}));
    annotation (preferredView="info", Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
              -100},{160,80}})),                                   Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-160,-100},{160,
              80}})),
      experiment(
        StopTime=100,
        __Dymola_NumberOfIntervals=10000,
        __Dymola_Algorithm="Dassl"),
      Documentation(info="<html>
<p>The TankExample displays a simple test case where the multi-domain motor model typeI is being used to drive a pump that fills the reservoir.</p>
<p>A restriction in the water flow through the piping system emulates a clogged pipe, which is reflected in the water volume and level in the reservoir component. The clogging of the pipe starts at t=25 s. </p>
<p>Simulate the system for 100 seconds. Variables of interest and comparison are:</p>
<ul>
<li><span style=\"font-family: Courier New;\">Motor1.Imag</span></li>
<li><span style=\"font-family: Courier New;\">Motor1.s</span></li>
<li><span style=\"font-family: Courier New;\">Motor1.wr</span></li>
<li><span style=\"font-family: Courier New;\">Motor1.P</span></li>
<li><span style=\"font-family: Courier New;\">Motor1.Q</span></li>
<li><span style=\"font-family: Courier New;\">reservoir.level</span></li>
<li><span style=\"font-family: Courier New;\">reservoir.V</span></li>
</ul>
</html>"));
  end TankExample;
    annotation (preferredView="info",
    Documentation);
end MultiDomainApplications;
