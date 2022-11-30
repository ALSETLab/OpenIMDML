within OpenIMDML.Examples.MultiDomainExamples;
package MultiDomainApplications "Coupling OpenIMDML + other library examples"
  extends Modelica.Icons.ExamplesPackage;

  model TankExample
    extends BaseClasses.ValidationPartial2;
    Modelica.Fluid.Sources.FixedBoundary SOURCE(
      p=system.p_ambient,
      T=303.15,
      redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater,
      nPorts=1)
      annotation (Placement(transformation(extent={{28,-50},{48,-30}})));
    Modelica.Fluid.Machines.Pump pump(
      redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater,
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
      redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater)
      annotation (Placement(transformation(
          origin={121,-40},
          extent={{-9,-10},{11,10}},
          rotation=0)));
    Modelica.Fluid.Vessels.OpenTank reservoir(
      T_start=Modelica.Units.Conversions.from_degC(20),
      use_portsData=true,
      crossArea=5,
      level_start=0,
      height=5,
      portsData={Modelica.Fluid.Vessels.BaseClasses.VesselPortsData(diameter=
          0.1)},
      redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater,
      nPorts=1)
      annotation (Placement(transformation(extent={{120,0},{152,32}})));
    MultiDomainModels.Motors.MD_ALL_IN_ONE_ThreePhaseMotor motor(
      M_b=10000,
      Sup=true,
      Ctrl=false,
      redeclare
        OpenIMDML.MultiDomainModels.Motors.ThreePhase.PSAT.MD_MotorTypeI motor(
          R1=0.08))
      annotation (Placement(transformation(extent={{30,-10},{50,10}})));
    Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor1
      annotation (Placement(transformation(extent={{56,-10},{76,10}})));
    Modelica.Mechanics.Rotational.Components.Inertia load_inertia1(J=2, w(fixed
          =false, start=0.0001))
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={94,0})));
    inner Modelica.Fluid.System system(
      T_ambient=323.15,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      m_flow_start=0)                annotation (Placement(transformation(extent={{70,60},
              {90,80}})));
    Modelica.Blocks.Sources.Ramp valveClosing(
      height=-0.9,
      duration=1,
      startTime=25,
      offset=1)
      annotation (Placement(transformation(extent={{60,-90},{80,-70}})));
    Modelica.Fluid.Valves.ValveIncompressible clogged_pipe(
      redeclare package Medium =
          Modelica.Media.Water.ConstantPropertyLiquidWater,
      dp_nominal=10000,
      m_flow_nominal=243.912)
      annotation (Placement(transformation(extent={{86,-30},{106,-50}})));
      OpenIPSL.Electrical.Events.PwFault fault(
      t1=35,
      t2=35.3,
      R=0.05,
      X=0.05) annotation (Placement(transformation(extent={{-80,40},{-60,60}})));
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
    connect(fault.p, tf1_mt1.n) annotation (Line(points={{-81.6667,50},{-90,50},
            {-90,0},{-89,0}}, color={0,0,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end TankExample;
end MultiDomainApplications;