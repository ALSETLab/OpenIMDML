within OpenIMDML.Examples.MultiDomainExamples;
package MultiDomainApplications "Coupling OpenIMDML + other library examples"
  extends Modelica.Icons.ExamplesPackage;

  model PumpFeedingTankWithMultiDomainMotor "A modified version of the tank filled by mechanical pump from Dassault Systemes Cooling Library"
    extends DymolaModels.Icons.Basic.Example;
    package Medium = Cooling.Media.Liquids.EthyleneGlycol20;

    parameter Real H = 0.833*2*Modelica.Constants.pi*SysData.fn;
    parameter Real K = 2*Modelica.Constants.pi/60;
    Cooling.Sources.LiquidPressureBoundary inlet(
      redeclare package Medium=Medium,
      N=1,
      p_set=100000,
      T_set=293.15) annotation (Placement(transformation(extent={{54,22},{74,42}})));
    Cooling.Machines.LiquidPumpMechanical pump(
      redeclare package Medium=Medium,
      Nparallel=1,
      redeclare Cooling.Machines.Records.Data.PumpData.Grundfos_NBE32_160
                                                                  mapData,
      m_flowInit=Modelica.Constants.eps,
      initializeShaftAngle=false,
      m_flowNominal=2,
      d_init=1044,
      allowFlowReversal=false,
      p_aInit=100000,
      p_bInit=100000) annotation (Placement(transformation(extent={{92,22},{112,
              42}})));
    Cooling.Reservoirs.OpenLiquidTank tank(
      redeclare package LiquidMedium = Medium,
      H_init=0,
      N_ports=1,
      D=0.5,
      p_ambient=100000,
      T_liquidInit=293.15)
      annotation (Placement(transformation(extent={{152,74},{172,94}})));
    inner Cooling.Common.SystemSettings systemSettings
      annotation (Placement(transformation(extent={{-110,100},{-78,120}})));
    Cooling.Pipes.CircularLiquidPipe pipe(
      redeclare package Medium = Medium,
      d_nominal=1044,
      m_flowInit=Modelica.Constants.eps,
      m_flowNominal=1,
      useHeatTransfer=false,
      N=1,
      L=1,
      D=0.01,
      redeclare model DP = Cooling.Pipes.Friction.QuadraticNominalOpPoint,
      dp_nominal=10000,
      p_aInit=100000,
      p_bInit=100000,
      T_aInit=293.15,
      T_bInit=293.15)
      annotation (Placement(transformation(extent={{132,22},{152,42}})));
    OpenIPSL.Electrical.Machines.PSSE.GENCLS inf(
      V_b=16000,
      v_0=1.05,
      M_b=600000000,
      H=0,
      X_d=0)
           annotation (Placement(transformation(extent={{-186,54},{-166,74}})));
    OpenIPSL.Electrical.Buses.Bus bus1(V_b=16000, v_0=1.05)
      annotation (Placement(transformation(extent={{-164,54},{-144,74}})));
    OpenIPSL.Electrical.Buses.Bus bus2(V_b=230000, v_0=1.03)
      annotation (Placement(transformation(extent={{-128,54},{-108,74}})));
    OpenIPSL.Electrical.Branches.PwLine line_mt1(
      R=0,
      X=0.02,
      G=0,
      B=0) annotation (Placement(transformation(extent={{-112,50},{-84,78}})));
    OpenIPSL.Electrical.Buses.Bus bus3(V_b=230000, v_0=1.023)
      annotation (Placement(transformation(extent={{-84,54},{-64,74}})));
    OpenIPSL.Electrical.Buses.Bus bus4(V_b=23000, angle_0=0.017715091907742)
      annotation (Placement(transformation(extent={{-48,54},{-28,74}})));
    Modelica.Mechanics.Rotational.Sensors.TorqueSensor Torque_Sensor
      annotation (Placement(transformation(extent={{12,54},{32,74}})));
    Modelica.Mechanics.Rotational.Components.Inertia Load_Inertia(J=0.01)
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={86,64})));
    OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf1(
      V_b=16000,
      Vn=16000,
      rT=0,
      xT=0.025)
      annotation (Placement(transformation(extent={{-146,54},{-126,74}})));
    OpenIPSL.Electrical.Branches.PSAT.TwoWindingTransformer tf2(
      V_b=230000,
      Vn=230000,
      rT=0,
      xT=0.15) annotation (Placement(transformation(extent={{-66,54},{-46,74}})));
    MultiDomainModels.Motors.MD_ALL_IN_ONE_ThreePhaseMotor motor(
      M_b=5000,
      Sup=true,
      redeclare
        OpenIMDML.MultiDomainModels.Motors.ThreePhase.PSAT.MD_MotorTypeV motor)
      annotation (Placement(transformation(extent={{-26,54},{-6,74}})));
    OpenIPSL.Electrical.Loads.PSAT.PQ Load(V_b=230000, P_0=0)
      annotation (Placement(transformation(extent={{-90,24},{-70,44}})));
    Modelica.Blocks.Sources.Ramp ramp(
      startTime=10,
      offset=Modelica.Constants.eps,
      height=H,
      duration=100)
                   annotation (Placement(transformation(extent={{-58,12},{-38,
              32}})));
    inner OpenIPSL.Electrical.SystemBase SysData(S_b=100000000, fn=60)
      annotation (Placement(transformation(extent={{-180,100},{-118,120}})));
    Cooling.Sources.LiquidPressureBoundary inlet1(
      redeclare package Medium = Medium,
      N=1,
      p_set=100000,
      T_set=293.15) annotation (Placement(transformation(extent={{-50,-100},{
              -30,-80}})));
    Modelica.Blocks.Sources.Ramp ramp1(
      startTime=10,
      offset=0,
      height=3000,
      duration=100)
                   annotation (Placement(transformation(extent={{-92,-60},{-72,
              -40}})));
    Modelica.Blocks.Math.Gain rpmToOmega(k=K)                          "rpm to rad/s conversion"
      annotation (Placement(transformation(extent={{-62,-60},{-42,-40}})));
    Modelica.Mechanics.Rotational.Sources.Speed speed annotation (Placement(transformation(extent={{-32,-60},
              {-12,-40}})));
    Cooling.Machines.LiquidPumpMechanical pump1(
      redeclare package Medium = Medium,
      Nparallel=1,
      redeclare Cooling.Machines.Records.Data.PumpData.Grundfos_NBE32_160
        mapData,
      m_flowInit=0,
      initializeShaftAngle=false,
      m_flowNominal=2,
      d_init=1044,
      allowFlowReversal=false,
      p_aInit=100000,
      p_bInit=100000) annotation (Placement(transformation(extent={{-12,-100},{
              8,-80}})));
    Cooling.Reservoirs.OpenLiquidTank
                              tank1(
      redeclare package LiquidMedium = Medium,
      H_init=0,
      N_ports=1,
      D=0.5,
      p_ambient=100000,
      T_liquidInit=293.15)
                annotation (Placement(transformation(extent={{48,-60},{68,-40}})));
    Cooling.Pipes.CircularLiquidPipe
                             pipe1(
      redeclare package Medium = Medium,
      d_nominal=1044,
      m_flowInit=0,
      m_flowNominal=1,
      useHeatTransfer=false,
      N=1,
      L=1,
      D=0.01,
      redeclare model DP = Cooling.Pipes.Friction.QuadraticNominalOpPoint,
      dp_nominal=10000,
      p_aInit=100000,
      p_bInit=100000,
      T_aInit=293.15,
      T_bInit=293.15)                                               annotation (Placement(transformation(extent={{28,-100},
              {48,-80}})));
    Modelica.Mechanics.Rotational.Components.SpringDamper springDamper(c=0.5, d=
          0.5) annotation (Placement(transformation(extent={{42,54},{62,74}})));
  equation

    connect(inlet.port[1], pump.port_a) annotation (Line(points={{74,32},{92,32}},               color={255,127,36}));
    connect(pipe.port_b, tank.port[1]) annotation (Line(points={{152,32},{182,
            32},{182,84},{171.8,84}},                                                       color={255,127,36}));
    connect(pump.port_b, pipe.port_a) annotation (Line(points={{112,32},{132,32}},         color={255,127,36}));
    connect(inf.p,bus1. p)
      annotation (Line(points={{-166,64},{-154,64}},
                                                   color={0,0,255}));
    connect(bus2.p,line_mt1. p)
      annotation (Line(points={{-118,64},{-110.6,64}},
                                                   color={0,0,255}));
    connect(line_mt1.n,bus3. p)
      annotation (Line(points={{-85.4,64},{-74,64}},
                                                  color={0,0,255}));
    connect(bus1.p,tf1. p)
      annotation (Line(points={{-154,64},{-147,64}},
                                                  color={0,0,255}));
    connect(tf1.n,bus2. p)
      annotation (Line(points={{-125,64},{-118,64}},
                                                 color={0,0,255}));
    connect(bus3.p,tf2. p)
      annotation (Line(points={{-74,64},{-67,64}},
                                              color={0,0,255}));
    connect(tf2.n,bus4. p)
      annotation (Line(points={{-45,64},{-38,64}},
                                               color={0,0,255}));
    connect(Load.p,bus3. p)
      annotation (Line(points={{-80,44},{-80,64},{-74,64}},
                                                          color={0,0,255}));
    connect(bus4.p,motor. pwpin)
      annotation (Line(points={{-38,64},{-26,64}},
                                               color={0,0,255}));
    connect(motor.flange1,Torque_Sensor. flange_a)
      annotation (Line(points={{-6,64},{12,64}},
                                               color={0,0,0}));
    connect(Load_Inertia.flange_b, pump.flange)
      annotation (Line(points={{96,64},{102,64},{102,42}}, color={0,0,0}));
    connect(Torque_Sensor.tau, motor.mech_torque) annotation (Line(points={{14,53},
            {14,46},{-10,46},{-10,52}},     color={0,0,127}));
    connect(ramp.y, motor.we)
      annotation (Line(points={{-37,22},{-16,22},{-16,52}}, color={0,0,127}));
    connect(pump1.flange, speed.flange) annotation (Line(points={{-2,-80},{0,
            -80},{0,-50},{-12,-50}}, color={0,0,0}));
    connect(speed.w_ref,rpmToOmega. y) annotation (Line(points={{-34,-50},{-41,
            -50}},                                                                           color={0,0,127}));
    connect(ramp1.y, rpmToOmega.u)
      annotation (Line(points={{-71,-50},{-64,-50}}, color={0,0,127}));
    connect(inlet1.port[1], pump1.port_a)
      annotation (Line(points={{-30,-90},{-12,-90}}, color={255,127,36}));
    connect(pipe1.port_b, tank1.port[1]) annotation (Line(points={{48,-90},{78,
            -90},{78,-50},{67.8,-50}}, color={255,127,36}));
    connect(pump1.port_b, pipe1.port_a)
      annotation (Line(points={{8,-90},{28,-90}}, color={255,127,36}));
    connect(Torque_Sensor.flange_b, springDamper.flange_a)
      annotation (Line(points={{32,64},{42,64}}, color={0,0,0}));
    connect(springDamper.flange_b, Load_Inertia.flange_a)
      annotation (Line(points={{62,64},{76,64}}, color={0,0,0}));
    annotation (preferredView = "info",experiment(StopTime=5000),
                                         Documentation(info="<html>
<p>A mechanical pump is connected to a short pipe and a tank. The pump speed is zero at t=0. All components are initialized
with m_flow= 0. The input of the pump driving flange is ramped up after 10s, the pump starts to fill the tank. 
At the end of the simulation, the hydrostatic pressure at the bottom of the tank reaches the maximum pressure difference
of the pump at the given speed. The tank level can be kept, but no further filling is possible. </p>

<p>The top model has a multi-domain all-in-one motor that drives the mechanical pump, while the bottom model represents speed and torque trough a ramp component.</p>
<<p>The idea behind the PumpFeedingTankWithMultiDomainMotor is to show the capabilities of the multi-domain motor, replicating the same expected response are the
model without any motor.</p>
<p>Simulate the system for 5000 seconds. Variables of interest and comparison are:</p>
<ul>
<li><code>pipe.port_a.m_flow vs pipe1.port_a.m_flow</code></li>
<li><code>tank.H vs tank1.H</code></li>
<li><code>motor.motor.wr</code></li>
<li><code>motor.motor.P</code></li>
<li><code>motor.motor.Q</code></li>
</ul>
</html>"),      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,
              -140},{200,140}})),                                  Diagram(coordinateSystem(preserveAspectRatio=false, extent={
              {-200,-140},{200,140}}), graphics={Text(
            extent={{-64,-86},{56,-146}},
            textColor={28,108,200},
            textString="Example from Cooling Library",
            textStyle={TextStyle.Bold}), Text(
            extent={{-124,98},{140,-100}},
            textColor={0,140,72},
            textStyle={TextStyle.Bold},
            textString="Electrical System and Motor (OpenIMDML) + Load (Dassault Cooling Library)")}),                   __3dsJenkins(ignore_pedantic_check=true));
  end PumpFeedingTankWithMultiDomainMotor;
end MultiDomainApplications;
