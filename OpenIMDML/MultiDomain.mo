within OpenIMDML;
package MultiDomain
  package Motors
    package SinglePhase
    end SinglePhase;

    package ThreePhase
      package PSSE
      end PSSE;

      package PSAT
        model MultiDomainMotorTypeI "MultiDomain Induction Machine - Order I"
          parameter OpenIPSL.Types.ApparentPower M_b=1 "Machine base power"
                                                                           annotation (Dialog(group="Power flow data"));
          extends OpenIPSL.Electrical.Essentials.pfComponent(
            final enabledisplayPF=false,
            final enablefn=false,
            final enableV_b=false,
            final enableangle_0=true,
            final enablev_0=true,
            final enableQ_0=false,
            final enableP_0=false,
            final enableS_b=true);

          parameter Boolean Sup = true "True: Start-up process, False: Steady-state condition" annotation (Dialog(group="Motor Setup"));
          parameter Boolean Ctrl = true "True: Model for VSD control, False: Model not controllable"
                                                                                                    annotation (Dialog(group="Motor Setup"));
          parameter OpenIPSL.Types.PerUnit S0=0 "Steady-state slip value" annotation (Dialog(group="Machine parameters",enable=(Sup == false)));

          parameter Real N = 1 "Number of pair of Poles"
                                                        annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit Xs=0.15 "Stator reactance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit Rr1=0.05 "1st cage rotor resistance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit Xr1=0.15 "1st cage rotor reactance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit Xm=5 "Magnetizing reactance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.Time Hm=3 "Inertia constant" annotation (Dialog(group="Machine parameters"));

          OpenIPSL.Types.PerUnit v(start=v_0) "Bus voltage magnitude";
          OpenIPSL.Types.Angle anglev(start=angle_0) "Bus voltage angle";
          OpenIPSL.Types.PerUnit s(start=s0);
          OpenIPSL.Types.PerUnit P;
          OpenIPSL.Types.PerUnit Q;
          OpenIPSL.Types.PerUnit P_motor;
          OpenIPSL.Types.PerUnit Q_motor;
          Modelica.Units.SI.ActivePower Power_W;
          Modelica.Units.SI.ReactivePower Power_Var;
          OpenIPSL.Types.PerUnit Re;
          OpenIPSL.Types.PerUnit Xe;
          Complex Imotor;
          OpenIPSL.Interfaces.PwPin p(
            vr(start=vr0),
            vi(start=vi0),
            ir,
            ii)
            annotation (Placement(transformation(extent={{90,-10},{110,10}})));

          Modelica.Units.SI.AngularVelocity nr;
          Modelica.Units.SI.AngularVelocity ns;

          Modelica.Mechanics.Rotational.Interfaces.Flange_b flange annotation (
              Placement(transformation(extent={{-110,-10},{-90,10}}),
                iconTransformation(extent={{-110,-10},{-90,10}})));
          Modelica.Blocks.Sources.RealExpression Rotor_Speed(y=nr) annotation (Placement(transformation(extent={{70,-10},
                    {50,10}})));
          Modelica.Mechanics.Rotational.Components.Inertia Rotor_Inertia(J=J_load)
            annotation (Placement(transformation(extent={{-20,-10},{-40,10}})));
          Modelica.Blocks.Interfaces.RealOutput wr
            "Absolute angular velocity of flange as output signal"
            annotation (Placement(transformation(extent={{-100,-50},{-120,-30}}),
                iconTransformation(
                extent={{20,-20},{-20,20}},
                rotation=90,
                origin={60,-120})));
          Modelica.Blocks.Interfaces.RealInput mech_torque annotation (Placement(
                transformation(
                extent={{-20,-20},{20,20}},
                rotation=90,
                origin={-60,-120}),
                                  iconTransformation(
                extent={{-20,-20},{20,20}},
                rotation=90,
                origin={-60,-120})));
          OpenIPSL.Types.PerUnit Tmech_pu_sys;
          OpenIPSL.Types.PerUnit Tmech_pu_motor;
          Modelica.Blocks.Interfaces.RealInput we if Ctrl annotation (Placement(transformation(
                extent={{-20,-20},{20,20}},
                rotation=90,
                origin={60,-120}), iconTransformation(
                extent={{-20,-20},{20,20}},
                rotation=90,
                origin={0,-120})));
          Modelica.Units.SI.AngularVelocity w_sync;
          OpenIPSL.Types.PerUnit Xmag;
          Modelica.Units.SI.Torque T_b;
          Modelica.Blocks.Math.Gain we_fix(k=1)
            annotation (Placement(transformation(extent={{68,-86},{80,-74}})));
          Modelica.Blocks.Sources.Constant we_source_fix(k=0)
                                                         if not Ctrl
            annotation (Placement(transformation(extent={{32,-86},{44,-74}})));
        protected
          parameter Modelica.Units.SI.Inertia J_load = 2*Hm*M_b/((2*Modelica.Constants.pi*fn/N)^2);
          parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0);
          parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0);
          parameter OpenIPSL.Types.PerUnit ir0=(P_0/S_b*vr0 + Q_0/S_b*vi0)/(vr0^2 + vi0^2);
          parameter OpenIPSL.Types.PerUnit ii0=(P_0/S_b*vi0 - Q_0/S_b*vr0)/(vr0^2 + vi0^2);
          parameter OpenIPSL.Types.PerUnit s0 = (if Sup == false then S0 else 1 - Modelica.Constants.eps);
          parameter Modelica.Units.SI.AngularVelocity w_b = 2*Modelica.Constants.pi*fn;
          Real CoB = M_b/S_b;
          Modelica.Mechanics.Rotational.Sources.Speed speed(exact=true)
                                                            annotation (Placement(
                transformation(
                extent={{-10,10},{10,-10}},
                rotation=180,
                origin={10,0})));

        initial equation
          //der(s) = if Sup == false then 0 else 1;
        equation

          //Synchronous speed based on controllable boolean parameter
          w_sync = (if Ctrl then we_fix.y else w_b);

          //Torque base
          T_b = S_b/w_sync;

          //Real and Imaginary currents drawn by the motor in the motor base
          p.ii/CoB = (-p.vr/Xmag) + (p.vi*Re - p.vr*Xe)/(Re*Re + Xe*Xe);
          p.ir/CoB =   p.vi/Xmag  + (p.vr*Re + p.vi*Xe)/(Re*Re + Xe*Xe);

          //Active and Reactive Power consumption in the system base (p.u.)
          P =   p.vr*p.ir  + p.vi*p.ii;
          Q = (-p.vr*p.ii) + p.vi*p.ir;

          //Active and Reactive Power consumption in the machine base (p.u.)
          P_motor = P/CoB;
          Q_motor = Q/CoB;

          //Active and Reactive Power consumption
          Power_W = P*S_b;
          Power_Var = Q*S_b;

          //Real and Imaginary Motor Currents in the machine base
          Imotor.re = p.ir/CoB;
          Imotor.im = p.ir/CoB;

          //Voltage and angle at the pwpin connection
          v = sqrt(p.vr^2 + p.vi^2);
          anglev = atan2(p.vi, p.vr);

          //Synchronous speed and rotor speed in rad/s
          ns = w_sync/N;
          nr = (1-s)*ns;

          //Rotor impedance
          Re = Rr1/s;
          Xe = (if Ctrl == false then Xs + Xr1 else (we_fix.y/w_b)*(Xs + Xr1));

          //Magnetization Impedance
          Xmag = (if Ctrl == false then Xm else (we_fix.y/w_b)*Xm);

          //Conversion from SI torqur to p.u. torque
          Tmech_pu_sys = mech_torque/T_b;
          Tmech_pu_motor = Tmech_pu_sys/CoB;

          //Mechanical Swing Equation
          der(s) = (if Ctrl == false then (Tmech_pu_motor - P_motor)/(2*Hm) else (Tmech_pu_motor - P_motor/(we_fix.y/w_b))/(2*Hm));

          connect(wr,Rotor_Speed. y) annotation (Line(points={{-110,-40},{40,-40},{40,0},
                  {49,0}}, color={0,0,127}));
          connect(speed.w_ref,Rotor_Speed. y)
            annotation (Line(points={{22,-1.9984e-15},{49,-1.9984e-15},{49,0}},
                                                                  color={0,0,127}));
          connect(speed.flange, Rotor_Inertia.flange_a) annotation (Line(points={{
                  1.77636e-15,7.21645e-16},{-20,0}},
                                              color={0,0,0}));
          connect(we_fix.u, we)
            annotation (Line(points={{66.8,-80},{60,-80},{60,-120}},
                                                                   color={0,0,127}));
          connect(Rotor_Inertia.flange_b, flange)
            annotation (Line(points={{-40,0},{-100,0}}, color={0,0,0}));
          connect(we_fix.u, we_source_fix.y)
            annotation (Line(points={{66.8,-80},{44.6,-80}}, color={0,0,127}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                    -100},{100,100}}),
                                 graphics={Rectangle(
                  fillColor={255,255,255},
                  extent={{-100,-100},{100,100}}),Ellipse(
                  fillColor={255,255,255},
                  extent={{-56,-58},{55.9318,54}}),Text(
                  extent={{-50,48},{50,-52}},
                  lineColor={0,0,0},
                  textString="M"),Text(
                  origin={0,-80},
                  extent={{-100,-20},{100,20}},
                  fontName="Arial",
                  lineColor={0,0,0},
                  textString="%name")}),Documentation(revisions="<html>
<table cellspacing=\"1\" cellpadding=\"1\" border=\"1\">
<tr>
<td><p>Reference</p></td>
<td><p>PSAT Manual 2.1.8</p></td>
</tr>
<tr>
<td><p>Last update</p></td>
<td>September 2015</td>
</tr>
<tr>
<td><p>Author</p></td>
<td><p>Joan Russinol, KTH Royal Institute of Technology</p></td>
</tr>
<tr>
<td><p>Contact</p></td>
<td><p>see <a href=\"modelica://OpenIPSL.UsersGuide.Contact\">UsersGuide.Contact</a></p></td>
</tr>
</table>
</html>"),  Diagram(coordinateSystem(extent={{-100,-100},{100,100}})));
        end MultiDomainMotorTypeI;
      end PSAT;
    end ThreePhase;
  end Motors;
end MultiDomain;
