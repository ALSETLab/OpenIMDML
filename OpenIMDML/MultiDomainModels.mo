within OpenIMDML;
package MultiDomainModels
  package Motors
    package SinglePhase
      model SPIM
        "This model is the steady-state circuit model of the single phase induction motor model."
        OpenIPSL.Interfaces.PwPin p
          annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));

          extends OpenIPSL.Electrical.Essentials.pfComponent(
          final enabledisplayPF=false,
          final enablefn=false,
          final enableV_b=false,
          final enableangle_0=true,
          final enablev_0=true,
          final enableQ_0=true,
          final enableP_0=true,
          final enableS_b=true);

          parameter OpenIPSL.Types.PerUnit R1 "Stator winding resistor";
          parameter OpenIPSL.Types.PerUnit R2 "Rotor winding resistor";
          parameter OpenIPSL.Types.PerUnit X1 "Stator winding reactance";
          parameter OpenIPSL.Types.PerUnit X2 "Rotor winding reactance";
          parameter OpenIPSL.Types.PerUnit Xm "Magnitization reactance value";
          parameter OpenIPSL.Types.PerUnit H "Inertia coeficient";
          parameter Real a;
          parameter Real b;
          parameter Real c;

          OpenIPSL.Types.PerUnit RF "Equivalent resistor component of the forward circuit";
          OpenIPSL.Types.PerUnit XF "Equivalent resistor component of the forward circuit";
          OpenIPSL.Types.PerUnit RB "Equivalent resistor component of the backward circuit";
          OpenIPSL.Types.PerUnit XB "Equivalent resistor component of the backward circuit";
          OpenIPSL.Types.PerUnit s(start = s0)  "Induction motor slip";
          OpenIPSL.Types.PerUnit I "Magnitude of the consumed current";
          OpenIPSL.Types.PerUnit P_AGF "Air Gap Power for forward magnetic field";
          OpenIPSL.Types.PerUnit P_AGB "Air Gap Power for reverse field";
          OpenIPSL.Types.PerUnit P( start = P_0/S_b) "Net air gap power in single-phase induction motor";
          //OpenIPSL.Types.PerUnit Q "Reactive Power consumed by the single phase induction motor model";
          OpenIPSL.Types.PerUnit Te "Electrical Torque";
          OpenIPSL.Types.PerUnit Tm "Mechanical Torque of the Load";
          OpenIPSL.Types.PerUnit Pc;
          OpenIPSL.Types.PerUnit Qc;

      protected
        parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0);
        parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0);
        parameter OpenIPSL.Types.PerUnit s0 = 0.1;
        parameter OpenIPSL.Types.PerUnit ir0=(P_0/S_b*vr0 + Q_0/S_b*vi0)/(vr0^2 + vi0^2);
        parameter OpenIPSL.Types.PerUnit ii0=(P_0/S_b*vi0 - Q_0/S_b*vr0)/(vr0^2 + vi0^2);
        parameter Real A = a + b + c;
        parameter Real B = -b - 2*c;
        parameter Real C = c;

      initial equation
        der(s) = 0;

      equation

        Pc = p.vr*p.ir + p.vi*p.ii;
        Qc = (-p.vr*p.ii) + p.vi*p.ir;

        RF = (-X2*Xm*R2/s + R2*Xm*(X2 + Xm)/s)/((R2/s)^2 + (X2 + Xm)^2);
        XF = ((R2/s)^2*Xm + X2*Xm*(X2 + Xm))/((R2/s)^2 + (X2 + Xm)^2);
        RB = (-X2*Xm*R2/(2-s) + R2*Xm*(X2 + Xm)/(2-s))/((R2/(2-s))^2 + (X2 + Xm)^2);
        XB = ((R2/(2-s))^2*Xm + X2*Xm*(X2 + Xm))/((R2/(2-s))^2 + (X2 + Xm)^2);

        p.ir = p.vr*(0.5*RF + 0.5*RB + R1)/((0.5*RF + 0.5*RB + R1)^2 + (0.5*XF + 0.5*XB + X1)^2) + p.vi*(0.5*XF + 0.5*XB + X1)/((0.5*RF + 0.5*RB + R1)^2 + (0.5*XF + 0.5*XB + X1)^2);
        p.ii = p.vi*(0.5*RF + 0.5*RB + R1)/((0.5*RF + 0.5*RB + R1)^2 + (0.5*XF + 0.5*XB + X1)^2) - p.vr*(0.5*XF + 0.5*XB + X1)/((0.5*RF + 0.5*RB + R1)^2 + (0.5*XF + 0.5*XB + X1)^2);
        I = sqrt(p.ir^2 + p.ii^2);

        P_AGF = I^2*0.5*RF;
        P_AGB = I^2*0.5*RB;
        P     = P_AGF - P_AGB;

        Te = P/(1-s);
        Tm = A + B*s + C*s^2;
        der(s) = (Tm - Te)/(2*H);
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0}),
              Text(
                extent={{-100,-60},{100,-100}},
                lineColor={28,108,200},
                textString="%name"),             Text(
                extent={{-50,50},{50,-50}},
                lineColor={0,0,0},
                textString="M"),                Ellipse(
                fillColor={255,255,255},
                extent={{-56,-56},{55.932,56}})}),                     Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end SPIM;

      package BaseClasses
        extends Modelica.Icons.BasesPackage;
        model SPIMBaseModel
          "This model is the steady-state circuit model of the single phase induction motor model."
          OpenIPSL.Interfaces.PwPin p
            annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));

            extends OpenIPSL.Electrical.Essentials.pfComponent(
            final enabledisplayPF=false,
            final enablefn=false,
            final enableV_b=false,
            final enableangle_0=true,
            final enablev_0=true,
            final enableQ_0=true,
            final enableP_0=true,
            final enableS_b=true);

            //parameter OpenIPSL.Types.PerUnit R1 "Stator winding resistor";
            //parameter OpenIPSL.Types.PerUnit R2 "Rotor winding resistor";
            //parameter OpenIPSL.Types.PerUnit X1 "Stator winding reactance";
            //parameter OpenIPSL.Types.PerUnit X2 "Rotor winding reactance";
            //parameter OpenIPSL.Types.PerUnit Xm "Magnitization reactance value";
            //parameter OpenIPSL.Types.PerUnit H "Inertia coeficient";
            //parameter Real a;
            //parameter Real b;
            //parameter Real c;

            //OpenIPSL.Types.PerUnit RF "Equivalent resistor component of the forward circuit";
            //OpenIPSL.Types.PerUnit XF "Equivalent resistor component of the forward circuit";
            //OpenIPSL.Types.PerUnit RB "Equivalent resistor component of the backward circuit";
            //OpenIPSL.Types.PerUnit XB "Equivalent resistor component of the backward circuit";
            //OpenIPSL.Types.PerUnit s  "Induction motor slip";
            //OpenIPSL.Types.PerUnit I "Magnitude of the consumed current";
            //OpenIPSL.Types.PerUnit P_AGF "Air Gap Power for forward magnetic field";
            //OpenIPSL.Types.PerUnit P_AGB "Air Gap Power for reverse field";
            //OpenIPSL.Types.PerUnit P(start = P_0/S_b) "Net air gap power in single-phase induction motor";
            //OpenIPSL.Types.PerUnit Te "Electrical Torque";
            //OpenIPSL.Types.PerUnit Tm "Mechanical Torque of the Load";
            //OpenIPSL.Types.PerUnit Pc;
            //OpenIPSL.Types.PerUnit Qc;

        protected
          parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0);
          parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0);
          parameter OpenIPSL.Types.PerUnit s0 = 0.01;
          parameter OpenIPSL.Types.PerUnit ir0=(P_0/S_b*vr0 + Q_0/S_b*vi0)/(vr0^2 + vi0^2);
          parameter OpenIPSL.Types.PerUnit ii0=(P_0/S_b*vi0 - Q_0/S_b*vr0)/(vr0^2 + vi0^2);

        initial equation
          der(s) = 0;

        equation

          Pc = p.vr*p.ir + p.vi*p.ii;
          Qc = (-p.vr*p.ii) + p.vi*p.ir;

          RF = (-X2*Xm*R2/s + R2*Xm*(X2 + Xm)/s)/((R2/s)^2 + (X2 + Xm)^2);
          XF = ((R2/s)^2*Xm + X2*Xm*(X2 + Xm))/((R2/s)^2 + (X2 + Xm)^2);
          RB = (-X2*Xm*R2/(2-s) + R2*Xm*(X2 + Xm)/(2-s))/((R2/(2-s))^2 + (X2 + Xm)^2);
          XB = ((R2/(2-s))^2*Xm + X2*Xm*(X2 + Xm))/((R2/(2-s))^2 + (X2 + Xm)^2);

          p.ir = p.vr*(0.5*RF + 0.5*RB + R1)/((0.5*RF + 0.5*RB + R1)^2 + (0.5*XF + 0.5*XB + X1)^2) + p.vi*(0.5*XF + 0.5*XB + X1)/((0.5*RF + 0.5*RB + R1)^2 + (0.5*XF + 0.5*XB + X1)^2);
          p.ii = p.vi*(0.5*RF + 0.5*RB + R1)/((0.5*RF + 0.5*RB + R1)^2 + (0.5*XF + 0.5*XB + X1)^2) - p.vr*(0.5*XF + 0.5*XB + X1)/((0.5*RF + 0.5*RB + R1)^2 + (0.5*XF + 0.5*XB + X1)^2);
          I = sqrt(p.ir^2 + p.ii^2);

          P_AGF = I^2*0.5*RF;
          P_AGB = I^2*0.5*RB;
          P     = P_AGF - P_AGB;

          Te = P/(1-s);
          Tm = A + B*s + C*s^2;
          der(s) = (Tm - Te)/(2*H);
          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                  Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,0}),
                Text(
                  extent={{-100,-60},{100,-100}},
                  lineColor={28,108,200},
                  textString="%name"),             Text(
                  extent={{-50,50},{50,-50}},
                  lineColor={0,0,0},
                  textString="M"),                Ellipse(
                  fillColor={255,255,255},
                  extent={{-56,-56},{55.932,56}})}),                     Diagram(
                coordinateSystem(preserveAspectRatio=false)));
        end SPIMBaseModel;
      end BaseClasses;
    end SinglePhase;

    package ThreePhase
      model MultiDomainMotor "All-in-one Three-Phase Induction Motor ModelFramework for all three-phase MultiDomain motor models"
        parameter OpenIPSL.Types.ApparentPower M_b = 15e6 "Machine base power"
                                                                              annotation (Dialog(group="Power flow data"));
        extends OpenIPSL.Electrical.Essentials.pfComponent(
          final enablefn=false,
          final enableV_b=false,
          final enableangle_0=false,
          final enablev_0=false,
          final enableQ_0=false,
          final enableP_0=false,
          final enabledisplayPF=true,
          final enableS_b=false);

        parameter Boolean Sup = true "True: Start-up process, False: Steady-state condition" annotation (Dialog(group="Motor Setup"));
        parameter Boolean Ctrl = true "True: Model for VSD control, False: Model not controllable"
                                                                                                  annotation (Dialog(group="Motor Setup"));
        parameter Real N = 1 "Number of pair of Poles"
                                                      annotation (Dialog(group="Machine Parameters"));
        parameter Modelica.Units.SI.Time H = 0.4 "Inertia constant"
                                                                   annotation (Dialog(group="Machine Parameters"));

        replaceable BaseClasses.BaseMultiDomainMotor motor(
          M_b=M_b,
          P_0=P_0,
          Q_0=Q_0,
          v_0=v_0,
          angle_0=angle_0,
          Sup=Sup,
          Ctrl=Ctrl,
          N=N,
          H=H) annotation (
          Dialog(group="Motor Selection"),
          choicesAllMatching=true,
          Placement(transformation(extent={{10,-10},{-10,10}})));
        OpenIPSL.Interfaces.PwPin pwpin
          annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
        Modelica.Mechanics.Rotational.Interfaces.Flange_b flange1
          annotation (Placement(transformation(extent={{90,-10},{110,10}})));
        Modelica.Blocks.Interfaces.RealOutput wr
          "Absolute angular velocity of flange as output signal" annotation (
            Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-60,-110}), iconTransformation(
              extent={{-11.7614,-11.7642},{28.2274,-51.7642}},
              rotation=270,
              origin={-28.2358,-111.761})));
        Modelica.Blocks.Interfaces.RealInput we if Ctrl annotation (Placement(transformation(
              extent={{-20,-20},{20,20}},
              rotation=90,
              origin={0,-120})));
        Modelica.Blocks.Interfaces.RealInput mech_torque annotation (Placement(
              transformation(
              extent={{-20,-20},{20,20}},
              rotation=90,
              origin={60,-120})));
      equation
        connect(motor.p, pwpin)
          annotation (Line(points={{-10,0},{-100,0}}, color={0,0,255}));
        connect(motor.flange, flange1)
          annotation (Line(points={{10,0},{100,0}}, color={0,0,0}));
        connect(motor.wr, wr) annotation (Line(points={{-6,-12},{-6,-96},{-60,-96},{-60,
                -110}}, color={0,0,127}));
        connect(motor.we, we)
          annotation (Line(points={{0,-12},{0,-120}}, color={0,0,127}));
        connect(motor.mech_torque, mech_torque) annotation (Line(points={{6,-12},{6,-96},
                {60,-96},{60,-120}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Text(
                extent={{-90,100},{90,60}},
                textColor={0,140,72},
                textString="MD ALL-IN-ONE"),
                Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,255}),
                                                 Text(
                extent={{-50,48},{50,-52}},
                lineColor={0,0,0},
                textString="M"),                Ellipse(
                fillColor={255,255,255},
                extent={{-60,-60},{60,60}}),
                                Text(
                origin={0,-80},
                extent={{-70,-10},{70,10}},
                fontName="Arial",
                lineColor={0,0,0},
                textString="%name")}),                                 Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end MultiDomainMotor;

      package PSSE

        model CIM5_CIM6 "Multi-Domain CIM5/6 Three-Phase Induction Motor Model."
          extends BaseClasses.BaseMultiDomainMotor;
          import Modelica.Constants.eps;
          import OpenIPSL.NonElectrical.Functions.SE;

          parameter Integer Mtype = 1 "1- Motor Type A; 2- Motor Type B" annotation (Dialog(group=
                  "Motor Setup"), choices(choice=1, choice=2));
          parameter OpenIPSL.Types.PerUnit Ra=0 "Stator resistance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit Xa=0.0759 "Stator reactance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit Xm=3.1241 "Magnetizing reactance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit R1=0.0085 "1st cage rotor resistance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit X1=0.0759 "1st cage rotor reactance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit R2=0 "2nd cage rotor resistance. To model single cage motor set R2 = 0." annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit X2=0 "2nd cage rotor reactance. To model single cage motor set X2 = 0." annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit E1=1 "First Saturation Voltage Value"
                                                                                annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit SE1 = 0.06 "Saturation Factor at E1" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit E2=1.2 "Second Saturation Voltage Value"
                                                                                   annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit SE2 = 0.6 "Saturation Factor at E2" annotation (Dialog(group="Machine parameters"));
          parameter Modelica.Units.SI.Time H = 0.4 "Inertia constant";

          OpenIPSL.Types.PerUnit Te_motor;
          OpenIPSL.Types.PerUnit Te_sys;

          OpenIPSL.Types.PerUnit Epr;
          OpenIPSL.Types.PerUnit Epi;
          OpenIPSL.Types.PerUnit Eppr;
          OpenIPSL.Types.PerUnit Eppi;
          OpenIPSL.Types.PerUnit Epp;
          OpenIPSL.Types.PerUnit Ekr;
          OpenIPSL.Types.PerUnit Eki;
          OpenIPSL.Types.PerUnit NUM;
          OpenIPSL.Types.PerUnit EQC;
          OpenIPSL.Types.PerUnit EQ1;
          OpenIPSL.Types.PerUnit EQ2;
          OpenIPSL.Types.PerUnit EQ3;
          OpenIPSL.Types.PerUnit EQ4;
          OpenIPSL.Types.PerUnit EQ5;
          OpenIPSL.Types.PerUnit EQ6;
          OpenIPSL.Types.PerUnit EQ7;
          OpenIPSL.Types.PerUnit EQ8;
          OpenIPSL.Types.PerUnit EQ9;
          OpenIPSL.Types.PerUnit EQ10;
          OpenIPSL.Types.PerUnit EQ11;
          OpenIPSL.Types.PerUnit EQ12;
          OpenIPSL.Types.PerUnit EQ13;
          OpenIPSL.Types.PerUnit EQ14;
          OpenIPSL.Types.PerUnit EQ15;
          OpenIPSL.Types.PerUnit EQ16;
          OpenIPSL.Types.PerUnit EQ17;
          OpenIPSL.Types.PerUnit EQ18;
          OpenIPSL.Types.PerUnit EQ19;
          OpenIPSL.Types.PerUnit EQ20;
          OpenIPSL.Types.PerUnit EQ21;
          OpenIPSL.Types.PerUnit EQ22;
          OpenIPSL.Types.PerUnit EQ23;
          OpenIPSL.Types.PerUnit EQ24;
          OpenIPSL.Types.PerUnit Omegar "Rotor angular velocity";

          OpenIPSL.Types.PerUnit Ls;
          OpenIPSL.Types.PerUnit Ll;
          OpenIPSL.Types.PerUnit Lp;
          OpenIPSL.Types.PerUnit Lpp;

          OpenIPSL.Types.PerUnit Xa_c;
          OpenIPSL.Types.PerUnit Xm_c;
          OpenIPSL.Types.PerUnit X1_c;
          OpenIPSL.Types.PerUnit X2_c;

          OpenIPSL.Types.PerUnit constant1;
          OpenIPSL.Types.PerUnit constant2;
          OpenIPSL.Types.PerUnit constant3;
          OpenIPSL.Types.PerUnit constant4;
          OpenIPSL.Types.PerUnit constant5;

          Modelica.Units.SI.Time Tp0;
          Modelica.Units.SI.Time Tpp0;

        equation

          // Frequency dependent circuit impedances
          Xa_c = if (Ctrl == false) then Xa else (we_fix.y/w_b)*(Xa);
          Xm_c = if (Ctrl == false) then Xm else (we_fix.y/w_b)*(Xm);
          X1_c = if (Ctrl == false) then X1 else (we_fix.y/w_b)*(X1);
          X2_c = if (Ctrl == false) then X2 else (we_fix.y/w_b)*(X2);

          // Frequency dependent parameters and constants
          Ls = Xa_c + Xm_c;
          Ll = Xa_c;
          Lp = Xa_c + X1_c*Xm_c/(X1_c + Xm_c);
          Lpp = if (Mtype == 1 and R2 == 0 and X2 == 0) then Lp elseif (Mtype == 1) then Xa_c + X1_c*Xm_c*X2_c/(X1_c*X2_c + X1_c*Xm_c + X2_c*Xm_c) elseif (Mtype == 2 and R2 == 0 and X2 == 0) then Lp else Xa_c + (Xm_c*(X1_c+X2_c)/(X1_c + X2_c + Xm_c));
          Tp0 = if (Mtype == 1) then (X1_c + Xm_c)/(w_b*R1) elseif (Mtype == 2 and R2 == 0 and X2 == 0) then (X1_c + Xm_c)/(w_b*R1) else (X1_c + X2_c +Xm_c)/(w_b*R2);
          Tpp0 = if (Mtype == 1 and R2 == 0 and X2 == 0) then 1e-7 elseif (Mtype == 1) then (X2_c + (X1_c*Xm_c/(X1_c + Xm_c)))/(w_b*R2) elseif (Mtype == 2 and R2 == 0 and X2 == 0) then 1e-7 else (1/((1/(X1_c+Xm_c) + 1/X2_c)))/(w_b*R1);
          constant5 = Ls - Lp;
          constant3 = Lp - Ll;
          constant4 = (Lp - Lpp)/((Lp - Ll)^2);
          constant2 = (Lp - Lpp)/(Lp - Ll);
          constant1 = (Lpp - Ll)/(Lp - Ll);

          // Steady-State circuit set of algebraic-differential equations
          Eppr = EQ1 + EQ2;
          EQ1 = Epr*constant1;
          EQ2 = Ekr*constant2;
          EQ3 = Tpp0*der(Ekr);
          EQ3 = EQ4 + EQ5;
          EQ4 = (Tpp0*w_b*s)*Eki;
          EQ5 = Epr - Ekr - EQ6;
          EQ6 = Ii*constant3;
          EQ7 = EQ5*constant4;
          EQ8 = EQ7 + Ii;
          EQ9 = EQ8*constant5;
          EQ10 = Eppi*EQC;
          EQ11 = Epi*(Tp0*w_b*s);
          EQ12 = EQ10 + EQ11 - Epr - EQ9;
          EQ12 = Tp0*der(Epr);
          EQC = NUM/(Epp + eps);
          NUM = SE(Epp,SE1,SE2,1,1.2);
          Epp = sqrt(Eppr^2 + Eppi^2);
          EQ13 = EQC*Eppr;
          EQ14 = Epr*(Tp0*w_b*s);
          EQ22 = Ir - EQ21;
          EQ15 = EQ22*constant5;
          EQ16 = EQ15 - EQ14 - EQ13 - Epi;
          EQ16 = Tp0*der(Epi);
          EQ17 = Ir*constant3;
          EQ18 = EQ17 + Epi - Eki;
          EQ21 = EQ18*constant4;
          EQ19 = Ekr*(Tpp0*w_b*s);
          EQ20 = EQ18 - EQ19;
          EQ20 = Tpp0*der(Eki);
          EQ24 = Epi*constant1;
          EQ23 = Eki*constant2;
          Eppi = EQ23 + EQ24;

          //The link between voltages, currents and state variables is
          Vr = Eppr + Ra*Ir - Lpp*Ii;
          Vi = Eppi + Ra*Ii + Lpp*Ir;

          // Mechanical Equation
          s = (1 - Omegar);
          der(s) = (Tmech_pu_motor - Te_motor)/(2*H);

          //Electromagnetic torque equation in system and machine base
          Te_sys = Te_motor*CoB;
          Te_motor = Eppr*Ir + Eppi*Ii;

            annotation (Dialog(group="Machine parameters"));
        end CIM5_CIM6;
      end PSSE;

      package PSAT
        model MotorTypeI "Multi-Domain Type I Three-Phase Induction Motor Model."
          extends BaseClasses.BaseMultiDomainMotor;

          // Parameter Set
          parameter OpenIPSL.Types.PerUnit Xs=0.0759 "Stator reactance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit R1=0.0085 "1st cage rotor resistance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit X1=0.0759 "1st cage rotor reactance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit Xm=3.1241 "Magnetizing reactance" annotation (Dialog(group="Machine parameters"));

          // Variable Set
          OpenIPSL.Types.PerUnit Re;
          OpenIPSL.Types.PerUnit Xe;
          OpenIPSL.Types.PerUnit Xmag;
          OpenIPSL.Types.PerUnit Omegar "Rotor angular velocity";
          OpenIPSL.Types.PerUnit Pmotor;
        equation

          //Real and Imaginary currents drawn by the motor in the motor base
          Ii = (-p.vr/Xmag) + (p.vi*Re - p.vr*Xe)/(Re*Re + Xe*Xe);
          Ir =   p.vi/Xmag  + (p.vr*Re + p.vi*Xe)/(Re*Re + Xe*Xe);

          //Active Power Calculation based on current calculation
          Pmotor = (if Ctrl == false then P_motor else P_motor/(we_fix.y/w_b));

          //Rotor impedance
          Re = R1/s;
          Xe = (if Ctrl == false then Xs + X1 else (we_fix.y/w_b)*(Xs + X1));

          //Magnetization Impedance
          Xmag = (if Ctrl == false then Xm else (we_fix.y/w_b)*Xm);

          //Mechanical Swing Equation
          s = (1 - Omegar);
          der(s) = (Tmech_pu_motor - Pmotor)/(2*H);

        end MotorTypeI;

        model MotorTypeIII "Multi-Domain Type III Three-Phase Induction Motor Model."
          extends BaseClasses.BaseMultiDomainMotor;

          parameter OpenIPSL.Types.PerUnit Rs=0 "Stator resistance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit Xs=0.0759 "Stator reactance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit R1=0.0085 "1st cage rotor resistance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit X1=0.0759 "1st cage rotor reactance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit Xm=3.1241 "Magnetizing reactance" annotation (Dialog(group="Machine parameters"));

          OpenIPSL.Types.PerUnit epr;
          OpenIPSL.Types.PerUnit epm;
          OpenIPSL.Types.PerUnit X0;
          OpenIPSL.Types.PerUnit Xp;
          OpenIPSL.Types.Time Tp0;

          OpenIPSL.Types.PerUnit Te_motor;
          OpenIPSL.Types.PerUnit Te_sys;


        equation

          //Rotor impedance based on controllable model
          X0  = (if Ctrl == false then Xs + Xm else (we_fix.y/w_b)*(Xs + Xm));
          Xp  = (if Ctrl == false then Xs + X1*Xm/(X1 + Xm) else (we_fix.y/w_b)*(Xs + X1*Xm/(X1 + Xm)));
          Tp0 = (if Ctrl == false then (X1 + Xm)/(w_b*R1) else (we_fix.y/w_b)*((X1 + Xm)/(w_b*R1)));


          //The real and imaginary current relationship
          Vr = epr + Rs*Ir - Xp*Ii;
          Vi = epm + Rs*Ii + Xp*Ir;

          //Electromagnetic differential equations
          der(epr) =   w_sync*s*epm  - (epr + (X0 - Xp)*Ii)/Tp0;
          der(epm) = (-w_sync*s*epr) - (epm - (X0 - Xp)*Ir)/Tp0;

          //Mechanical Slip Equation
          der(s) = (Tmech_pu_motor - Te_motor)/(2*H);

          //Electromagnetic torque equation in system and machine base
          Te_sys = Te_motor*CoB;
          Te_motor = (if Ctrl == false then (epr*Ir + epm*Ii) else (epr*Ir + epm*Ii)/(we_fix.y/w_b));


        end MotorTypeIII;

        model MotorTypeV "Multi-Domain Type V Three-Phase Induction Motor Model."
          extends BaseClasses.BaseMultiDomainMotor;

          parameter OpenIPSL.Types.PerUnit Rs=0 "Stator resistance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit Xs=0.0759 "Stator reactance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit R1=0.0085 "1st cage rotor resistance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit X1=0.0759 "1st cage rotor reactance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit R2=0 "1st cage rotor resistance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit X2=0 "1st cage rotor reactance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit Xm=3.1241 "Magnetizing reactance" annotation (Dialog(group="Machine parameters"));

          OpenIPSL.Types.PerUnit epr;
          OpenIPSL.Types.PerUnit epm;
          OpenIPSL.Types.PerUnit eppr;
          OpenIPSL.Types.PerUnit eppm;
          OpenIPSL.Types.PerUnit X0;
          OpenIPSL.Types.PerUnit Xp;
          OpenIPSL.Types.PerUnit Xpp;
          OpenIPSL.Types.PerUnit Tp0;
          OpenIPSL.Types.PerUnit Tpp0;

          OpenIPSL.Types.PerUnit Te_motor;
          OpenIPSL.Types.PerUnit Te_sys;

        equation

          //Rotor impedance based on controllable model
          X0   = (if (Ctrl == false) then Xs + Xm else (we_fix.y/w_b)*(Xs + Xm));
          Xp   = (if (Ctrl == false) then Xs + X1*Xm/(X1 + Xm) else (we_fix.y/w_b)*(Xs + X1*Xm/(X1 + Xm)));
          Xpp  = (if (Ctrl == false and R2 ==0 and X2 == 0) then Xp elseif (Ctrl == false) then (Xs + X1*X2*Xm/(X1*X2 + X1*Xm + X2*Xm)) elseif (Ctrl == true and R2 ==0 and X2 == 0) then Xp else (we_fix.y/w_b)*(Xs + X1*X2*Xm/(X1*X2 + X1*Xm + X2*Xm)));
          Tp0  = (if (Ctrl == false) then (X1 + Xm)/(w_b*R1) else (we_fix.y/w_b)*((X1 + Xm)/(w_b*R1)));
          Tpp0 = (if (Ctrl == false and R2 ==0 and X2 == 0) then Modelica.Constants.inf elseif (Ctrl == false) then (X2 + X1*Xm/(X1+Xm))/(w_b*R2) elseif  (Ctrl == true and R2 ==0 and X2 == 0) then Modelica.Constants.inf else (we_fix.y/w_b)*(X2 + X1*Xm/(X1+Xm))/(w_b*R2));

          //The link between voltages, currents and state variables is
          Vr = eppr + Rs*Ir - Xpp*Ii;
          Vi = eppm + Rs*Ii + Xpp*Ir;

          //Electromagnetic differential equations
          der(epr)  =   w_sync*s*epm  - (epr + (X0 - Xp)*Ii)/Tp0;
          der(epm)  = (-w_sync*s*epr) - (epm - (X0 - Xp)*Ir)/Tp0;
          der(eppr) = (-w_sync*s*(epm -eppm)) + der(epr) - (epr - eppm - (Xp - Xpp)*Ii)/Tpp0;
          der(eppm) =   w_sync*s*(epr - eppr) + der(epm) - (epm - eppr + (Xp - Xpp)*Ir)/Tpp0;

          //Mechanical Slip Equation
          der(s) = (Tmech_pu_motor - Te_motor)/(2*H);

          //Electromagnetic torque equation in system and machine base
          Te_sys = Te_motor*CoB;
          Te_motor = eppr*Ir + eppm*Ii;

        end MotorTypeV;

        model MotorTypeV_different
          extends BaseClasses.BaseMultiDomainMotor2;

          parameter OpenIPSL.Types.PerUnit Rs=0 "Stator resistance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit Xs=0.0759 "Stator reactance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit R1=0.0085 "1st cage rotor resistance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit X1=0.0759 "1st cage rotor reactance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit R2=0 "1st cage rotor resistance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit X2=0 "1st cage rotor reactance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit Xm=3.1241 "Magnetizing reactance" annotation (Dialog(group="Machine parameters"));

          OpenIPSL.Types.PerUnit epr;
          OpenIPSL.Types.PerUnit epm;
          OpenIPSL.Types.PerUnit eppr;
          OpenIPSL.Types.PerUnit eppm;
          OpenIPSL.Types.PerUnit X0;
          OpenIPSL.Types.PerUnit Xp;
          OpenIPSL.Types.PerUnit Xpp;
          OpenIPSL.Types.PerUnit Tp0;
          OpenIPSL.Types.PerUnit Tpp0;

          OpenIPSL.Types.PerUnit Te_motor;
          OpenIPSL.Types.PerUnit Te_sys;

        equation

          //Rotor impedance based on controllable model
          X0   = (if (Ctrl == false) then Xs + Xm else Xs + Xm);
          Xp   = (if (Ctrl == false) then Xs + X1*Xm/(X1 + Xm) else (we_fix.y/w_b)*(Xs + (we_fix.y/w_b)*X1*Xm/((we_fix.y/w_b)*X1 + Xm)));
          Xpp  = (if (Ctrl == false and R2 ==0 and X2 == 0) then Xp elseif (Ctrl == false) then (Xs + X1*X2*Xm/(X1*X2 + X1*Xm + X2*Xm)) elseif (Ctrl == true and R2 ==0 and X2 == 0) then Xp else (we_fix.y/w_b)*(Xs + (we_fix.y/w_b)^2*X1*X2*Xm/((we_fix.y/w_b)^2*X1*X2 + (we_fix.y/w_b)*X1*Xm + (we_fix.y/w_b)*X2*Xm)));
          Tp0  = (if (Ctrl == false) then (X1 + Xm)/(w_b*R1) else (((we_fix.y/w_b)*X1 + Xm)/(w_b*R1)));
          Tpp0 = (if (Ctrl == false and R2 ==0 and X2 == 0) then Modelica.Constants.inf elseif (Ctrl == false) then (X2 + X1*Xm/(X1+Xm))/(w_b*R2) elseif  (Ctrl == true and R2 ==0 and X2 == 0) then Modelica.Constants.inf else ((we_fix.y/w_b)*X2 + (we_fix.y/w_b)*X1*Xm/((we_fix.y/w_b)*X1+Xm))/(w_b*R2));

          //The link between voltages, currents and state variables is
          Vr = eppr + Rs*Ir - Xpp*Ii;
          Vi = eppm + Rs*Ii + Xpp*Ir;

          //Electromagnetic differential equations
          der(epr)  =   w_sync*s*epm  - (epr + (X0 - Xp)*Ii)/Tp0;
          der(epm)  = (-w_sync*s*epr) - (epm - (X0 - Xp)*Ir)/Tp0;
          der(eppr) = (-w_sync*s*(epm -eppm)) + der(epr) - (epr - eppm - (Xp - Xpp)*Ii)/Tpp0;
          der(eppm) =   w_sync*s*(epr - eppr) + der(epm) - (epm - eppr + (Xp - Xpp)*Ir)/Tpp0;

          //Mechanical Slip Equation
          der(s) = (Tmech_pu_motor - Te_motor)/(2*H);

          //Electromagnetic torque equation in system and machine base
          Te_sys = Te_motor*CoB;
          Te_motor = eppr*Ir + eppm*Ii;

        end MotorTypeV_different;

        model MotorTypeIII_different
          extends BaseClasses.BaseMultiDomainMotor2;

          parameter OpenIPSL.Types.PerUnit Rs=0 "Stator resistance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit Xs=0.0759 "Stator reactance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit R1=0.0085 "1st cage rotor resistance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit X1=0.0759 "1st cage rotor reactance" annotation (Dialog(group="Machine parameters"));
          parameter OpenIPSL.Types.PerUnit Xm=3.1241 "Magnetizing reactance" annotation (Dialog(group="Machine parameters"));

          OpenIPSL.Types.PerUnit epr;
          OpenIPSL.Types.PerUnit epm;
          OpenIPSL.Types.PerUnit X0;
          OpenIPSL.Types.PerUnit Xp;
          OpenIPSL.Types.Time Tp0;

          OpenIPSL.Types.PerUnit Te_motor;
          OpenIPSL.Types.PerUnit Te_sys;

        initial equation

          //der(epr) = 0;
          //der(epm) = 0;

        equation

          //Rotor impedance based on controllable model
          X0  = (if Ctrl == false then Xs + Xm else (we_fix.y/w_b)*(Xs + Xm));
          Xp  = (if Ctrl == false then Xs + X1*Xm/(X1 + Xm) else (we_fix.y/w_b)*(Xs + X1*Xm/(X1 + Xm)));
          Tp0 = (if Ctrl == false then (X1 + Xm)/(w_b*R1) else (we_fix.y/w_b)*((X1 + Xm)/(w_sync*R1)));


          //The real and imaginary current relationship
          Vr = epr + Rs*Ir - Xp*Ii;
          Vi = epm + Rs*Ii + Xp*Ir;

          //Electromagnetic differential equations
          der(epr) =   w_sync*s*epm  - (epr + (X0 - Xp)*Ii)/Tp0;
          der(epm) = (-w_sync*s*epr) - (epm - (X0 - Xp)*Ir)/Tp0;

          //Mechanical Slip Equation
          der(s) = (Tmech_pu_motor - Te_motor)/(2*H);

          //Electromagnetic torque equation in system and machine base
          Te_sys = Te_motor*CoB;
          Te_motor = (if Ctrl == false then (epr*Ir + epm*Ii) else (epr*Ir + epm*Ii)/(we_fix.y/w_b));

        end MotorTypeIII_different;
      end PSAT;

      package BaseClasses
        extends Modelica.Icons.BasesPackage;
        partial model BaseMultiDomainMotor

          parameter OpenIPSL.Types.ApparentPower M_b = 15e6 "Machine base power"
                                                                                annotation (Dialog(group="Power flow data"));
          extends OpenIPSL.Electrical.Essentials.pfComponent(
            final enabledisplayPF=false,
            final enablefn=false,
            final enableV_b=false,
            final enableangle_0=false,
            final enablev_0=false,
            final enableQ_0=false,
            final enableP_0=false,
            final enableS_b=false);

            import Modelica.Constants.pi;
            import OpenIPSL.NonElectrical.Functions.SE;
            import Modelica.Constants.eps;

          parameter Boolean Sup = true "True: Start-up process, False: Steady-state condition" annotation (Dialog(group="Motor Setup"));
          parameter Boolean Ctrl = true "True: Model for VSD control, False: Model not controllable"
                                                                                                    annotation (Dialog(group="Motor Setup"));
          parameter Real N = 1 "Number of pair of Poles"
                                                        annotation (Dialog(group="Machine parameters"));
          parameter Modelica.Units.SI.Time H = 0.4 "Inertia constant"
                                                                     annotation (Dialog(group="Machine parameters"));

          OpenIPSL.Types.PerUnit v "Bus voltage magnitude";
          OpenIPSL.Types.Angle anglev " Bus voltage angle";
          OpenIPSL.Types.Angle delta " Bus voltage angle";
          OpenIPSL.Types.PerUnit s(start = s0);
          OpenIPSL.Types.PerUnit P;
          OpenIPSL.Types.PerUnit Q;
          Modelica.Units.SI.AngularVelocity nr;
          Modelica.Units.SI.AngularVelocity ns;
          Modelica.Units.SI.AngularVelocity w_sync;
          Modelica.Units.SI.Torque T_b;
          OpenIPSL.Types.PerUnit Tmech_pu_sys;
          OpenIPSL.Types.PerUnit Tmech_pu_motor;
          OpenIPSL.Types.PerUnit P_motor "OK";
          OpenIPSL.Types.PerUnit Q_motor "OK";
          OpenIPSL.Types.PerUnit Vr "OK";
          OpenIPSL.Types.PerUnit Vi "OK";
          OpenIPSL.Types.PerUnit Ir "OK";
          OpenIPSL.Types.PerUnit Ii "OK";
          OpenIPSL.Types.PerUnit Imag;

          Modelica.Blocks.Sources.RealExpression Rotor_Speed(y=nr)
            annotation (Placement(transformation(extent={{70,-10},{50,10}})));
          Modelica.Mechanics.Rotational.Components.Inertia Rotor_Inertia(J=J_load, w(fixed=
                  true, start=w0))
            annotation (Placement(transformation(extent={{-40,-10},{-60,10}})));
          Modelica.Blocks.Math.Gain we_fix(k=1)
            annotation (Placement(transformation(extent={{70,-90},{80,-80}})));
          Modelica.Blocks.Sources.Constant we_source_fix(k=0)
                                                         if not Ctrl
            annotation (Placement(transformation(extent={{32,-86},{44,-74}})));
          OpenIPSL.Interfaces.PwPin p(
            vr(start=vr0),
            vi(start=vi0),
            ir(start=ir0_sys),
            ii(start=ii0_sys))
            annotation (Placement(transformation(extent={{90,-10},{110,10}})));
          Modelica.Mechanics.Rotational.Interfaces.Flange_b flange annotation (
              Placement(transformation(extent={{-110,-10},{-90,10}}),
                iconTransformation(extent={{-110,-10},{-90,10}})));
          Modelica.Blocks.Interfaces.RealOutput wr
            "Absolute angular velocity of flange as output signal"
            annotation (Placement(transformation(extent={{-100,-50},{-120,-30}}),
                iconTransformation(
                extent={{20,-20},{-20,20}},
                rotation=90,
                origin={60,-120})));
          Modelica.Blocks.Interfaces.RealInput we if Ctrl annotation (Placement(
                transformation(
                extent={{-20,-20},{20,20}},
                rotation=90,
                origin={60,-120}), iconTransformation(
                extent={{-20,-20},{20,20}},
                rotation=90,
                origin={0,-120})));
          Modelica.Blocks.Interfaces.RealInput mech_torque annotation (Placement(
                transformation(
                extent={{-20,-20},{20,20}},
                rotation=90,
                origin={-60,-120}), iconTransformation(
                extent={{-20,-20},{20,20}},
                rotation=90,
                origin={-60,-120})));
        protected
          Modelica.Mechanics.Rotational.Sources.Speed speed(exact=true)
                                                            annotation (Placement(
                transformation(
                extent={{-10,10},{10,-10}},
                rotation=180)));

        protected
          parameter Modelica.Units.SI.Inertia J_load = 2*H*M_b/((2*pi*fn/N)^2);
          parameter OpenIPSL.Types.PerUnit p0 = P_0/M_b;
          parameter OpenIPSL.Types.PerUnit q0 = Q_0/M_b;
          parameter Modelica.Units.SI.AngularVelocity w_b=2*pi*fn/N "Base freq in rad/s";
          parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0);
          parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0);
          parameter OpenIPSL.Types.PerUnit ir0=(p0*vr0 + q0*vi0)/(vr0^2 + vi0^2);
          parameter OpenIPSL.Types.PerUnit ii0=(p0*vi0 - q0*vr0)/(vr0^2 + vi0^2);
          parameter OpenIPSL.Types.PerUnit ir0_sys = CoB*ir0 "Initial real current in system base";
          parameter OpenIPSL.Types.PerUnit ii0_sys = CoB*ii0 "Initial imaginary current in system base";
          parameter Real CoB = M_b/S_b;
          parameter Real s0 = 1 - eps;
          parameter Real w0 = if Sup == true then eps else 2*Modelica.Constants.pi*SysData.fn;


        equation
          connect(Rotor_Inertia.flange_b,flange)
            annotation (Line(points={{-60,0},{-100,0}}, color={0,0,0}));
          connect(speed.w_ref,Rotor_Speed. y)
            annotation (Line(points={{12,0},{49,0}},           color={0,0,127}));
          connect(speed.flange,Rotor_Inertia. flange_a) annotation (Line(points={{-10,0},
                  {-40,0}},              color={0,0,0}));
          connect(wr,Rotor_Speed. y) annotation (Line(points={{-110,-40},{40,-40},{40,
                  0},{49,0}},
                           color={0,0,127}));
          connect(we,we_fix. u)
            annotation (Line(points={{60,-120},{60,-85},{69,-85}}, color={0,0,127}));
          connect(we_source_fix.y,we_fix. u)
            annotation (Line(points={{44.6,-80},{69,-80},{69,-85}}, color={0,0,127}));

          //Synchronous speed based on controllable boolean parameter
          w_sync = (if Ctrl == true then we_fix.y else w_b);


          // Network Interface Equations
          anglev = atan2(p.vi, p.vr) "OK";
          delta = anglev "OK";
          v = sqrt(p.vr^2 + p.vi^2) "OK";
          Vr = p.vr "OK";
          Vi = p.vi "OK";
          [Ir; Ii] = (1/CoB)*[p.ir; p.ii] "OK";
          Imag = sqrt(Ir^2 + Ii^2);

          // MISSING VOLTAGE EQUATIONS

          P = p.vr*p.ir + p.vi*p.ii;
          Q = (-p.vr*p.ii) + p.vi*p.ir;

          //Active and Reactive Power consumption in the machine base
          P_motor = P/CoB;
          Q_motor = Q/CoB;

          //Rotor speed equation
          ns = w_sync/N;
          nr = (1-s)*ns;

          //Conversion from SI torqur to p.u. torque
          Tmech_pu_sys = mech_torque/T_b;
          Tmech_pu_motor = Tmech_pu_sys/CoB;

          //Torque System base
          T_b = S_b/w_b;

          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                  Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200}),
                                                  Ellipse(
                  fillColor={255,255,255},
                  extent={{-56,-58},{55.932,54}}), Text(
                  extent={{-50,48},{50,-52}},
                  lineColor={0,0,0},
                  textString="M"),Text(
                  origin={0,-80},
                  extent={{-100,-20},{100,20}},
                  fontName="Arial",
                  lineColor={0,0,0},
                  textString="%name"),
                Text(
                  extent={{-90,100},{-30,60}},
                  textColor={0,140,72},
                  textString="MD")}),    Diagram(coordinateSystem(preserveAspectRatio=false)));
        end BaseMultiDomainMotor;

        partial model BaseMultiDomainMotor_forpaper

          parameter OpenIPSL.Types.ApparentPower M_b = 15e6 "Machine base power"
                                                                                annotation (Dialog(group="Power flow data"));
          extends OpenIPSL.Electrical.Essentials.pfComponent(
            final enabledisplayPF=false,
            final enablefn=false,
            final enableV_b=false,
            final enableangle_0=false,
            final enablev_0=false,
            final enableQ_0=false,
            final enableP_0=false,
            final enableS_b=true);

            import Modelica.Constants.pi;
            import OpenIPSL.NonElectrical.Functions.SE;
            import Modelica.Constants.eps;

          parameter Boolean Sup = true "True: Start-up process, False: Steady-state condition" annotation (Dialog(group="Motor Setup"));
          parameter Boolean Ctrl = true "True: Model for VSD control, False: Model not controllable"
                                                                                                    annotation (Dialog(group="Motor Setup"));
          parameter Real N = 1 "Number of pair of Poles"
                                                        annotation (Dialog(group="Machine parameters"));
          parameter Modelica.Units.SI.Time H = 0.4 "Inertia constant"
                                                                     annotation (Dialog(group="Machine parameters"));

          OpenIPSL.Types.PerUnit v "Bus voltage magnitude";
          OpenIPSL.Types.Angle anglev " Bus voltage angle";
          OpenIPSL.Types.Angle delta " Bus voltage angle";
          OpenIPSL.Types.PerUnit s(start = s0);
          OpenIPSL.Types.PerUnit P;
          OpenIPSL.Types.PerUnit Q;
          Modelica.Units.SI.AngularVelocity nr;
          Modelica.Units.SI.AngularVelocity ns;
          Modelica.Units.SI.AngularVelocity w_sync;
          Modelica.Units.SI.Torque T_b;
          OpenIPSL.Types.PerUnit Tmech_pu_sys;
          OpenIPSL.Types.PerUnit Tmech_pu_motor;
          OpenIPSL.Types.PerUnit P_motor "OK";
          OpenIPSL.Types.PerUnit Q_motor "OK";
          OpenIPSL.Types.PerUnit Vr "OK";
          OpenIPSL.Types.PerUnit Vi "OK";
          OpenIPSL.Types.PerUnit Ir "OK";
          OpenIPSL.Types.PerUnit Ii "OK";
          OpenIPSL.Types.PerUnit Imag;

          Modelica.Blocks.Sources.RealExpression Rotor_Speed(y=nr)
            annotation (Placement(transformation(extent={{70,-10},{50,10}})));
          Modelica.Mechanics.Rotational.Components.Inertia Rotor_Inertia(J=J_load)
            annotation (Placement(transformation(extent={{-40,-10},{-60,10}})));
          Modelica.Blocks.Math.Gain we_fix(k=1)
            annotation (Placement(transformation(extent={{70,-90},{80,-80}})));
          Modelica.Blocks.Sources.Constant we_source_fix(k=0)
                                                         if not Ctrl
            annotation (Placement(transformation(extent={{32,-86},{44,-74}})));
          OpenIPSL.Interfaces.PwPin p(
            vr(start=vr0),
            vi(start=vi0),
            ir(start=ir0_sys),
            ii(start=ii0_sys))
            annotation (Placement(transformation(extent={{90,-10},{110,10}})));
          Modelica.Mechanics.Rotational.Interfaces.Flange_b flange annotation (
              Placement(transformation(extent={{-110,-10},{-90,10}}),
                iconTransformation(extent={{-110,-10},{-90,10}})));
          Modelica.Blocks.Interfaces.RealOutput wr
            "Absolute angular velocity of flange as output signal"
            annotation (Placement(transformation(extent={{-100,-50},{-120,-30}}),
                iconTransformation(
                extent={{20,-20},{-20,20}},
                rotation=90,
                origin={60,-120})));
          Modelica.Blocks.Interfaces.RealInput we if Ctrl annotation (Placement(
                transformation(
                extent={{-20,-20},{20,20}},
                rotation=90,
                origin={60,-120}), iconTransformation(
                extent={{-20,-20},{20,20}},
                rotation=90,
                origin={0,-120})));
          Modelica.Blocks.Interfaces.RealInput mech_torque annotation (Placement(
                transformation(
                extent={{-20,-20},{20,20}},
                rotation=90,
                origin={-60,-120}), iconTransformation(
                extent={{-20,-20},{20,20}},
                rotation=90,
                origin={-60,-120})));
        protected
          Modelica.Mechanics.Rotational.Sources.Speed speed(exact=true)
                                                            annotation (Placement(
                transformation(
                extent={{-10,10},{10,-10}},
                rotation=180)));

        protected
          parameter Modelica.Units.SI.Inertia J_load = 2*H*M_b/((2*pi*fn/N)^2);
          parameter OpenIPSL.Types.PerUnit p0 = P_0/M_b;
          parameter OpenIPSL.Types.PerUnit q0 = Q_0/M_b;
          parameter Modelica.Units.SI.AngularVelocity w_b=2*pi*fn "Base freq in rad/s";
          parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0);
          parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0);
          parameter OpenIPSL.Types.PerUnit ir0=(p0*vr0 + q0*vi0)/(vr0^2 + vi0^2);
          parameter OpenIPSL.Types.PerUnit ii0=(p0*vi0 - q0*vr0)/(vr0^2 + vi0^2);
          parameter OpenIPSL.Types.PerUnit ir0_sys = CoB*ir0 "Initial real current in system base";
          parameter OpenIPSL.Types.PerUnit ii0_sys = CoB*ii0 "Initial imaginary current in system base";
          parameter Real CoB = M_b/S_b;
          parameter Real s0 = 1 - eps;
        equation
          connect(Rotor_Inertia.flange_b,flange)
            annotation (Line(points={{-60,0},{-100,0}}, color={0,0,0}));
          connect(speed.w_ref,Rotor_Speed. y)
            annotation (Line(points={{12,0},{49,0}},           color={0,0,127}));
          connect(speed.flange,Rotor_Inertia. flange_a) annotation (Line(points={{-10,0},
                  {-40,0}},              color={0,0,0}));
          connect(wr,Rotor_Speed. y) annotation (Line(points={{-110,-40},{40,-40},{40,
                  0},{49,0}},
                           color={0,0,127}));
          connect(we,we_fix. u)
            annotation (Line(points={{60,-120},{60,-85},{69,-85}}, color={0,0,127}));
          connect(we_source_fix.y,we_fix. u)
            annotation (Line(points={{44.6,-80},{69,-80},{69,-85}}, color={0,0,127}));

            //Synchronous speed based on controllable boolean parameter
          w_sync = (if Ctrl == true then we_fix.y else w_b);

          //Torque System base
          T_b = S_b/w_b;

          // Network Interface Equations
          anglev = atan2(p.vi, p.vr) "OK";
          delta = anglev "OK";
          v = sqrt(p.vr^2 + p.vi^2) "OK";
          Vr = p.vr "OK";
          Vi = p.vi "OK";
          [Ir; Ii] = (1/CoB)*[p.ir; p.ii] "OK";
          Imag = sqrt(Ir^2 + Ii^2);

          // MISSING VOLTAGE EQUATIONS

          P = p.vr*p.ir + p.vi*p.ii;
          Q = (-p.vr*p.ii) + p.vi*p.ir;

          //Active and Reactive Power consumption in the machine base
          P_motor = P/CoB;
          Q_motor = Q/CoB;

          //Rotor speed equation
          ns = w_sync/N;
          nr = (1-s)*ns;

          //Conversion from SI torqur to p.u. torque
          Tmech_pu_sys = mech_torque/T_b;
          Tmech_pu_motor = Tmech_pu_sys/CoB;

          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                  Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200}),
                                                  Ellipse(
                  fillColor={255,255,255},
                  extent={{-56,-58},{55.932,54}}), Text(
                  extent={{-50,44},{50,-56}},
                  lineColor={0,0,0},
                  textString="M"),Text(
                  origin={0,-80},
                  extent={{-80,-20},{80,20}},
                  fontName="Arial",
                  lineColor={0,0,0},
                  textString="BaseMultiDomainMotor"),
                Text(
                  extent={{-90,90},{-30,50}},
                  textColor={0,140,72},
                  textString="MD"),
                Text(
                  extent={{120,20},{180,-20}},
                  textColor={28,108,200},
                  textString="pwpin"),
                Text(
                  extent={{48,-140},{76,-180}},
                  textColor={28,108,200},
                  textString="wr"),
                Text(
                  extent={{-14,-140},{16,-180}},
                  textColor={28,108,200},
                  textString="we"),
                Text(
                  extent={{-172,-86},{-38,-236}},
                  textColor={28,108,200},
                  textString="mech_torque"),
                Text(
                  extent={{-176,20},{-116,-20}},
                  textColor={28,108,200},
                  textString="flange")}),Diagram(coordinateSystem(preserveAspectRatio=false)));
        end BaseMultiDomainMotor_forpaper;
      end BaseClasses;
    end ThreePhase;
  end Motors;
end MultiDomainModels;
