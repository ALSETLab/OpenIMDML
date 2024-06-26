within OpenIMDML.NonMultiDomain.Motors.SinglePhase;
model NMD_SPIM "This model is the steady-state circuit model of the non multi-domain single phase induction motor model."

  extends
    OpenIMDML.NonMultiDomain.Motors.SinglePhase.BaseClasses.BaseNonMultiDomainSinglePhase;

    //parameter Boolean Sup = true "True: Start-up process, False: Steady-state condition" annotation (Dialog(group="Motor Setup"));

    parameter OpenIPSL.Types.PerUnit R1 "Stator winding resistor" annotation (Dialog(group="Machine parameters"));
    parameter OpenIPSL.Types.PerUnit R2 "Rotor winding resistor" annotation (Dialog(group="Machine parameters"));
    parameter OpenIPSL.Types.PerUnit X1 "Stator winding reactance" annotation (Dialog(group="Machine parameters"));
    parameter OpenIPSL.Types.PerUnit X2 "Rotor winding reactance" annotation (Dialog(group="Machine parameters"));
    parameter OpenIPSL.Types.PerUnit Xm "Magnitization reactance value" annotation (Dialog(group="Machine parameters"));
    parameter OpenIPSL.Types.PerUnit H "Inertia coeficient" annotation (Dialog(group="Machine parameters"));
    parameter Real a annotation (Dialog(group="Machine parameters"));
    parameter Real b annotation (Dialog(group="Machine parameters"));
    parameter Real c annotation (Dialog(group="Machine parameters"));

    OpenIPSL.Types.PerUnit RF "Equivalent resistor component of the forward circuit";
    OpenIPSL.Types.PerUnit XF "Equivalent resistor component of the forward circuit";
    OpenIPSL.Types.PerUnit RB "Equivalent resistor component of the backward circuit";
    OpenIPSL.Types.PerUnit XB "Equivalent resistor component of the backward circuit";
    OpenIPSL.Types.PerUnit s(start = s0)  "Induction motor slip";

    OpenIPSL.Types.PerUnit I "Magnitude of the consumed current";
    OpenIPSL.Types.PerUnit Ir "Real component of the consumed current in Machine Base";
    OpenIPSL.Types.PerUnit Ii "Imaginary component of the consumed current in Machine Base";
    OpenIPSL.Types.PerUnit P_AGF "Air Gap Power for forward magnetic field";
    OpenIPSL.Types.PerUnit P_AGB "Air Gap Power for reverse field";
    OpenIPSL.Types.PerUnit P( start = P_0/S_b) "Net air gap power in single-phase induction motor";
    //OpenIPSL.Types.PerUnit Q "Reactive Power consumed by the single phase induction motor model";
    OpenIPSL.Types.PerUnit Te "Electrical Torque";
    OpenIPSL.Types.PerUnit Tm "Mechanical Torque of the Load";
    OpenIPSL.Types.PerUnit Pc;
    OpenIPSL.Types.PerUnit Qc;

protected
  parameter Real A = a + b + c;
  parameter Real B = -b - 2*c;
  parameter Real C = c;
  parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0);
  parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0);
  parameter OpenIPSL.Types.PerUnit ir0=(P_0/S_b*vr0 + Q_0/S_b*vi0)/(vr0^2 + vi0^2);
  parameter OpenIPSL.Types.PerUnit ii0=(P_0/S_b*vi0 - Q_0/S_b*vr0)/(vr0^2 + vi0^2);
  OpenIPSL.Types.PerUnit sm  "Induction motor slip for calculation when dividing by s";
  parameter Real CoB = M_b/S_b;
  parameter OpenIPSL.Types.PerUnit s0 = (2*ii0*R2^2*Xm)/(2*ir0*R2*Xm^2 - 2*ii0*R2^2*Xm);

initial equation
  der(s) = 0;

equation

  //Network Interface Equations
  [Ir; Ii] = (1/CoB)*[p.ir; p.ii];
  I = sqrt(Ir^2 + Ii^2);

  //Active and Reactive Power Consumption in the system base
  Pc = p.vr*p.ir + p.vi*p.ii;
  Qc = (-p.vr*p.ii) + p.vi*p.ir;

  //Slip for calculations
  sm = s + Modelica.Constants.eps;

  RF = (-X2*Xm*R2/sm + R2*Xm*(X2 + Xm)/sm)/((R2/sm)^2 + (X2 + Xm)^2);
  XF = ((R2/sm)^2*Xm + X2*Xm*(X2 + Xm))/((R2/sm)^2 + (X2 + Xm)^2);
  RB = (-X2*Xm*R2/(2-sm) + R2*Xm*(X2 + Xm)/(2-sm))/((R2/(2-sm))^2 + (X2 + Xm)^2);
  XB = ((R2/(2-sm))^2*Xm + X2*Xm*(X2 + Xm))/((R2/(2-sm))^2 + (X2 + Xm)^2);

  Ir = p.vr*(0.5*RF + 0.5*RB + R1)/((0.5*RF + 0.5*RB + R1)^2 + (0.5*XF + 0.5*XB + X1)^2) + p.vi*(0.5*XF + 0.5*XB + X1)/((0.5*RF + 0.5*RB + R1)^2 + (0.5*XF + 0.5*XB + X1)^2);
  Ii = p.vi*(0.5*RF + 0.5*RB + R1)/((0.5*RF + 0.5*RB + R1)^2 + (0.5*XF + 0.5*XB + X1)^2) - p.vr*(0.5*XF + 0.5*XB + X1)/((0.5*RF + 0.5*RB + R1)^2 + (0.5*XF + 0.5*XB + X1)^2);

  P_AGF = I^2*0.5*RF;
  P_AGB = I^2*0.5*RB;
  P     = P_AGF - P_AGB;

  Te = P;
  Tm = A + B*s + C*s^2;
  der(s) = (Tm - Te)/(2*H);
  annotation (preferredView = "info",Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end NMD_SPIM;
