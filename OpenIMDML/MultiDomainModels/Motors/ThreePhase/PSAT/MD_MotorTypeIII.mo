within OpenIMDML.MultiDomainModels.Motors.ThreePhase.PSAT;
model MD_MotorTypeIII
  "Multi-Domain Type III Three-Phase Induction Motor Model."
  extends BaseClasses.BaseMultiDomainThreePhase;

  parameter OpenIPSL.Types.PerUnit Rs=0 "Stator resistance" annotation (Dialog(group="Machine parameters"));
  parameter OpenIPSL.Types.PerUnit Xs=0.0759 "Stator reactance" annotation (Dialog(group="Machine parameters"));
  parameter OpenIPSL.Types.PerUnit R1=0.0085 "1st cage rotor resistance" annotation (Dialog(group="Machine parameters"));
  parameter OpenIPSL.Types.PerUnit X1=0.0759 "1st cage rotor reactance" annotation (Dialog(group="Machine parameters"));
  parameter OpenIPSL.Types.PerUnit Xm=3.1241 "Magnetizing reactance" annotation (Dialog(group="Machine parameters"));

  OpenIPSL.Types.PerUnit epr(start = epr0, fixed=false);
  OpenIPSL.Types.PerUnit epm(start = epm0, fixed=false);
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
  der(epr) =   w_b*s*epm  - (epr + (X0 - Xp)*Ii)/Tp0;
  der(epm) = (-w_b*s*epr) - (epm - (X0 - Xp)*Ir)/Tp0;

  //Mechanical Slip Equation
  der(s) = (Tmech_pu_motor - Te_motor)/(2*H);

  //Electromagnetic torque equation in system and machine base
  Te_sys = Te_motor*CoB;
  Te_motor = (if Ctrl == false then (epr*Ir + epm*Ii) else (epr*Ir + epm*Ii)/(we_fix.y/w_b));

  annotation(preferredView = "info");
end MD_MotorTypeIII;
