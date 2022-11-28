within OpenIMDML.MultiDomainModels.Motors.SinglePhase;
model MD_DPIM_2
  "This model is the steady-state circuit model of the single phase induction motor model initialized by either a split-phase auxiliary circuit or a capacitor-start auxiliary circuit."

extends
    OpenIMDML.MultiDomainModels.Motors.SinglePhase.BaseClasses.BaseMultiDomainSinglePhase(
      Rotor_Inertia(w(fixed=false, start=w0)));

  import Modelica.ComplexMath;
  import Complex;
  import Modelica.ComplexMath.arg;
  import Modelica.ComplexMath.real;
  import Modelica.ComplexMath.imag;
  import Modelica.ComplexMath.conj;
  import Modelica.ComplexMath.fromPolar;
  import Modelica.ComplexMath.j;

  parameter Boolean Sup = true "True: Start-up process, False: Steady-state condition" annotation (Dialog(group="Motor Setup"));

  parameter Integer init "Initialization Method: (1) Split-Phase Motor, (2) Capacitor-Start Motor" annotation (Dialog(group="Motor Setup"), choices(choice=1, choice=2));
  parameter Real switch_open_speed = 0.2 "Auxiliary winding cut-off speed" annotation (Dialog(group="Machine parameters"));
  parameter Modelica.Units.SI.Inductance Lmainr "Mutual-inductance of the main winding" annotation (Dialog(group="Machine parameters"));
  parameter Modelica.Units.SI.Inductance Lmain "Self-inductance of the magnetizing branch" annotation (Dialog(group="Machine parameters"));
  parameter Modelica.Units.SI.Inductance Lauxr "Mutual-inductance of the auxiliary winding" annotation (Dialog(group="Machine parameters"));
  parameter Modelica.Units.SI.Inductance Laux "Self-inductance of the auxiliary winding" annotation (Dialog(group="Machine parameters"));
  parameter Modelica.Units.SI.Inductance Lr "Self-inductance of the equivalent rotor windings" annotation (Dialog(group="Machine parameters"));
  parameter Modelica.Units.SI.Resistance Rmain "Resistance of the main winding" annotation (Dialog(group="Machine parameters"));
  parameter Modelica.Units.SI.Resistance Rr "Resistance of the rotor winding" annotation (Dialog(group="Machine parameters"));
  parameter Modelica.Units.SI.Resistance Raux "Resistance of the auxiliary winding" annotation (Dialog(group="Machine parameters"));
  parameter Modelica.Units.SI.Capacitance Cc "Capacitance of the capacitor-start configuration" annotation (Dialog(group="Machine parameters", enable=(init == 2)));

  Modelica.Units.SI.Reactance Xmain;
  Modelica.Units.SI.Reactance Xaux;
  Modelica.Units.SI.Reactance Xc;
  OpenIPSL.Types.PerUnit P;
  OpenIPSL.Types.PerUnit Pmotorbase;
  OpenIPSL.Types.PerUnit Q;
  OpenIPSL.Types.PerUnit s;
  OpenIPSL.Types.PerUnit Te "Total Electrical Torque";
  Complex Te_pre;
  Modelica.Units.SI.ActivePower Pc;
  Modelica.Units.SI.ReactivePower Qc;

    //Modelica.SIunits.Torque Tele;

  Complex Kplus;
  Complex Kminus;
  Complex Vmain;
  Complex Vaux;
  Complex Lambda_main;
  Complex Lambda_aux;
  Complex Imain;
  Complex Iaux;
  Real KplusK_real;
  Real KplusK_imag;
  Real KminusK_real;
  Real KminusK_imag;

protected
  parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0);
  parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0);
  parameter OpenIPSL.Types.PerUnit ir0=(P_0/S_b*vr0 + Q_0/S_b*vi0)/(vr0^2 + vi0^2);
  parameter OpenIPSL.Types.PerUnit ii0=(P_0/S_b*vi0 - Q_0/S_b*vr0)/(vr0^2 + vi0^2);
  parameter Modelica.Units.SI.AngularVelocity w0 = if Sup == true then Modelica.Constants.eps else w_b;
  Modelica.Units.SI.Impedance Z_b=V_b^2/S_b;
  Modelica.Units.SI.Current I_b=S_b/V_b;
  OpenIPSL.Types.PerUnit sm  "Induction motor slip for calculation when dividing by s";

initial equation
  if Sup == true then
    s = 1- Modelica.Constants.eps;
    else
  der(s) = 0;
  end if;

equation

  //Reactance of the Capacitor
  Xc = 1/(w_b*Cc);

  //SI Reactance of the Main Winding of the Dual Phase Induction Motor
  Xmain = w_b*Lmain;
  Xaux = w_b*Laux;

  //Slip for calculations
  sm = s;

  // Calculation of the coeficients of the two-phase motor

  Kplus = (sm*w_b)/(2*(Rr+ j*sm*w_b*Lr));
  Kminus = ((2-sm)*w_b)/(2*(Rr+ j*(2-sm)*w_b*Lr));

  KplusK_real = real(Kplus + Kminus);
  KplusK_imag = imag(Kplus + Kminus);
  KminusK_real = real(Kplus - Kminus);
  KminusK_imag = imag(Kplus - Kminus);

  Lambda_main = if sm > switch_open_speed then (Lmain - j*Lmainr^2*(Kplus + Kminus))*Imain + Lmainr*Lauxr*(Kplus - Kminus)*Iaux else (Lmain - j*Lmainr^2*(Kplus + Kminus))*Imain;
  Lambda_aux = if sm > switch_open_speed  then -Lmainr*Lauxr*(Kplus - Kminus)*Imain + (Laux - j*Lauxr^2*(Kplus + Kminus))*Iaux else 0*j;
  Vmain = V_b*(p.vr + j*p.vi);
  Vaux = if sm > switch_open_speed  then Vmain else 0*j;

  Imain = (Vmain - j*w_b*Lambda_main)/Rmain;

  if sm > switch_open_speed and init == 1 then
    Vaux = Iaux*Raux + j*w_b*Lambda_aux;
  elseif sm > switch_open_speed and init == 2 then
    Vaux = Iaux*Raux + j*w_b*Lambda_aux - j*Iaux/(w_b*Cc);
  else
    //Vaux = 0*j;
    Iaux = 0*j;
  end if;

  //Iaux = if sm > switch_open_speed and init == 1 then (Vaux - j*w_b*Lambda_aux)/Raux elseif sm > switch_open_speed and init == 2 then (Vaux - j*w_b*Lambda_aux)/(Raux - j/(w_b*Cc)) else 0*j;

  p.ir = real(Imain + Iaux)/I_b;
  p.ii = imag(Imain + Iaux)/I_b;

  P = p.vr*p.ir + p.vi*p.ii;
  Q = (-p.vr*p.ii) + p.vi*p.ir;

  Pmotorbase = P/CoB;

  Pc = P*S_b;
  Qc = Q*S_b;

  Te_pre = (Lmainr^2*Imain*conj(Imain) + Lauxr^2*Iaux*conj(Iaux))*conj(Kplus - Kminus) + j*Lmainr*Lauxr*(conj(Imain)*Iaux - Imain*conj(Iaux))*conj(Kplus + Kminus);
  Te  =  (N/T_bm)*real(Te_pre);

  der(s) = (Tmech_pu_motor - Pmotorbase)/(2*H);
  annotation (preferredView = "info",Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end MD_DPIM_2;
