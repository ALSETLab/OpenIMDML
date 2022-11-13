within OpenIMDML.MultiDomainModels.Motors.SinglePhase;
model MD_DPIM
  "This model is the steady-state circuit model of the single phase induction motor model initialized by either a split-phase auxiliary circuit or a capacitor-start auxiliary circuit."

extends
    OpenIMDML.MultiDomainModels.Motors.SinglePhase.BaseClasses.BaseMultiDomainSinglePhase(
      Rotor_Inertia(w(fixed=false, start=w0)));

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
  OpenIPSL.Types.PerUnit P;
  OpenIPSL.Types.PerUnit Q;
  OpenIPSL.Types.PerUnit s;
  OpenIPSL.Types.PerUnit Te1 "First Component of the Electrical Torque";
  OpenIPSL.Types.PerUnit Te2 "Second Component of the Electrical Torque";
  OpenIPSL.Types.PerUnit Te "Total Electrical Torque";
  Modelica.Units.SI.ActivePower Pc;
  Modelica.Units.SI.ReactivePower Qc;

    //Modelica.SIunits.Torque Tele;
  Modelica.Units.SI.Current Iaux_real;
  Modelica.Units.SI.Current Iaux_imag;
  Modelica.Units.SI.Current Imain_real;
  Modelica.Units.SI.Current Imain_imag;
  Modelica.Units.SI.Current Itotal;
  Modelica.Units.SI.Current Itotal_real;
  Modelica.Units.SI.Current Itotal_imag;
  Modelica.Units.SI.Voltage Vmain_real;
  Modelica.Units.SI.Voltage Vmain_imag;
  Modelica.Units.SI.Voltage Vaux_real;
  Modelica.Units.SI.Voltage Vaux_imag;
  Modelica.Units.SI.Voltage Vmain_aux_real;
  Modelica.Units.SI.Voltage Vmain_aux_imag;
  Real K1_real;
  Real K1_imag;
  Real K2_real;
  Real K2_imag;
  Real K3_real;
  Real K3_imag;
  Real KplusK_real;
  Real KplusK_imag;
  Real KminusK_real;
  Real KminusK_imag;
  Real Kden_real;
  Real Kden_imag;
  Modelica.Units.SI.Conductance Cond1_aux_real;
  Modelica.Units.SI.Conductance Cond1_aux_imag;
  Modelica.Units.SI.Conductance Cond2_aux_real;
  Modelica.Units.SI.Conductance Cond2_aux_imag;
  Modelica.Units.SI.Conductance Cond1_main_real;
  Modelica.Units.SI.Conductance Cond1_main_imag;
  Real Constant1;
  Real Constant2;

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

  //SI Reactance of the Main Winding of the Dual Phase Induction Motor
  Xmain = w_b*Lmain;
  Xaux = w_b*Laux;

  //Slip for calculations
  sm = s;

  // Calculation of the coeficients of the two-phase motor

  KplusK_real = Rr*w_b*(Rr^2 - Lr^2*(sm-2)*sm*w_b^2)/(((Lr^2*(sm-2)^2*w_b^2) + Rr^2)*((Lr*sm*w_b)^2 + Rr^2));
  KplusK_imag = -Lr*w_b^2*((Lr*(sm-2)*sm*w_b)^2 + Rr^2*(sm^2-2*sm+2))/(((Lr*(sm-2)*w_b)^2 + Rr^2)*((Lr*sm*w_b)^2 + Rr^2));
  KminusK_real = Rr*(sm-1)*w_b*(Lr^2*(sm-2)*sm*w_b^2 + Rr^2)/(((Lr*(sm-2)*w_b)^2 + Rr^2)*((Lr*sm*w_b)^2 + Rr^2));
  KminusK_imag = -2*Lr*Rr^2*(sm-1)*w_b^2/(((Lr*(sm-2)*w_b)^2 + Rr^2)*((Lr*sm*w_b)^2 + Rr^2));

  K1_real = Rmain + w_b*Lmainr^2*KplusK_real;
  K1_imag = w_b*Lmain + w_b*Lmainr^2*KplusK_imag;

  K2_real = -w_b*Lmainr*Lauxr*KminusK_imag;
  K2_imag = w_b*Lmainr*Lauxr*KminusK_real;

  K3_real = Raux + w_b*Lauxr^2*KplusK_real;
  K3_imag = if init == 1 then w_b*Laux + w_b*Lauxr^2*KplusK_imag else w_b*Laux + w_b*Lauxr^2*KplusK_imag - 1/(w_b*Cc);

  Kden_real = (K2_real^2 - K2_imag^2 + K1_real*K3_real - K1_imag*K3_imag);
  Kden_imag = (2*K2_real*K2_imag + K1_real*K3_imag + K3_real*K1_imag);

  Cond2_aux_real = (K2_real*Kden_real + K2_imag*Kden_imag)/(Kden_real^2 + Kden_imag^2);
  Cond2_aux_imag = (K2_imag*Kden_real - K2_real*Kden_imag)/(Kden_real^2 + Kden_imag^2);

  Cond1_aux_real = (K1_real*Kden_real + K1_imag*Kden_imag)/(Kden_real^2 + Kden_imag^2);
  Cond1_aux_imag = (K1_imag*Kden_real - K1_real*Kden_imag)/(Kden_real^2 + Kden_imag^2);

  Cond1_main_real = K1_real/(K1_real^2 + K1_imag^2);
  Cond1_main_imag = K1_imag/(K1_real^2 + K1_imag^2);
  Constant1 = (K2_real*K1_real + K2_imag*K1_imag)/(K1_real^2 + K1_imag^2);
  Constant2 = (K2_imag*K1_real - K2_real*K1_imag)/(K1_real^2 + K1_imag^2);

  Vmain_real = p.vr*V_b;
  Vmain_imag = p.vi*V_b;
  Vmain_aux_real = if sm > switch_open_speed then p.vr*V_b else 0;
  Vmain_aux_imag = if sm > switch_open_speed then p.vi*V_b else 0;
  Vaux_real = if sm > switch_open_speed then p.vr*V_b else 0;
  Vaux_imag = if sm > switch_open_speed then p.vi*V_b else 0;

  Iaux_real = Cond1_aux_real*Vaux_real - Cond1_aux_imag*Vaux_imag + Cond2_aux_real*Vaux_real - Cond2_aux_imag*Vaux_imag;
  Iaux_imag = (Cond1_aux_real*Vaux_imag + Cond1_aux_imag*Vaux_real + Cond2_aux_real*Vaux_imag + Cond2_aux_imag*Vaux_real);
  Imain_real = Cond1_main_real*Vmain_real - Cond1_main_imag*Vmain_imag - Constant1*Iaux_real + Constant2*Iaux_imag;
  Imain_imag = -(Cond1_main_imag*Vmain_real + Cond1_main_real*Vmain_imag - Constant2*Iaux_real - Constant1*Iaux_imag);
  Itotal = sqrt((Imain_real + Iaux_real)^2 + (Imain_imag + Iaux_imag)^2);
  Itotal_real = Imain_real + Iaux_real;
  Itotal_imag = (Imain_imag + Iaux_imag);
  p.ir = Itotal_real/I_b;
  p.ii = Itotal_imag/I_b;

  P = p.vr*p.ir + p.vi*p.ii;
  Q = (-p.vr*p.ii) + p.vi*p.ir;
  Pc = P*S_b;
  Qc = Q*S_b;


  Te1 = ((Lmainr^2*(Imain_real^2 + Imain_imag^2) + Lauxr^2*(Iaux_real^2 + Iaux_imag^2))*KminusK_real)/T_bm;
  Te2 = (2*Lmainr*Lauxr*KplusK_real*(Imain_real*Iaux_imag - Imain_imag*Iaux_real))/T_bm;
  Te  =  N*(Te1 + Te2);

  der(s) = (Tmech_pu_motor - Te)/(2*H);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end MD_DPIM;
