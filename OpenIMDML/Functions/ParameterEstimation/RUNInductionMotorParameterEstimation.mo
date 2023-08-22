within OpenIMDML.Functions.ParameterEstimation;
function RUNInductionMotorParameterEstimation "Iterative method for estimating the induction motor equivalent
circuit parameters using only the motor nameplate data."

  import          Modelica.Units.SI;
  import         Modelica.Units.NonSI;
  import Modelica.Units.Conversions.*;
  import Complex;
  import Modelica.ComplexMath.arg;
  import Modelica.ComplexMath.real;
  import Modelica.ComplexMath.imag;
  import Modelica.ComplexMath.j;
  import Modelica.ComplexMath.abs;
  import OpenIMDML.Functions.ParameterEstimation.MotorParameterEstimationAuxiliaryFunctions.MotorParameterEstimationGaussSeidel;
  import OpenIMDML.Functions.ParameterEstimation.MotorParameterEstimationAuxiliaryFunctions.NEMALetterCode;
  import OpenIMDML.Functions.ParameterEstimation.MotorParameterEstimationAuxiliaryFunctions.NEMADesignClass;
  import OpenIMDML.Functions.ParameterEstimation.MotorParameterEstimationAuxiliaryFunctions.mean2numbers;

  // Inputs
  input Real Php = 15 "Rated Output HorsePower";
  input Real eff = 0.917 "Full Load Efficiency";
  input Real PF = 0.77 "Full Load Power Factor";
  input Real f = 60 "System Electrical Frequency";
  input Integer N = 6 "Number of Poles";
  input Real Nr = 1180 "Full Load Speed";
  input String NEMACode = "F" "NEMA Code Letter";
  input String NEMADesign = "B" "Nema Design Class";
  input Real VRated = 230 "Rated Terminal Voltage";

  // Variables
protected
   Complex I1;
   Real Pout "Rated Output HorsePower";
   Real Pin;
   Real Ploss;
   Real Sin;
   Real Qin;
   Real Ns;
   Real s;
   Real ILR;
   //output Real Pmech;
   //output Real Pstray;
   //output Real Pcore;
   Real Pmech;
   Real Pstray;
   Real Pcore;
   Real Pconv;
   //output Real PAG;
   //output Real PSCL;
   Real PAG;
   Real PSCL;
   Real PRCL;
   Real I1mag;
   Real R1;
   Real I1_r;
   Real I1_i;

  // Parameter
  //parameter Real Fmech = 0.53871877102;
  parameter Real Fmech = 0.536;
  parameter Real Fcore = 0.0;
  //parameter Real Fstray = if Pout < 91e3 then 0.018 elseif (91e3 <= Pout and Pout < 375e3) then 0.015 elseif (375e3 <= Pout and Pout < 1850e3) then 0.012 else 0.009;
  parameter Real Fstray = 0;

algorithm
  clearlog();
  Pout := Php*746;
  Pin := Pout/eff;
  Ploss := Pin - Pout;
  Sin := Pout/(eff*PF);
  Qin := sqrt(Sin^2 - Pin^2);
  I1mag := Sin/VRated;
  I1 := (Pin - j*Qin)/VRated;
  I1_r := real(I1);
  I1_i := imag(I1);
  Ns := 120*f/N;
  s := (Ns - Nr)/Ns;
  ILR := Php*(1e3*MotorParameterEstimationAuxiliaryFunctions.NEMALetterCode(
    NEMACode))/VRated;
  Pmech := Ploss*Fmech;
  Pstray := Pout*Fstray;
  Pcore := Pout*Fcore;
  Pconv := Pout + Pmech + Pstray;
  PAG := Pconv/(1-s);
  PSCL := Pin - PAG - Pcore;
  PRCL := PAG - Pconv;
  R1 := PSCL/(I1mag^2);
  Modelica.Utilities.Streams.print("(1) - Up to here it is working fine");

  MotorParameterEstimationAuxiliaryFunctions.MotorParameterEstimationGaussSeidel(
    I1_r,
    I1_i,
    R1,
    VRated,
    ILR,
    s,
    PAG,
    Qin,
    NEMADesign);
annotation(preferredView = "info");
end RUNInductionMotorParameterEstimation;
