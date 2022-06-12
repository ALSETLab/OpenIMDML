within OpenIMDML.Functions.ParameterEstimation.MotorParameterEstimationAuxiliaryFunctions;
function MotorParameterEstimationGaussSeidel

  import Modelica.Units.SI;
  import Modelica.Units.NonSI;
  import Modelica.Units.Conversions.*;
  import Complex;
  import Modelica.ComplexMath.arg;
  import Modelica.ComplexMath.real;
  import Modelica.ComplexMath.imag;
  import Modelica.ComplexMath.j;
  import Modelica.ComplexMath.abs;

  parameter Integer n = 10;

  // Input Variables
  input Real I1_real;
  input Real I1_imag;
  input Real R1;
  input Real VRated;
  input Real ILR;
  input Real s;
  input Real PAG;
  input Real Qin;
  input String MotorDesign;

  // Output Variables
  output Real r1;
  output Real R2_final;
  output Real X1_final;
  output Real X2_final;
  output Real XM_final;

  // Intermediate Variables
protected
  Real XLR[n];
  Real Emag;
  Real I2_real[n];
  Real I2_imag[n];
  Real E_real[n];
  Real E_imag[n];
  Real X1[n];
  Real R2[n];
  Real X2[n];
  Real XM[n];

  /*Real [:] XLR;
  Real Emag;
  Real [:] I2_real;
  Real [:] I2_imag;
  Real [:] E_real;
  Real [:] E_imag;
  Real [:] X1;
  Real [:] R2;
  Real [:] X2;
  Real [:] XM;*/

algorithm

E_real[1] := VRated;
E_imag[1] := 0;
I2_real[1] := I1_real;
I2_imag[1] := 0;
R2[1] := 0;
X2[1] := 0;
XM[1] := 0;
XLR[1] := 0;

Modelica.Utilities.Streams.print("(2) - Up to here it is working fine.");

  for i in 2:n loop
    Emag :=sqrt(E_real[i-1]^2 + E_imag[i-1]^2);
    R2[i] := s*((Emag)^2 + sqrt((Emag)^4 - 4*PAG*(X2[i-1])^2))/(2*PAG);
    XLR[i] := sqrt((VRated/ILR)^2 - (R1 + R2[i])^2);
    X1[i] := XLR[i]*NEMADesignClass(MotorDesign);
    X2[i] := XLR[i]*(1 - NEMADesignClass(MotorDesign));
    XM[i] := Emag^2/(Qin - (I1_real^2 + I1_imag^2)*X1[i] - (I2_real[i]^2 + I2_imag[i]^2)^2*X2[i]);
    E_real[i] := VRated - (I1_real*R1 - I1_imag*X1[i]);
    E_imag[i] := -(I1_imag*R1 + I1_real*X1[i]);
    I2_real[i] := ((E_real[i]*R2[i]/s) + E_imag[i]*X2[i])/((R2[i]/s)^2 + X2[i]^2);
    I2_imag[i] := ((E_imag[i]*R2[i]/s) - E_real[i]*X2[i])/((R2[i]/s)^2 + X2[i]^2);
  end for;

r1:=R1;
X1_final := X1[n];
X2_final := X2[n];
R2_final := R2[n];
XM_final := XM[n];

Modelica.Utilities.Streams.print("Estimated Parameter R1, R2, X1, X2, and Xm [ohm].");

end MotorParameterEstimationGaussSeidel;
