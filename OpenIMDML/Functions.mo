within OpenIMDML;
package Functions
  extends Modelica.Icons.FunctionsPackage;
  package ParameterEstimation
    function RUN_InductionMotorParameterEstimation "Iterative method for estimating the induction motor equivalent
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

      // Inputs
      input Real Php "Rated Output HorsePower";
      input Real eff "Full Load Efficiency";
      input Real PF "Full Load Power Factor";
      input Real f "System Electrical Frequency";
      input Integer N "Number of Poles";
      input Real Nr "Full Load Speed";
      input String NEMACode "NEMA Code Letter";
      input String NEMADesign "Nema Design Class";
      input Real VRated "Rated Terminal Voltage";

      // Variables
    //protected
       Complex I1;
       Real Pout "Rated Output HorsePower";
       Real Pin;
       Real Ploss;
       Real Sin;
       Real Qin;
       Real Ns;
       Real s;
       Real ILR;
       output Real Pmech;
       output Real Pstray;
       output Real Pcore;
       Real Pconv;
       output Real PAG;
       output Real PSCL;
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

    end RUN_InductionMotorParameterEstimation;
    extends Modelica.Icons.FunctionsPackage;

    package MotorParameterEstimationAuxiliaryFunctions
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

      function NEMALetterCode

        input String NemaCode;
        output Modelica.Units.SI.ReactivePower SLR "Locked Rotor Reactive Power";

      protected
        parameter Real A = mean2numbers( 0,     3.15);
        parameter Real B = mean2numbers( 3.15,  3.55);
        parameter Real C = mean2numbers( 3.55,  4.00);
        parameter Real D = mean2numbers( 4.00,  4.50);
        parameter Real E = mean2numbers( 4.50,  5.00);
        parameter Real F = mean2numbers( 5.00,  5.60);
        parameter Real G = mean2numbers( 5.60,  6.30);
        parameter Real H = mean2numbers( 6.30,  7.10);
        parameter Real J = mean2numbers( 7.10,  8.00);
        parameter Real K = mean2numbers( 8.00,  9.00);
        parameter Real L = mean2numbers( 9.00, 10.00);
        parameter Real M = mean2numbers(10.00, 11.20);
        parameter Real N = mean2numbers(11.20, 12.50);
        parameter Real O = mean2numbers(12.50, 14.00);
        parameter Real P = mean2numbers(14.00, 16.00);
        parameter Real R = mean2numbers(16.00, 18.00);
        parameter Real S = mean2numbers(18.00, 20.00);
        parameter Real T = mean2numbers(20.00, 22.40);
        parameter Real U = mean2numbers(22.40, 22.40);

      algorithm

        //Locked Rotor Reactive Power Estimation
      SLR := if NemaCode == "A" then A elseif NemaCode == "B" then B elseif NemaCode == "C" then C
         elseif NemaCode == "D" then D elseif NemaCode == "E" then E elseif NemaCode == "F" then F
         elseif NemaCode == "G" then G elseif NemaCode == "H" then H elseif NemaCode == "J" then J
         elseif NemaCode == "K" then K elseif NemaCode == "L" then L elseif NemaCode == "M" then M
         elseif NemaCode == "N" then N elseif NemaCode == "O" then O elseif NemaCode == "P" then P
         elseif NemaCode == "R" then R elseif NemaCode == "S" then S elseif NemaCode == "T" then T
         else U;
      end NEMALetterCode;

      function NEMADesignClass

      input String Design;
      output Real Ratio;

      algorithm

      Ratio := if Design == "A" then 0.5 elseif Design == "B" then 0.6 elseif Design == "C" then 0.7 else 0.5;
      end NEMADesignClass;

      function mean2numbers

        input Real a;
        input Real b;
        output Real c;
      algorithm

        c:= (a + b)/2;
      end mean2numbers;
    end MotorParameterEstimationAuxiliaryFunctions;
  end ParameterEstimation;
end Functions;
