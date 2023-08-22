within OpenIMDML.Functions.ParameterEstimation.MotorParameterEstimationAuxiliaryFunctions;
function NEMALetterCode "NEMA letter code table range"

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

annotation(preferredView = "info");
end NEMALetterCode;
