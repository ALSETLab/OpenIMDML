within OpenIMDML.Functions.ParameterEstimation.MotorParameterEstimationAuxiliaryFunctions;
function mean2numbers "Mean of two values"

  input Real a;
  input Real b;
  output Real c;
algorithm

  c:= (a + b)/2;
  annotation(preferredView = "info");
end mean2numbers;
