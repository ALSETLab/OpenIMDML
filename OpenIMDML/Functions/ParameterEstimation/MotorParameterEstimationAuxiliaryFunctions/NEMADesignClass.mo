within OpenIMDML.Functions.ParameterEstimation.MotorParameterEstimationAuxiliaryFunctions;
function NEMADesignClass "NEMA design class range"

input String Design;
output Real Ratio;

algorithm

Ratio := if Design == "A" then 0.5 elseif Design == "B" then 0.6 elseif Design == "C" then 0.7 else 0.5;
annotation(preferredView = "info");
end NEMADesignClass;
