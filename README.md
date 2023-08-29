# OpenIMDML - Open Instance Multi-Domain Motor Library
## Scope
The OpenIMDML is an open-source Modelica-based library that is focused on modeling and simulation of induction motor models in the phasor domain. It was built using the Modelica language, leveraging the Modelica Standard Library, and functions in conjunction with [OpenIPSL](https://github.com/OpenIPSL/OpenIPSL). The library contains an extensive selection of single-phase and three-phase induction motor models, both in multi-domain and non-multi-domain modeling representation, and a variable speed drive model that is not modeled in traditional power system tools but is ubiquitously used in startup and operation of induction motors.
### Citing OpenIMDML in Publications
If you use OpenIMDML in your work or research, all we ask in exchange is that you cite the reference publications according to your use. Preferably, please cite this repository by using our preferred reference, as seen on the GitHub GUI.

We have submitted a paper to the SoftwareX journal, which we would welcome you to cite as:

> Fachini, Fernando, et al. "OpenIMDML: Open Instance Multi-Domain Motor Library utilizing the Modelica Modeling Language." SoftwareX (2023)

Previous work leading to the library has been published and can also be cited as:
> Fachini, Fernando, et al. "Multi-domain power and thermo-fluid system stability modeling using modelica and openipsl." 2022 IEEE Power & Energy Society General Meeting (PESGM). IEEE, 2022.

## OpenIMDML Library Structure
The library is structured into five sub-packages (illustrated in Figure shown below): Examples, NonMultiDomain, MultiDomain, Controls, and Functions.\
![Library Structure](docs/Figures/Library_structure.png "Library Structure")
- **Examples:** contains Modelica model examples of all the components and models developed in the OpenIMDML Library.
- **NonMultiDomain:** contains single-phase and three-phase non multi-domain induction motor models.
-  **MultiDomain:**  contains single-phase and three-phase multi-domain induction motor models.
-  **Controls:** contains power electronics and controller logic models for the variable speed drive model.
-  **Functions:** contains one function that implements an iterative method for estimating the induction motor equivalent circuit parameters using only the motor nameplate data.

