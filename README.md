# OpenIMDML - Open Instance Multi-Domain Motor Library
## Scope
The OpenIMDML is an open-source Modelica-based library that is focused on modeling and simulation of induction motor models in the phasor domain. It was built using the Modelica language, leveraging the Modelica Standard Library, and functions in conjunction with [OpenIPSL](https://github.com/OpenIPSL/OpenIPSL).
## OpenIMDML Library Structure
The library is structured into five sub-packages (illustrated in Figure shown below): Examples, NonMultiDomain, MultiDomain, Controls, and Functions.\
![Library Structure](docs/Figures/Library_structure.png "Library Structure")\
- **Examples:** contains Modelica model examples of all the components and models developed in the \texttt{OpenIMDML} Library. The \textit{Examples} package contains three sub-packages, namely \textit{MultiDomainExamples}, \textit{NonMultiDomainExamples}, and \textit{BaseClasses}. The \textit{MultiDomainExamples} sub-package presents examples of the single-phase and three-phase multi-domain motor developed in the library, including motor validation examples, examples including a variable speed drive and its controls, and a simple example of a multi-domain motor interacting with a pump that fills up a reservoir. The \textit{NonMultiDomainExamples} sub-package contains examples of single-phase and three-phase non multi-domain motor models from the library, including examples that incorporate the variable speed drive. The difference between multi-domain and non multi-domain is linked to the driven load representation. Lastly, the \textit{BaseClasses} sub-package contains partial examples of examples that are utilized in more than one example.


