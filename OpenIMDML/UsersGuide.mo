within OpenIMDML;
package UsersGuide "User's Guide"
  extends Modelica.Icons.Information;
  model Overview "Overview"
    extends Modelica.Icons.Information;

  annotation(preferredView = "info",
                Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p>The OpenIMDML Library has five packages: Examples, NonMultiDomain, MultiDomain, Controls, and Functions. They are briefly described below: </p>
<ul>
<li><a href=\"modelica://OpenIMDML.Examples\">Examples</a>: Contains example cases for running the multidomain and non-multidomain motors in different scenarios.</li>
<li><a href=\"modelica://OpenIMDML.NonMultiDomain\">NonMultiDomain</a>: Contains multiple variations of non-multidomain single-phase and three-phase motor models.</li>
<li><a href=\"modelica://OpenIMDML.MultiDomain\">MultiDomain:</a> Contains multiple variations of multidomain single-phase and three-phase motor models.</li>
<li><a href=\"modelica://OpenIMDML.Controls\">Controls</a><a href=\"modelica://DroneLibrary.Mechanical\">:</a> Contains both the power electronics model as well as the controller logic model.</li>
<li><a href=\"modelica://OpenIMDML.Functions\">Functions</a><a href=\"modelica://DroneLibrary.Sensors\">:</a> Contains a function that estimates induction motor parameters based on nameplate information.</li>
</html>"));
  end Overview;

  package ReleaseNotes "Release Notes"
    extends Modelica.Icons.ReleaseNotes;
    class v100 "Version 1.0.0 (2023-08-21)"
       extends Modelica.Icons.ReleaseNotes;

      annotation (preferredView="info",Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)),
        Documentation(info="<html>
<h4>What&apos;s Changed</h4>
<p>This version of <code>OpenIMDML</code> is the initial release. </p>
<h5>Highlights</h5>
<ul>
<li>Multi-domain models of three-phase and single-phase induction motors.</li>
<li>Variable speed drive model for motor control.</li>
<li>Example cases to simulate different fidelity level multi-domain motors.</li>
<li>Example cases to simulate the motor operation with variable speed drive.</li>
<li>Simple example of a multi-domain motor filling up a reservoir with water. </li>
</html>"),     preferredView = "info");
    end v100;

    annotation (Documentation(info="<html>
<p>List of releases of the library.</p>
</html>"),   preferredView = "info");
  end ReleaseNotes;

  model References "References"
    extends Modelica.Icons.References;

  annotation(preferredView = "info",
                Documentation(info="<html>
<dl>
<dt>[Baudette2018]</dt>
<dd>M.Baudette, et al , OpenIPSL: Open-instance power system li-
brary—update 1.5 to “iTesla power systems library (iPSL): A modelica
library for phasor time-domain simulations”, SoftwareX 7 (2018) 34–36.
</dd>
<dt>[Chapman2004]</dt>
<dd>S. J Chapman, Electric machinery fundamentals, McGraw-hill, 2004.
</dd>
<dt>[Chow2020]</dt>
<dd>J. H. Chow, J. J. Sanchez-Gasca, Power system modeling, computation,
and control, John Wiley & Sons, 2020.
</dd>
<dt>[DOE2014]</dt>
<dd>U. DOE, Improving motor and drive system performance: a sourcebook
for industry, US Department of Energy (2014).
</dd>
<dt>[Fachini2022]</dt>
<dd>F. Fachini, M. De Castro, M. Liu, T. Bogodorova, L. Vanfretti, W. Zuo,
Multi-domain power and thermo-fluid system stability modeling using
Modelica and OpenIPSL, in: 2022 IEEE Power & Energy Society General
Meeting (PESGM), IEEE, 2022, pp. 1–5.
</dd>
<dt>[Fitzgerald2003]</dt>
<dd>A. E. Fitzgerald, C. Kingsley, S. D. Umans, B. James, Electric machin-
ery, Vol. 5, McGRAW-hill New York, 2003.
</dd>
<dt>[Fritzon2014]</dt>
<dd>P. Fritzson, Principles of object-oriented modeling and simulation with
Modelica 3.3: a cyber-physical approach, John Wiley & Sons, 2014.
</dd>
<dt>[Kosterev2008]</dt>
<dd>D. Kosterev. et al, Load modeling in power system studies: WECC
progress update, in: 2008 IEEE PES GM, 2008, pp. 1–8.
</dd>
<dt>[Ma2020]</dt>
<dd>Z. Ma et al, Mathematical representation of WECC composite load
model, Journal of Modern Power Systems and Clean Energy 8 (5) (2020)
1015–1023.
</dd>
<dt>[Milano2005]</dt>
<dd>F. Milano, An open source power system analysis toolbox, IEEE Trans-
actions on Power systems 20 (3) (2005) 1199–1206.
</dd>
<dt>[Milano2008]</dt>
<dd>F. Milano, L. Vanfretti, J. C. Morataya, An open source power system
virtual laboratory: The PSAT case and experience, IEEE Transactions on
Education 51 (1) (2008) 17–23.
</dd>
<dt>[Milano2010]</dt>
<dd>F. Milano, Power system modeling and scripting, Springer Science &
Business Media, 2010.
</dd>
<dt>[Ong1998]</dt>
<dd>C.-M. Ong, et al., Dynamic simulation of electric machinery: using
MATLAB/SIMULINK, Vol. 5, Prentice hall PTR Upper Saddle River,
NJ, 1998.
</dd>
<dt>[OpenIPSL2023]</dt>
<dd>M. de Castro, D. Winkler, G. Laera, L. Vanfretti, S. A. Dorado-Rojas,
T. Rabuzin, B. Mukherjee, M. Navarro, Version [openipsl 2.0. 0]-[itesla
power systems library (IPSL): A Modelica library for phasor time-domain
simulations], SoftwareX 21 (2023) 101277.
</dd>
<dt>[Siemens2017]</dt>
<dd>Siemens PTI, PSS®e 34.2.0 model library, Siemens Power Technologies
International, Schenectady, NY (2017).
</dd>
<dt>[Vanfretti2016]</dt>
<dd>L. Vanfretti, T. Rabuzin, M. Baudette, M. Murad, iTesla Power Systems
Library (IPSL): A Modelica library for phasor time-domain simulations,
SoftwareX 5 (2016) 84–88.
</dd>
</dl>
</html>"));
  end References;
annotation(preferredView = "info", Documentation(info="<html>
<p>
The <strong>Open-Instance MultiDomain Motor Library - <code>OpenIMDML</code></strong>
contains many single-phase and three-phase induction motor models written in <a href=\"http://modelica.org\">Modelica</a>
language that are meant to be used with the <strong>Open-Instance Power System Library - <code>OpenIPSL</code></strong>.
</p>
<p>
<strong><code>OpenIPSL</code></strong>, which is the foundation library to <strong><code>OpenIMDML</code></strong>, contains non-multi-domain induction
motor models that were implemented based on <a href=\"http://faraday1.ucd.ie/psat.html\">PSAT</a>. <strong><code>OpenIMDML</code></strong>, on the other hand, was implemented to add multi-domain capability to the foundation library by means
of multi-domain motor and variable speed drive models.
</p>
<p>
This is a very short <strong>User's Guide</strong> that will try to help users to get familiar
with the library providing general information about the <strong><code>OpenIMDML</code></strong>.
</p>
<p>
More information about the library can be found on this <a href=\"https://github.com/ALSETLab/OpenIMDML\">openimdml.org</a>
dedicated to the <strong><code>OpenIMDML</code></strong> organization.
</p>
</html>"));
end UsersGuide;
