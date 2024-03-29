within OpenIMDML.NonMultiDomain.Motors.SinglePhase.BaseClasses;
partial model BaseNonMultiDomainSinglePhase "Base class model for the single-phase and dual-phase non multi-domain induction motor models."

  parameter OpenIPSL.Types.ApparentPower M_b = SysData.S_b "Machine base power"
                                                                               annotation (Dialog(group="Power flow data"));
  extends OpenIPSL.Electrical.Essentials.pfComponent(
    final enabledisplayPF=false,
    final enablefn=false,
    final enableV_b=false,
    final enableangle_0=true,
    final enablev_0=true,
    final enableQ_0=true,
    final enableP_0=true,
    final enableS_b=true);

  OpenIPSL.Interfaces.PwPin p
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Text(
          extent={{-100,-60},{100,-100}},
          lineColor={28,108,200},
          textString="%name"),             Text(
          extent={{-50,50},{50,-50}},
          lineColor={0,0,0},
          textString="M"),                Ellipse(
          fillColor={255,255,255},
          extent={{-56,-56},{55.932,56}}),
          Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0})}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end BaseNonMultiDomainSinglePhase;
