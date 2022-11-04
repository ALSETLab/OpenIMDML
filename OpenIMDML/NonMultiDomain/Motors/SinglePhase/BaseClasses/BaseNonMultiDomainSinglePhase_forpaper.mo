within OpenIMDML.NonMultiDomain.Motors.SinglePhase.BaseClasses;
partial model BaseNonMultiDomainSinglePhase_forpaper
  "Base class model for the single-phase and dual-phase non multi-domain induction motor models."

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
          extent={{-50,50},{50,-50}},
          lineColor={0,0,0},
          textString="M"),                Ellipse(
          fillColor={255,255,255},
          extent={{-56,-56},{55.932,56}}),
          Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0}),
                          Text(
          origin={0,-80},
          extent={{-80,-20},{80,20}},
          fontName="Arial",
          lineColor={0,0,0},
          textString="BaseNonMultDomainMotor
Single-Phase"),
        Text(
          extent={{-80,90},{-28,50}},
          textColor={238,46,47},
          textString="NMD")}),  Diagram(coordinateSystem(preserveAspectRatio=false)));
end BaseNonMultiDomainSinglePhase_forpaper;
