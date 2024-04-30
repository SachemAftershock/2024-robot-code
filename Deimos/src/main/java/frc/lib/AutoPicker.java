package frc.lib;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoPicker {
    private final SendableChooser<String> autoChooser;

    public AutoPicker(){
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("AmpsideRed", "AmpsideRed");
        autoChooser.addOption("HumanSideRed", "HumanSideRed");
        autoChooser.addOption("CenterSide3xRight", "CenterSide3xRight");
        autoChooser.addOption("CenterSide3xLeft", "CenterSide3xLeft");
        autoChooser.addOption("CenterSideSingle", "CenterSideSingle");
        autoChooser.addOption("Center4noteREDORec", "Center4noteREDORec");
        autoChooser.addOption("BlueSourceRedAmpTwoVALIDATEDRec", "BlueSourceRedAmpTwoVALIDATEDRec");
        autoChooser.addOption("SpeakerCenterSideTwoNoteVERIFIEDRec", "SpeakerCenterSideTwoNoteVERIFIEDRec");
        autoChooser.addOption("SpeakerRightSideTwoNoteVERIFIEDRec", "SpeakerRightSideTwoNoteVERIFIEDRec");
        autoChooser.addOption("CenterThreeNoteVALIDATEDRec", "CenterThreeNoteVALIDATEDRec");

        


        SmartDashboard.putData("Auto Selector", autoChooser);
    }

    public String getSelectedAuto() {
        return autoChooser.getSelected();
    }
}

