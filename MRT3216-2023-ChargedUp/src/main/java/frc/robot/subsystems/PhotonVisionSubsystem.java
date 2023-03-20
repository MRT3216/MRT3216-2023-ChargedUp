package frc.robot.subsystems;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
    AprilTagFieldLayout aprilTagFieldLayout;

    private PhotonVisionSubsystem() {
        try {
            aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch(IOException e) {
            // TODO: decide what you want to do if the layout fails to load
            System.out.println("Whoops! AprilTag layout file not found.");
        }
    }

}
