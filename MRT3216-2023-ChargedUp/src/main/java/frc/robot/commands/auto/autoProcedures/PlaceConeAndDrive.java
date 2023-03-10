package frc.robot.commands.auto.autoProcedures;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.DrivePath;
import frc.robot.settings.Constants.Auto;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class PlaceConeAndDrive extends ParallelCommandGroup {
    public PlaceConeAndDrive(SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem,
            IntakeSubsystem intakeSystem, LimelightSubsystem limelightSubsystem) {
        super(
                new SequentialCommandGroup(
                        new WaitCommand(Auto.kStartDelayTime),
                        new PlaceHighCone(swerveSubsystem, armSubsystem, intakeSystem, limelightSubsystem),
                        new DrivePath(swerveSubsystem, armSubsystem, intakeSystem, limelightSubsystem,
                                "PlaceAndDrive")));

    }
}
