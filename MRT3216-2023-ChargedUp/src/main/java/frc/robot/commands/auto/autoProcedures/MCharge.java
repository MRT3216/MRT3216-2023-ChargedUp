package frc.robot.commands.auto.autoProcedures;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.DrivePath;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class MCharge extends ParallelCommandGroup {
    public MCharge(SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem,
            IntakeSubsystem intakeSystem, LimelightSubsystem limelightSubsystem) {
        super(
                new SequentialCommandGroup(
                        new PlaceHighCone(swerveSubsystem, armSubsystem, intakeSystem, limelightSubsystem),
                        new DrivePath(swerveSubsystem, armSubsystem, intakeSystem,
                                limelightSubsystem, "MCharge")));
    }
}
