package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.settings.Constants.Auto;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DrivePath extends ParallelDeadlineGroup {
    // my sense of humor is writing a class called GoFetch lol
    public DrivePath(SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem,
             IntakeSubsystem intakeSystem, LimelightSubsystem limelightSubsystem,
             String pathname) {
        super(
                new DriveHolonomicTrajectory(swerveSubsystem,
                        PathPlanner.loadPath(pathname, Auto.kMaxFetchVelocity,
                                Auto.kMaxFetchAcc))
                );

    }
}