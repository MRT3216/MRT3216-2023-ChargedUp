package frc.robot.commands.auto.autoProcedures;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.DrivePath;
import frc.robot.settings.Constants.Auto;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class PlaceCubeAndDrive extends ParallelCommandGroup {
    public PlaceCubeAndDrive(SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem,
            IntakeSubsystem intakeSystem, LimelightSubsystem limelightSubsystem) {
        super(

                new SequentialCommandGroup(
                        new WaitCommand(Auto.kStartDelayTime),
                        new PlaceHighCone(swerveSubsystem, armSubsystem, intakeSystem, limelightSubsystem),
                        Commands.print("Drive auto starting"),
                        swerveSubsystem.followTrajectoryCommand(PathPlanner.loadPath("PlaceAndDrive", Auto.kMaxFetchVelocity, Auto.kMaxFetchAcc), true),
                        Commands.print("Drive auto finished"),
                        Commands.runOnce(() -> swerveSubsystem.stop(), swerveSubsystem)
                        ));

    }
}
