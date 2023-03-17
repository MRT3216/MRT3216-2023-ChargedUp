package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.settings.Constants.AUTO;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoCommands {
    private static AutoCommands instance;
    private ArmSubsystem armSubsystem;
    private IntakeSubsystem intakeSubystem;
    private SwerveSubsystem swerveSubsystem;

    public AutoCommands() {
        armSubsystem = ArmSubsystem.getInstance();
        intakeSubystem = IntakeSubsystem.getInstance();
        swerveSubsystem = SwerveSubsystem.getInstance();
    }

    // public Command getDriveBackwardsCommand() {
    // PathPlannerTrajectory path = PathPlanner.loadPath("Drive", new
    // PathConstraints(1, 1));
    // return new WaitCommand(AUTO.kStartDelayTime).andThen(
    // swerveSubsystem.getFollowTrajectoryCommand(path, true));
    // }

    // public Command getPlaceAndDriveCommand() {
    // PathPlannerTrajectory path = PathPlanner.loadPath("PlaceAndDrive", new
    // PathConstraints(1, 1));
    // return swerveSubsystem.getFollowTrajectoryCommand(path, true);
    // }

    public static AutoCommands getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new AutoCommands();
        }
        return instance;
    }
}
