package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Auto {
    private static Auto instance;
    private ArmSubsystem armSubsystem;
    private IntakeSubsystem intakeSubystem;
    private SwerveSubsystem swerveSubsystem;

    public Auto() {
        armSubsystem = ArmSubsystem.getInstance();
        intakeSubystem = IntakeSubsystem.getInstance();
        swerveSubsystem = SwerveSubsystem.getInstance();
    }

    public Command getDriveBackwardsCommand() {
        // return swerveSubsystem.getFollowTrajectoryCommand(
        // PathPlanner.loadPath("PlaceAndDrive", AUTO.kMaxFetchVelocity,
        // AUTO.kMaxFetchAcc), true);
        return new RunCommand(() -> swerveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        0.1,
                        0,
                        0,
                        this.swerveSubsystem.getGyroscopeRotation())));

    }

    public static Auto getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new Auto();
        }
        return instance;
    }
}
