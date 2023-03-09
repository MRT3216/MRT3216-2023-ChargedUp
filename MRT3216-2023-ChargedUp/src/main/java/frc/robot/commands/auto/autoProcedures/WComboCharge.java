package frc.robot.commands.auto.autoProcedures;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.DrivePath;
import frc.robot.commands.auto.PositionArmWrist;
import frc.robot.commands.auto.RunIntake;
import frc.robot.settings.Constants.ARM;
import frc.robot.settings.Constants.Auto;
import frc.robot.settings.Constants.WRIST;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class WComboCharge extends ParallelCommandGroup {
    public WComboCharge(SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem,
            IntakeSubsystem intakeSubsystem, LimelightSubsystem limelightSubsystem) {
        super(
                new FunctionalCommand(
                        () -> {
                            swerveSubsystem.zeroGyroscope();
                        },
                        () -> {
                        },
                        interrupted -> new InstantCommand(),
                        () -> true),
                new SequentialCommandGroup(
                        new WaitCommand(Auto.kStartDelayTime),
                        new PositionArmWrist(armSubsystem, ARM.kScoringHighConeDegrees, WRIST.kScoringHighConeDegrees),
                        new RunIntake(intakeSubsystem, false, true),
                        new ParallelCommandGroup(
                                new DrivePath(swerveSubsystem, armSubsystem, intakeSubsystem, limelightSubsystem,
                                        "cargoRetrieveW.1"),
                                new PositionArmWrist(armSubsystem, ARM.kGroundIntakeCubeDegrees,
                                        WRIST.kGroundIntakeCubeDegrees)),
                        new RunIntake(intakeSubsystem, true, false),
                        new ParallelCommandGroup(
                                new DrivePath(swerveSubsystem, armSubsystem, intakeSubsystem, limelightSubsystem,
                                        "comboChargeW.2"),
                                new PositionArmWrist(armSubsystem, ARM.kGroundIntakeCubeDegrees,
                                        WRIST.kGroundIntakeCubeDegrees)),
                        new RunIntake(intakeSubsystem, false, false),
                        new PositionArmWrist(armSubsystem, ARM.kStowedDegrees, WRIST.kStowedDegrees),
                        new DrivePath(swerveSubsystem, armSubsystem, intakeSubsystem, limelightSubsystem,
                                "comboChargeW.3")));

    }

}
