package frc.robot.commands.auto.autoProcedures;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.settings.Constants.ARM;
import frc.robot.settings.Constants.Auto;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class PlaceHighCone extends ParallelCommandGroup {
    public PlaceHighCone(SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem,
            IntakeSubsystem intakeSystem, LimelightSubsystem limelightSubsystem) {
        super(
                new SequentialCommandGroup(
                        new WaitCommand(Auto.kStartDelayTime),
                        armSubsystem.getCommand(ARM.Position.ScoringHighCone),
                        // new RunIntake(intakeSystem, false, true).withTimeout(2),
                        intakeSystem.getAutoConeCommand(false),
                        armSubsystem.getCommand(ARM.Position.Stowed)));
    }
}
