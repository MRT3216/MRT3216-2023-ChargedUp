package frc.robot.commands.auto.autoProcedures;

import javax.swing.text.Position;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

public class PlaceCubeAndDrive extends ParallelCommandGroup{
    public PlaceCubeAndDrive(SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem,
                         IntakeSubsystem intakeSystem, LimelightSubsystem limelightSubsystem) {
        super(
                new FunctionalCommand(
                    () -> {
                        swerveSubsystem.zeroGyroscope();
                    }, 
                    () -> {}, 
                    interrupted -> new InstantCommand(), 
                    () -> true
                ),
                new SequentialCommandGroup(
                    new WaitCommand(Auto.kStartDelayTime),
                    new PlaceHighCube(swerveSubsystem, armSubsystem, intakeSystem, limelightSubsystem),
                    new DrivePath(swerveSubsystem, armSubsystem, intakeSystem, limelightSubsystem, "PlaceAndDrive")          
                )
        );
                            
    }
}
