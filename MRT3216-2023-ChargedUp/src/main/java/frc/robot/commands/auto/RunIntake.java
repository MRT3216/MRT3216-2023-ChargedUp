package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends CommandBase{
    
    public RunIntake(IntakeSubsystem intakeSubsystem, boolean intake, boolean cone) {
        if (cone) {
            intakeSubsystem.getAutoCubeCommand(intake);
        } else {
            intakeSubsystem.getAutoCubeCommand(intake);
        }
    }
}
