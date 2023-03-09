package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class PositionArmWrist extends CommandBase{
    
    public PositionArmWrist(ArmSubsystem armSubsystem, int armPosition, int wristPosition){
        armSubsystem.setArmWristGoal(armPosition, wristPosition);
    }
}
