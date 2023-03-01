// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.settings.Constants.INTAKE.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.RobotMap;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance;
    protected boolean enabled;

    private CANSparkMax motor;

    /** Creates a new IntakeSubsystem. */
    private IntakeSubsystem() {
        motor = new CANSparkMax(RobotMap.ROBOT.INTAKE.MOTOR, MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.clearFaults();

        motor.setInverted(kMotorInverted);
        motor.setSmartCurrentLimit(kMotorCurrentLimit);
        motor.setIdleMode(IdleMode.kBrake);

        motor.burnFlash();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public Command getConeCommand(boolean intake) {
        return Commands.run(() -> {
            motor.set(intake ? kConeIntakeSpeed : kConeOuttakeSpeed);
        }).finallyDo((end) -> motor.set(0));
    }

    public Command getCubeCommand(boolean intake) {
        return Commands.run(() -> {
            motor.set(intake ? kCubeIntakeSpeed : kCubeOuttakeSpeed);
        }).finallyDo((end) -> motor.set(0));
    }

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new IntakeSubsystem();
        }
        return instance;
    }
}