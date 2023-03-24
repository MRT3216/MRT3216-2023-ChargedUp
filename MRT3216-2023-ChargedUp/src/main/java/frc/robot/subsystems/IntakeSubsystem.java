// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.settings.Constants.INTAKE.kConeIntakeSpeed;
import static frc.robot.settings.Constants.INTAKE.kConeOuttakeSpeed;
import static frc.robot.settings.Constants.INTAKE.kCubeIntakeSpeed;
import static frc.robot.settings.Constants.INTAKE.kCubeOuttakeSpeed;
import static frc.robot.settings.Constants.INTAKE.kCubeShootSpeed;
import static frc.robot.settings.Constants.INTAKE.kMotorCurrentLimit;
import static frc.robot.settings.Constants.INTAKE.kMotorInverted;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.ARM.GamePiece;
import frc.robot.settings.Constants.ARM.ScoringHeight;
import frc.robot.settings.Constants.AUTO;
import frc.robot.settings.RobotMap;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance;
    protected boolean enabled;
    private CANSparkMax motor;
    private SparkMaxLimitSwitch limitSwitch;
    private WristSubsystem wristSubsystem;

    /** Creates a new IntakeSubsystem. */
    private IntakeSubsystem() {
        this.wristSubsystem = WristSubsystem.getInstance();
        motor = new CANSparkMax(RobotMap.ROBOT.INTAKE.MOTOR, MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.clearFaults();

        motor.setInverted(kMotorInverted);
        motor.setSmartCurrentLimit(kMotorCurrentLimit);
        motor.setIdleMode(IdleMode.kBrake);
        // motor.enableVoltageCompensation(10);

        limitSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        motor.burnFlash();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler
        if (limitSwitch.isPressed()) {
            wristSubsystem.resetWristEncoderPosition();
            // System.out.println("Encoder position reset by limit switch");
        }
    }

    public Command getCommand(boolean intake, GamePiece piece, ScoringHeight height) {
        if (piece == GamePiece.Cone) {
            System.out.println("Running cone intake");
            return getIntakeCommand(intake ? kConeIntakeSpeed : kConeOuttakeSpeed);
        } else {
            System.out.println("Running cube intake");
            if (height == ScoringHeight.Hybrid) {
                return getIntakeCommand(intake ? kCubeIntakeSpeed : kCubeShootSpeed);
            } else {
                return getIntakeCommand(intake ? kCubeIntakeSpeed : kCubeOuttakeSpeed);
            }
        }
    }

    private Command getIntakeCommand(double speed) {
        return Commands.run(() -> {
            motor.set(speed);
        }).finallyDo((end) -> motor.set(0));
    }

    public Command getAutoConeCommand(boolean intake) {
        return new ProxyCommand(
                () -> Commands.run(() -> motor.set(intake ? kConeIntakeSpeed : kConeOuttakeSpeed))
                        .withTimeout(intake ? AUTO.kMaxIntakeTime : AUTO.kMaxOuttakeTime)
                        .finallyDo((end) -> motor.set(0)));
    }

    public Command getEjectConeCommand() {
        return new ProxyCommand(
                () -> Commands.run(() -> motor.set(kConeOuttakeSpeed))
                        .withTimeout(AUTO.kMaxOuttakeTime)
                        .finallyDo((end) -> motor.set(0)));
    }

    public Command getAutoCubeCommand(boolean intake) {
        return new ProxyCommand(
                () -> Commands.run(() -> motor.set(intake ? kCubeIntakeSpeed : kCubeOuttakeSpeed))
                        .withTimeout(intake ? AUTO.kMaxIntakeTime : AUTO.kMaxOuttakeTime)
                        .finallyDo((end) -> motor.set(0)));
    }

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new IntakeSubsystem();
        }
        return instance;
    }
}