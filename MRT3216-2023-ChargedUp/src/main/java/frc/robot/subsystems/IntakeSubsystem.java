// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.settings.Constants.INTAKE.kConeIntakeSpeed;
import static frc.robot.settings.Constants.INTAKE.kConeOuttakeSpeed;
import static frc.robot.settings.Constants.INTAKE.kCubeIntakeSpeed;
import static frc.robot.settings.Constants.INTAKE.kCubeOuttakeSpeed;
import static frc.robot.settings.Constants.INTAKE.kMotorCurrentLimit;
import static frc.robot.settings.Constants.INTAKE.kMotorInverted;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.settings.RobotMap;
import frc.robot.settings.Constants.Auto;

import frc.robot.settings.Constants;
import frc.robot.settings.RobotMap;
import frc.robot.settings.Constants.ARM.GamePiece;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;


public class IntakeSubsystem extends SubsystemBase implements Loggable {
    private static IntakeSubsystem instance;
    protected boolean enabled;
    private CANSparkMax motor;
    private SparkMaxLimitSwitch limitSwitch;
    private ArmSubsystem armSubsystem;

    /** Creates a new IntakeSubsystem. */
    private IntakeSubsystem() {
        this.armSubsystem = ArmSubsystem.getInstance();
        motor = new CANSparkMax(RobotMap.ROBOT.INTAKE.MOTOR, MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.clearFaults();

        motor.setInverted(kMotorInverted);
        motor.setSmartCurrentLimit(kMotorCurrentLimit);
        motor.setIdleMode(IdleMode.kBrake);

        limitSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        motor.burnFlash();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler
        if (limitSwitch.isPressed()) {
            armSubsystem.resetWristEncoderPosition();
            System.out.println("Encoder position reset by limit switch");
        }
    }

    public Command getCommand(boolean intake, GamePiece piece) {
        if(piece == GamePiece.Cone) {
            System.out.println("Running cone intake");
            return getConeCommand(intake);
        } else {
            System.out.println("Running cube intake");
            return getCubeCommand(intake);
        }
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

    public Command getAutoConeCommand(boolean intake) {
        return Commands.run(() -> {
            motor.set(intake ? kConeIntakeSpeed : kConeOuttakeSpeed);
            new WaitCommand(Auto.kMaxIntakeTime);
        }).finallyDo((end) -> motor.set(0));
    }

    public Command getAutoCubeCommand(boolean intake) {
        return Commands.run(() -> {
            motor.set(intake ? kConeIntakeSpeed : kConeOuttakeSpeed);
            new WaitCommand(Auto.kMaxIntakeTime);
        }).finallyDo((end) -> motor.set(0));
    }

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    @Log.BooleanBox(name = "Wrist Limit Switch", tabName = "ArmSubsystem", rowIndex = 4, columnIndex = 0, height = 1, width = 1)
    public boolean getEncoderArmPosition() {
        return this.limitSwitch.isPressed();
    }
}