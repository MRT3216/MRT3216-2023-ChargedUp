package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.ARM;
import frc.robot.settings.Constants.WRIST;
import frc.robot.settings.RobotMap.ROBOT;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class WristSubsystem extends SubsystemBase implements Loggable {
    // #region Fields

    private static WristSubsystem instance;
    protected boolean enabled;

    // #region Wrist Motors

    private ProfiledPIDController wristPidController;
    private CANSparkMax wristMotor;
    // private SparkMaxAbsoluteEncoder wristEncoder;
    private RelativeEncoder wristEncoderQuad;
    private boolean isWristZeroed;

    // #endregion

    // #region Wrist PID

    private double wristKp = WRIST.kWristKp;
    private double wristKi = WRIST.kWristKi;
    private double wristKd = WRIST.kWristKd;
    private double wristKg = WRIST.kWristKg;
    private double lastSpeed = 0;
    private double lastTime = Timer.getFPGATimestamp();

    // #endregion

    // #region Wrist Positions

    private int wHCone = WRIST.kScoringHighConeDegrees;
    private int wHCube = WRIST.kScoringHighCubeDegrees;
    private int wMCone = WRIST.kScoringMidConeDegrees;
    private int wMCube = WRIST.kScoringMidCubeDegrees;
    private int wHybrid = WRIST.kScoringHybridDegrees;
    private int wGUprightCone = WRIST.kGroundIntakeUprightConeDegrees;
    private int wGTippedCone = WRIST.kGroundIntakeTippedConeDegrees;
    private int wGCube = WRIST.kGroundIntakeCubeDegrees;
    private int wSCone = WRIST.kSubstationIntakeConeDegrees;
    private int wSCube = WRIST.kSubstationIntakeCubeDegrees;
    private int wStowed = WRIST.kStowedDegrees;
    private int wStart = WRIST.kStartDegrees;

    // #endregion

    // #endregion

    // #region WristSubsystem

    private WristSubsystem() {
        this.enabled = false;

        // #region Wrist Motor Initialization

        wristMotor = new CANSparkMax(ROBOT.WRIST.MOTOR, MotorType.kBrushless);
        wristMotor.restoreFactoryDefaults();
        wristMotor.clearFaults();
        wristMotor.setInverted(WRIST.kMotorInverted);
        wristMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setSmartCurrentLimit(WRIST.kMotorCurrentLimit);

        // wristEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
        // wristMotor.getPIDController().setFeedbackDevice(wristEncoder);
        wristEncoderQuad = wristMotor.getAlternateEncoder(8192);
        wristMotor.getPIDController().setFeedbackDevice(wristEncoderQuad);

        // wristMotor.setSoftLimit(SoftLimitDirection.kReverse, WRIST.kReverseLimit);
        // wristMotor.setSoftLimit(SoftLimitDirection.kForward, WRIST.kForwardLimit);
        // wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        // wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        wristMotor.burnFlash();
        // wristFeedforward = new ArmFeedforward(
        // WRIST.kWristKs, this.wristKg, WRIST.kWristKv, WRIST.kWristKa);

        // endregion

        // #region Arm PID

        // #region Wrist PID

        wristPidController = new ProfiledPIDController(
                this.wristKp,
                this.wristKi,
                this.wristKd,
                // The motion profile constraints
                new TrapezoidProfile.Constraints(WRIST.kWristMaxVelocity,
                        WRIST.kWristMaxAcceleration));

        wristPidController.setGoal(getWristDegreesWrtArm());

        wristPidController.setTolerance(WRIST.kWristPositionTolerance);
        System.out.println("Wrist Setpoint before reset:" + wristPidController.getSetpoint().position);
        System.out.println("Wrist Resetting PIDController; current degrees: " + getWristDegreesWrtArm());
        wristPidController.reset(getWristDegreesWrtArm());
        System.out.println("Wrist Setpoint after reset:" + wristPidController.getSetpoint().position);
        System.out.println("Wrist Initial Goal: " + wristPidController.getGoal().position);

        // endregion
    }

    @Override
    public void periodic() {
        if (this.enabled) {
            if (Math.abs(wristPidController.getSetpoint().position - getWristDegreesWrtArm()) < 20) {
                double wristPidVoltage = -wristPidController.calculate(getWristDegreesWrtArm());
                // TODO: Finish this
                // Calculate the acceleration based on the speed at the last time stamp
                // double acceleration = (wristPidController.getSetpoint().velocity - lastSpeed)
                // / (Timer.getFPGATimestamp() - lastTime);
                // Calculate the feedforward based on the current velocity and acceleration
                // double setpoint = calculateWristDegreesWrtGround(getArmDegrees(),
                // wristPidController.getSetpoint().position);
                // double ff = -wristFeedforward.calculate(setpoint,
                // wristPidController.getSetpoint().velocity);
                wristMotor.setVoltage(wristPidVoltage);
                // System.out.println("Wrist: " + wristPidVoltage);
                // System.out.println("FF: " + ff);

                // Save the current speed and time for the next loop
                // lastSpeed = wristPidController.getSetpoint().velocity;
                // lastTime = Timer.getFPGATimestamp();
            } else {
                this.setWristGoal(wristPidController.getGoal().position);
            }
        }
    }

    /** Enables the PID control. Resets the controller. */
    public void enable() {
        enabled = true;
        wristPidController.reset(getWristDegreesWrtArm());
    }

    /** Disables the PID control. Sets output to zero. */
    public void disable() {
        this.enabled = false;
        // This wasn't in ProfiledPIDSubsystem, but seems reasonable
        wristPidController.setGoal(getWristDegreesWrtArm());
        runWristMotor(0);
    }

    /**
     * Returns whether the controller is enabled.
     *
     * @return Whether the controller is enabled.
     */
    public boolean isEnabled() {
        return this.enabled;
    }

    public static WristSubsystem getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new WristSubsystem();
        }
        return instance;
    }

    // #endregion

    // #region Wrist

    public void setWristGoal(double degrees) {
        // (degrees, WRIST.kReverseLimitDegrees));
        // System.out.println("Wrist Goal Degrees: " + degrees);
        wristPidController.setGoal(degrees);
    }

    public boolean wristAtGoal() {
        return wristPidController.getPositionTolerance() >= Math
                .abs(getWristDegreesWrtArm() - wristPidController.getGoal().position);
    }

    public void runWristMotor(double speed) {
        wristMotor.set(speed);
    }

    public static double calculateWristDegreesWrtArm(double nativeUnits) {
        return (nativeUnits - WRIST.kZeroOffset) * WRIST.kScaleFactor;
    }

    public void stopWristMotorAndResetPID() {
        wristMotor.stopMotor();
        wristPidController.reset(getWristDegreesWrtArm());
    }

    public double getWristDegreesByPosition(ARM.Position position) {
        switch (position) {
            case ScoringHighCone:
                return this.wHCone;
            case ScoringHighCube:
                return this.wHCube;
            case ScoringMidCone:
                return this.wMCone;
            case ScoringMidCube:
                return this.wMCube;
            case ScoringHybrid:
                return this.wHybrid;
            case GroundIntakeUprightCone:
                return this.wGUprightCone;
            case GroundIntakeTippedCone:
                return this.wGTippedCone;
            case GroundIntakeCube:
                return this.wGCube;
            case SubstationIntakeCone:
                return this.wSCone;
            case SubstationIntakeCube:
                return this.wSCube;
            case Start:
                return this.wStart;
            // ARM.Positions.Stowed
            default:
                return this.wStowed;
        }
    }

    public void resetWristEncoderPosition() {
        wristEncoderQuad.setPosition(WRIST.kLimitSwitchPosition);

        if (!isWristZeroed) {
            isWristZeroed = true;
        }
    }

    public boolean isWristZeroed() {
        return isWristZeroed;
    }

    // #endregion

    // #region Command Factories

    public Command getWristGotoCommand(double wristDegrees) {
        return Commands.print("Setting wrist goal")
                .andThen(Commands.runOnce(() -> {
                    setWristGoal(wristDegrees);
                    this.enable();
                }, this))
                .andThen(Commands.waitUntil(() -> wristAtGoal()))
                .andThen(Commands.print("Wrist at goal"));
    }

    // #endregion

    // #endregion

    // #region Logging

    // #region Wrist Position PID Controller Column 1
    // Column 1, Rows 0-3

    @Log.NumberBar(name = "Wrist Encoder", tabName = "ArmSubsytem", rowIndex = 0, columnIndex = 1, height = 1, width = 1)
    public double getEncoderPosition() {
        return this.wristEncoderQuad.getPosition();
    }

    @Log.NumberBar(name = "Wrist Degrees Wrt A", tabName = "ArmSubsytem", rowIndex = 1, columnIndex = 1, height = 1, width = 1)
    public double getWristDegreesWrtArm() {
        return calculateWristDegreesWrtArm(wristEncoderQuad.getPosition());
    }

    @Log.NumberBar(name = "Wrist Goal", tabName = "ArmSubsytem", rowIndex = 2, columnIndex = 1, height = 1, width = 1)
    public double getWristGoal() {
        return wristPidController.getGoal().position;
    }

    @Log.NumberBar(name = "Wrist Setpoint", tabName = "ArmSubsytem", rowIndex = 3, columnIndex = 1, height = 1, width = 1)
    public double getWristSetpoint() {
        return wristPidController.getSetpoint().position;
    }

    // #endregion

    // #region Wrist Position PID Column 3
    // Column 3, Row 0-3

    @Config.NumberSlider(name = "Wrist P", tabName = "ArmSubsytem", defaultValue = WRIST.kWristKp, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 0, columnIndex = 3, height = 1, width = 1)
    public void setWristKp(double wristKp) {
        this.wristKp = wristKp;
        resetWristPID();
    }

    @Config.NumberSlider(name = "Wrist I", tabName = "ArmSubsytem", defaultValue = WRIST.kWristKi, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 1, columnIndex = 3, height = 1, width = 1)
    public void setWristKi(double wristKi) {
        this.wristKi = wristKi;
        resetWristPID();
    }

    @Config.NumberSlider(name = "Wrist D", tabName = "ArmSubsytem", defaultValue = WRIST.kWristKd, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 2, columnIndex = 3, height = 1, width = 1)
    public void setWristKd(double wristKd) {
        this.wristKd = wristKd;
        resetWristPID();
    }

    private void resetWristPID() {
        this.wristPidController.setPID(wristKp, wristKi, wristKd);
        System.out.println("Changing wrist P: " + wristKp + "  I: " + wristKi + " D:"
                + wristKd);
    }

    @Config.NumberSlider(name = "Wrist G", tabName = "ArmSubsytem", defaultValue = WRIST.kWristKg, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setWristKg(double wristKg) {
        this.wristKg = wristKg;
        // this.wristFeedforward = new ArmFeedforward(
        // WRIST.kWristKs, this.wristKg, WRIST.kWristKv, WRIST.kWristKa);
    }

    // #endregion

    // #region Wrist Scoring Positions Column 5
    // Column 5, Rows 0-4

    @Config.NumberSlider(name = "Wrist High Cone", tabName = "ArmSubsytem", defaultValue = WRIST.kScoringHighConeDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 0, columnIndex = 5, height = 1, width = 1)
    public void setWHCone(int wHCone) {
        this.wHCone = wHCone;
    }

    @Config.NumberSlider(name = "Wrist High Cube", tabName = "ArmSubsytem", defaultValue = WRIST.kScoringHighCubeDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 1, columnIndex = 5, height = 1, width = 1)
    public void setWHCube(int wHCube) {
        this.wHCube = wHCube;
    }

    @Config.NumberSlider(name = "Wrist Mid Cone", tabName = "ArmSubsytem", defaultValue = WRIST.kScoringMidConeDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 2, columnIndex = 5, height = 1, width = 1)
    public void setWMCone(int wMCone) {
        this.wMCone = wMCone;
    }

    @Config.NumberSlider(name = "Wrist Mid Cube", tabName = "ArmSubsytem", defaultValue = WRIST.kScoringMidCubeDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 3, columnIndex = 5, height = 1, width = 1)
    public void setWMCube(int wMCube) {
        this.wMCube = wMCube;
    }

    @Config.NumberSlider(name = "Wrist Hybrid", tabName = "ArmSubsytem", defaultValue = WRIST.kScoringHybridDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 4, columnIndex = 5, height = 1, width = 1)
    public void setWHybrid(int wHybrid) {
        this.wHybrid = wHybrid;
    }

    // #endregion

    // #region Wrist Pickup Positions Column 7
    // Column 7, Rows

    @Config.NumberSlider(name = "W G Up Cone", tabName = "ArmSubsytem", defaultValue = WRIST.kGroundIntakeUprightConeDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 0, columnIndex = 7, height = 1, width = 1)
    public void setWGUprightCone(int wGUprightCone) {
        this.wGUprightCone = wGUprightCone;
    }

    @Config.NumberSlider(name = "W G Down Cone", tabName = "ArmSubsytem", defaultValue = WRIST.kGroundIntakeTippedConeDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 1, columnIndex = 7, height = 1, width = 1)
    public void setWGTippedCone(int wGTippedCone) {
        this.wGTippedCone = wGTippedCone;
    }

    @Config.NumberSlider(name = "W Ground Cube", tabName = "ArmSubsytem", defaultValue = WRIST.kGroundIntakeCubeDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 2, columnIndex = 7, height = 1, width = 1)
    public void setWGCube(int wGCube) {
        this.wGCube = wGCube;
    }

    @Config.NumberSlider(name = "W Sub Cone", tabName = "ArmSubsytem", defaultValue = WRIST.kSubstationIntakeConeDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 3, columnIndex = 7, height = 1, width = 1)
    public void setWSCone(int wSCone) {
        this.wSCone = wSCone;
    }

    @Config.NumberSlider(name = "W Sub Cube", tabName = "ArmSubsytem", defaultValue = WRIST.kSubstationIntakeCubeDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 4, columnIndex = 7, height = 1, width = 1)
    public void setWSCube(int wSCube) {
        this.wSCube = wSCube;
    }

    // #endregion

    // #region Stowed Positions Column 8
    // Column 8, Rows 0-1

    @Config.NumberSlider(name = "Wrist Stowed", tabName = "ArmSubsytem", defaultValue = WRIST.kStowedDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 1, columnIndex = 8, height = 1, width = 1)
    public void setWStowed(int wStowed) {
        this.wStowed = wStowed;
    }

    // #endregion

    // #endregion
}