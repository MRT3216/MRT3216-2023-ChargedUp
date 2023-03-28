package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;
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
    private RelativeEncoder wristEncoderQuad;
    private boolean isWristZeroed;

    // #endregion

    // #region Wrist Positions

    private int wStowed = WRIST.kStowedDegrees;

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
                WRIST.kWristKp,
                WRIST.kWristKi,
                WRIST.kWristKd,
                // The motion profile constraints
                new TrapezoidProfile.Constraints(WRIST.kWristMaxVelocity,
                        WRIST.kWristMaxAcceleration));

        wristPidController.setGoal(getWristDegreesWrtArm());

        wristPidController.setTolerance(WRIST.kWristPositionPIDTolerane);
        if (Constants.showPrintStatements) {
            System.out.println("Wrist Setpoint before reset:" + wristPidController.getSetpoint().position);
            System.out.println("Wrist Resetting PIDController; current degrees: " + getWristDegreesWrtArm());
        }
        wristPidController.reset(getWristDegreesWrtArm());
        if (Constants.showPrintStatements) {
            System.out.println("Wrist Setpoint after reset:" + wristPidController.getSetpoint().position);
            System.out.println("Wrist Initial Goal: " + wristPidController.getGoal().position);
        }

        // Shuffleboard.getTab("WristSubsystem")
        // .add("Wrist PID", wristPidController)
        // .withSize(2, 2)
        // .withPosition(4, 0);

        // endregion
    }

    @Override
    public void periodic() {
        if (this.enabled) {
            if (Math.abs(wristPidController.getSetpoint().position - getWristDegreesWrtArm()) < 20) {
                double wristPidVoltage = -wristPidController.calculate(getWristDegreesWrtArm());
                wristMotor.setVoltage(wristPidVoltage);

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
        wristPidController.setGoal(degrees);
    }

    public boolean wristWithinLooseTolerance() {
        return WRIST.kWristPositionLooseTolerance >= Math
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
                return WRIST.kScoringHighConeDegrees;
            case ScoringHighCube:
                return WRIST.kScoringHighCubeDegrees;
            case ScoringMidCone:
                return WRIST.kScoringMidConeDegrees;
            case ScoringMidCube:
                return WRIST.kScoringMidCubeDegrees;
            case ScoringHybridCone:
                return WRIST.kScoringHybridConeDegrees;
            case ScoringHybridCube:
                return WRIST.kScoringHybridCubeDegrees;
            case GroundIntakeUprightCone:
                return WRIST.kGroundIntakeUprightConeDegrees;
            case GroundIntakeTippedCone:
                return WRIST.kGroundIntakeTippedConeDegrees;
            case GroundIntakeCube:
                return WRIST.kGroundIntakeCubeDegrees;
            case SingleSubstationIntakeCone:
                return WRIST.kSingleSubstationIntakeConeDegrees;
            case SingleSubstationIntakeCube:
                return WRIST.kSingleSubstationIntakeCubeDegrees;
            case DoubleSubstationIntakeCone:
                return WRIST.kDoubleSubstationIntakeConeDegrees;
            case DoubleSubstationIntakeCube:
                return WRIST.kDoubleSubstationIntakeCubeDegrees;
            case Start:
                return WRIST.kStartDegrees;
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

    // #endregion

    // #region Logging

    // #region Wrist Position PID Controller Column 0
    // Column 0, Rows 0-3

    @Log.NumberBar(name = "Wrist Encoder", rowIndex = 0, columnIndex = 0, height = 1, width = 1)
    public double getEncoderPosition() {
        return this.wristEncoderQuad.getPosition();
    }

    @Log.NumberBar(name = "Wrist Degrees Wrt A", rowIndex = 1, columnIndex = 0, height = 1, width = 1)
    public double getWristDegreesWrtArm() {
        return calculateWristDegreesWrtArm(wristEncoderQuad.getPosition());
    }

    @Log.NumberBar(name = "Wrist Goal", rowIndex = 2, columnIndex = 0, height = 1, width = 1)
    public double getWristGoal() {
        return wristPidController.getGoal().position;
    }

    @Log.NumberBar(name = "Wrist Setpoint", rowIndex = 3, columnIndex = 0, height = 1, width = 1)
    public double getWristSetpoint() {
        return wristPidController.getSetpoint().position;
    }

    // #endregion

    // #region Start and Stowed Positions and Double Substation Column 3
    // Column 3, Rows 0

    @Config(name = "Wrist Stowed/Test", tabName = "ArmSubsystem", defaultValueNumeric = WRIST.kStowedDegrees, rowIndex = 1, columnIndex = 1, height = 1, width = 1)
    public void setWStowed(int wStowed) {
        this.wStowed = wStowed;
    }

    // #endregion

    // #endregion
}