package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.ARM;
import frc.robot.settings.Constants.WRIST;
import frc.robot.settings.RobotMap.ROBOT;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class ArmSubsystem extends SubsystemBase implements Loggable {
    // #region Fields

    private static ArmSubsystem instance;
    protected boolean enabled;

    // #region Arm Motors

    private ProfiledPIDController armPidController;
    private CANSparkMax leftTopMotor;
    private CANSparkMax leftMiddleMotor;
    private CANSparkMax leftBottomMotor;
    private CANSparkMax rightTopMotor;
    private CANSparkMax rightMiddleMotor;
    private CANSparkMax rightBottomMotor;
    private CANSparkMax leadMotor;
    private CANSparkMax wristMotor;
    private SparkMaxAbsoluteEncoder armEncoder;

    // #endregion

    // #region Wrist Motors

    private ProfiledPIDController wristPidController;
    private final ArmFeedforward wristFeedforward;
    private SparkMaxAbsoluteEncoder wristEncoder;

    // #endregion

    // #region Arm PID

    private double armKp = ARM.kArmKp;
    private double armKi = ARM.kArmKi;
    private double armKd = ARM.kArmKd;

    // #endregion

    // #region Wrist PID

    private double wristKp = WRIST.kWristKp;
    private double wristKi = WRIST.kWristKi;
    private double wristKd = WRIST.kWristKd;

    // #endregion

    // #region Arm Positions

    private int aHCone = ARM.kScoringHighConeDegrees;
    private int aHCube = ARM.kScoringHighCubeDegrees;
    private int aMCone = ARM.kScoringMidConeDegrees;
    private int aMCube = ARM.kScoringMidCubeDegrees;
    private int aHybrid = ARM.kScoringHybridDegrees;
    private int aGUprightCone = ARM.kGroundIntakeUprightConeDegrees;
    private int aGTippedCone = ARM.kGroundIntakeTippedConeDegrees;
    private int aGCube = ARM.kGroundIntakeCubeDegrees;
    private int aSCone = ARM.kSubstationIntakeConeDegrees;
    private int aSCube = ARM.kSubstationIntakeCubeDegrees;
    private int aStowed = ARM.kStowedDegrees;

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

    // #endregion

    // #endregion

    // #region ArmSubsystem

    private ArmSubsystem() {
        this.enabled = false;

        // #region Arm Motor Initialization

        leftTopMotor = new CANSparkMax(ROBOT.ARM.LEFT_TOP, MotorType.kBrushless);
        leftMiddleMotor = new CANSparkMax(ROBOT.ARM.LEFT_MIDDLE, MotorType.kBrushless);
        leftBottomMotor = new CANSparkMax(ROBOT.ARM.LEFT_BOTTOM, MotorType.kBrushless);

        rightTopMotor = new CANSparkMax(ROBOT.ARM.RIGHT_TOP, MotorType.kBrushless);
        rightMiddleMotor = new CANSparkMax(ROBOT.ARM.RIGHT_MIDDLE, MotorType.kBrushless);
        rightBottomMotor = new CANSparkMax(ROBOT.ARM.RIGHT_BOTTOM, MotorType.kBrushless);

        leadMotor = rightMiddleMotor;
        leadMotor.restoreFactoryDefaults();
        leadMotor.clearFaults();

        leftTopMotor.follow(leadMotor, ARM.kLeftMotorsInverted);
        leftTopMotor.setIdleMode(IdleMode.kBrake);
        leftMiddleMotor.follow(leadMotor, ARM.kLeftMotorsInverted);
        leftMiddleMotor.setIdleMode(IdleMode.kBrake);
        leftBottomMotor.follow(leadMotor, ARM.kLeftMotorsInverted);
        leftBottomMotor.setIdleMode(IdleMode.kBrake);
        rightTopMotor.follow(leadMotor, ARM.kRightMotorsInverted);
        rightTopMotor.setIdleMode(IdleMode.kBrake);
        rightMiddleMotor.setInverted(ARM.kRightMotorsInverted);
        rightMiddleMotor.setIdleMode(IdleMode.kBrake);
        rightBottomMotor.follow(leadMotor, ARM.kRightMotorsInverted);
        rightBottomMotor.setIdleMode(IdleMode.kBrake);

        armEncoder = leadMotor.getAbsoluteEncoder(Type.kDutyCycle);
        leadMotor.getPIDController().setFeedbackDevice(armEncoder);

        leadMotor.setSoftLimit(SoftLimitDirection.kReverse, ARM.kReverseLimit);
        leadMotor.setSoftLimit(SoftLimitDirection.kForward, ARM.kForwardLimit);
        leadMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        leadMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        leadMotor.setSmartCurrentLimit(ARM.kMotorCurrentLimit);

        leadMotor.burnFlash();

        // endregion

        // #region Wrist Motor Initialization

        wristMotor = new CANSparkMax(ROBOT.WRIST.MOTOR, MotorType.kBrushless);
        wristMotor.restoreFactoryDefaults();
        wristMotor.clearFaults();
        wristMotor.setInverted(WRIST.kMotorInverted);
        wristMotor.setIdleMode(IdleMode.kBrake);
        leadMotor.setSmartCurrentLimit(WRIST.kMotorCurrentLimit);
        wristMotor.burnFlash();

        wristFeedforward = new ArmFeedforward(
                WRIST.kWristKs, WRIST.kWristKg, WRIST.kWristKv, WRIST.kWristKa);

        // endregion

        // #region Arm PID

        // The arm ProfiledPIDController
        armPidController = new ProfiledPIDController(
                this.armKp,
                this.armKi,
                this.armKd,
                // The motion profile constraints
                new TrapezoidProfile.Constraints(ARM.kArmMaxVelocity,
                        ARM.kArmMaxAcceleration));

        armPidController.setGoal(getArmDegrees());

        armPidController.setTolerance(ARM.kArmPositionTolerance);// , ARM.kArmVelocityTolerance);
        System.out.println("Arm Setpoint before reset:" + armPidController.getSetpoint().position);
        System.out.println("Arm Resetting PIDController; current degrees: " + getArmDegrees());
        armPidController.reset(getArmDegrees());
        System.out.println("Arm Setpoint after reset:" + armPidController.getSetpoint().position);
        System.out.println("Arm Initial Goal: " + armPidController.getGoal().position);

        // endregion

        // #region Wrist PID

        wristPidController = new ProfiledPIDController(
                this.wristKp,
                this.wristKi,
                this.wristKd,
                // The motion profile constraints
                new TrapezoidProfile.Constraints(WRIST.kWristMaxVelocity,
                        WRIST.kWristMaxAcceleration));

        wristPidController.setGoal(getArmDegrees());

        wristPidController.setTolerance(WRIST.kWristPositionTolerance);
        System.out.println("Wrist Setpoint before reset:" + wristPidController.getSetpoint().position);
        System.out.println("Wrist Resetting PIDController; current degrees: " + getArmDegrees());
        armPidController.reset(getArmDegrees());
        System.out.println("Wrist Setpoint after reset:" + wristPidController.getSetpoint().position);
        System.out.println("Wrist Initial Goal: " + wristPidController.getGoal().position);

        // endregion
    }

    @Override
    public void periodic() {
        if (this.enabled) {
            if (Math.abs(armPidController.getSetpoint().position - getArmDegrees()) < 20) {
                double armPidVoltage = armPidController.calculate(getArmDegrees());
                leadMotor.setVoltage(armPidVoltage);
            } else {
                this.setArmGoal(armPidController.getGoal().position);
            }

            // TODO: Finish this
            // double ff = wristFeedforward.calculate(,);
        }
    }

    /** Enables the PID control. Resets the controller. */
    public void enable() {
        enabled = true;
        armPidController.reset(getArmDegrees());
        // TODO: Check this
        wristPidController.reset(getWristDegrees());
    }

    /** Disables the PID control. Sets output to zero. */
    public void disable() {
        this.enabled = false;
        // This wasn't in ProfiledPIDSubsystem, but seems reasonable
        armPidController.setGoal(getArmDegrees());
        // TODO: Check this
        wristPidController.setGoal(getWristDegrees());
        runArmMotors(0);
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

    public static ArmSubsystem getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new ArmSubsystem();
        }
        return instance;
    }

    public void setArmWristGoal(double armDegrees, double wristDegrees) {
        setArmGoal(armDegrees);
        setWristGoal(wristDegrees);
    }

    public boolean armWristAtGoal() {
        return armAtGoal() && wristAtGoal();
    }

    // #endregion

    // #region Arm

    public void setArmGoal(double degrees) {
        degrees = Math.min(ARM.kForwardLimitDegrees, Math.max(degrees, ARM.kReverseLimitDegrees));
        System.out.println("Goal Degrees: " + degrees);
        armPidController.setGoal(degrees);
    }

    public boolean armAtGoal() {
        return armPidController.getPositionTolerance() >= Math
                .abs(getArmDegrees() - armPidController.getGoal().position);
    }

    public void runArmMotors(double speed) {
        leadMotor.set(speed);
    }

    public static double calculateArmDegrees(double nativeUnits) {
        return (nativeUnits - ARM.kZeroOffset) * ARM.kScaleFactor;
    }

    public void stopArmMotors() {
        leadMotor.stopMotor();
    }

    // #endregion

    // #region Wrist

    public void setWristGoal(double degrees) {
        degrees = Math.min(WRIST.kForwardLimitDegrees, Math.max(degrees, WRIST.kReverseLimitDegrees));
        System.out.println("Goal Degrees: " + degrees);
        wristPidController.setGoal(degrees);
    }

    public boolean wristAtGoal() {
        return wristPidController.getPositionTolerance() >= Math
                .abs(getWristDegrees() - wristPidController.getGoal().position);
    }

    public void runWristMotor(double speed) {
        wristMotor.set(speed);
    }

    public static double calculateWristDegreesWrtArm(double nativeUnits) {
        return (nativeUnits - WRIST.kZeroOffset) * WRIST.kScaleFactor;
    }

    public void stopWristMotors() {
        wristMotor.stopMotor();
    }

    // #endregion

    // #region Command Factories

    public Command getArmGotoCommand(double armDegrees) {
        return Commands.print("Setting goal")
                .andThen(Commands.runOnce(() -> {
                    setArmGoal(armDegrees);
                    this.enable();
                }, this))
                .andThen(Commands.waitUntil(() -> armAtGoal()))
                .andThen(Commands.print("Arm at goal"));
    }

    // #endregion

    // #region Logging

    // #region Arm Position PID Controller Column 0
    // Column 0, Rows 0-3

    @Log.NumberBar(name = "Arm Encoder", rowIndex = 0, columnIndex = 0, height = 1, width = 1)
    public double getEncoderArmPosition() {
        return this.armEncoder.getPosition();
    }

    @Log.NumberBar(name = "Arm Degrees", rowIndex = 1, columnIndex = 0, height = 1, width = 1)
    public double getArmDegrees() {
        return calculateArmDegrees(armEncoder.getPosition());
    }

    @Log.NumberBar(name = "Goal", rowIndex = 2, columnIndex = 0, height = 1, width = 1)
    public double getGoal() {
        return armPidController.getGoal().position;
    }

    @Log.NumberBar(name = "Setpoint", rowIndex = 3, columnIndex = 0, height = 1, width = 1)
    public double getSetpoint() {
        return armPidController.getSetpoint().position;
    }

    // #endregion

    // #region Wrist Position PID Controller Column 1
    // Column 1, Rows 0-3

    @Log.NumberBar(name = "Wrist Encoder", rowIndex = 0, columnIndex = 0, height = 1, width = 1)
    public double getEncoderPosition() {
        return this.wristEncoder.getPosition();
    }

    @Log.NumberBar(name = "Wrist Degrees", rowIndex = 1, columnIndex = 0, height = 1, width = 1)
    public double getWristDegrees() {
        return calculateWristDegreesWrtArm(wristEncoder.getPosition());
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

    // #region ARM PID Setters Column 2
    // Column 2, Rows 0-3

    @Config.NumberSlider(name = "Arm P", defaultValue = ARM.kArmKp, min = 0, max = 130, blockIncrement = 0.01, rowIndex = 0, columnIndex = 2, height = 1, width = 1)
    public void setArmKp(double armKp) {
        this.armKp = armKp;
    }

    @Config.NumberSlider(name = "Arm I", defaultValue = ARM.kArmKi, min = 0, max = 130, blockIncrement = 0.01, rowIndex = 1, columnIndex = 2, height = 1, width = 1)
    public void setArmKi(double armKi) {
        this.armKi = armKi;
    }

    @Config.NumberSlider(name = "Arm D", defaultValue = ARM.kArmKd, min = 0, max = 130, blockIncrement = 0.01, rowIndex = 2, columnIndex = 2, height = 1, width = 1)
    public void setArmKd(double armKd) {
        this.armKd = armKd;
    }

    // #endregion

    // #region Wrist Position PID Column 3
    // Column 3, Row 0-3

    @Config.NumberSlider(name = "Wrist P", defaultValue = WRIST.kWristKp, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 0, columnIndex = 3, height = 1, width = 1)
    public void setWristKp(double wristKp) {
        this.wristKp = wristKp;
    }

    @Config.NumberSlider(name = "Wrist I", defaultValue = WRIST.kWristKi, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 1, columnIndex = 3, height = 1, width = 1)
    public void setWristKi(double wristKi) {
        this.wristKi = wristKi;
    }

    @Config.NumberSlider(name = "Wrist D", defaultValue = WRIST.kWristKd, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 2, columnIndex = 3, height = 1, width = 1)
    public void setWristKd(double wristKd) {
        this.wristKd = wristKd;
    }

    // #endregion

    // #region Arm Scoring Positions Column 4
    // Column 4, Rows 0-4

    @Config.NumberSlider(name = "Arm High Cone", defaultValue = ARM.kScoringHighConeDegrees, min = 0, max = 130, blockIncrement = 1, rowIndex = 0, columnIndex = 4, height = 1, width = 1)
    public void setAHCone(int aHCone) {
        this.aHCone = aHCone;
    }

    @Config.NumberSlider(name = "Arm High Cube", defaultValue = ARM.kScoringHighCubeDegrees, min = 0, max = 130, blockIncrement = 1, rowIndex = 1, columnIndex = 4, height = 1, width = 1)
    public void setAHCube(int aHCube) {
        this.aHCube = aHCube;
    }

    @Config.NumberSlider(name = "Arm Mid Cone", defaultValue = ARM.kScoringMidConeDegrees, min = 0, max = 130, blockIncrement = 1, rowIndex = 2, columnIndex = 4, height = 1, width = 1)
    public void setAMCone(int aMCone) {
        this.aMCone = aMCone;
    }

    @Config.NumberSlider(name = "Arm Mid Cube", defaultValue = ARM.kScoringMidCubeDegrees, min = 0, max = 130, blockIncrement = 1, rowIndex = 3, columnIndex = 4, height = 1, width = 1)
    public void setAMCube(int aMCube) {
        this.aMCube = aMCube;
    }

    @Config.NumberSlider(name = "Arm Hybrid", defaultValue = ARM.kScoringHybridDegrees, min = 0, max = 130, blockIncrement = 1, rowIndex = 4, columnIndex = 4, height = 1, width = 1)
    public void setAHybrid(int aHybrid) {
        this.aHybrid = aHybrid;
    }

    // #endregion

    // #region Wrist Scoring Positions Column 5
    // Column 5, Rows 0-4

    @Config.NumberSlider(name = "Wrist High Cone", defaultValue = WRIST.kScoringHighConeDegrees, min = 0, max = 5, blockIncrement = 1, rowIndex = 0, columnIndex = 5, height = 1, width = 1)
    public void setWHCone(int wHCone) {
        this.wHCone = wHCone;
    }

    @Config.NumberSlider(name = "Wrist High Cube", defaultValue = WRIST.kScoringHighCubeDegrees, min = 0, max = 5, blockIncrement = 1, rowIndex = 1, columnIndex = 5, height = 1, width = 1)
    public void setWHCube(int wHCube) {
        this.wHCube = wHCube;
    }

    @Config.NumberSlider(name = "Wrist Mid Cone", defaultValue = WRIST.kScoringMidConeDegrees, min = 0, max = 5, blockIncrement = 1, rowIndex = 2, columnIndex = 5, height = 1, width = 1)
    public void setWMCone(int wMCone) {
        this.wMCone = wMCone;
    }

    @Config.NumberSlider(name = "Wrist Mid Cube", defaultValue = WRIST.kScoringMidCubeDegrees, min = 0, max = 5, blockIncrement = 1, rowIndex = 3, columnIndex = 5, height = 1, width = 1)
    public void setWMCube(int wMCube) {
        this.wMCube = wMCube;
    }

    @Config.NumberSlider(name = "Wrist Hybrid", defaultValue = WRIST.kScoringHybridDegrees, min = 0, max = 5, blockIncrement = 1, rowIndex = 4, columnIndex = 5, height = 1, width = 1)
    public void setWHybrid(int wHybrid) {
        this.wHybrid = wHybrid;
    }

    // #endregion

    // #region Arm Pickup Positions Column 6
    // Column 6, Rows 0-5

    @Config.NumberSlider(name = "A Ground Up Cone", defaultValue = ARM.kGroundIntakeUprightConeDegrees, min = 0, max = 130, blockIncrement = 1, rowIndex = 0, columnIndex = 6, height = 1, width = 1)
    public void setAGUprightCone(int aGUprightCone) {
        this.aGUprightCone = aGUprightCone;
    }

    @Config.NumberSlider(name = "A Ground Down Cone", defaultValue = ARM.kGroundIntakeTippedConeDegrees, min = 0, max = 130, blockIncrement = 1, rowIndex = 1, columnIndex = 6, height = 1, width = 1)
    public void setAGTippedCone(int aGTippedCone) {
        this.aGTippedCone = aGTippedCone;
    }

    @Config.NumberSlider(name = "A Ground Cube", defaultValue = ARM.kGroundIntakeCubeDegrees, min = 0, max = 130, blockIncrement = 1, rowIndex = 2, columnIndex = 6, height = 1, width = 1)
    public void setAGCube(int aGCube) {
        this.aGCube = aGCube;
    }

    @Config.NumberSlider(name = "A Sub Cone", defaultValue = ARM.kSubstationIntakeConeDegrees, min = 0, max = 130, blockIncrement = 1, rowIndex = 3, columnIndex = 6, height = 1, width = 1)
    public void setASCone(int aSCone) {
        this.aSCone = aSCone;
    }

    @Config.NumberSlider(name = "A Sub Cone", defaultValue = ARM.kSubstationIntakeCubeDegrees, min = 0, max = 130, blockIncrement = 1, rowIndex = 4, columnIndex = 6, height = 1, width = 1)
    public void setASCube(int aSCube) {
        this.aSCube = aSCube;
    }

    // #endregion

    // #region Wrist Pickup Positions Column 7
    // Column 7, Rows

    @Config.NumberSlider(name = "W Ground Up Cone", defaultValue = WRIST.kGroundIntakeUprightConeDegrees, min = 0, max = 5, blockIncrement = 1, rowIndex = 0, columnIndex = 7, height = 1, width = 1)
    public void setWGUprightCone(int wGUprightCone) {
        this.wGUprightCone = wGUprightCone;
    }

    @Config.NumberSlider(name = "W Ground Down Cone", defaultValue = WRIST.kGroundIntakeTippedConeDegrees, min = 0, max = 5, blockIncrement = 1, rowIndex = 1, columnIndex = 7, height = 1, width = 1)
    public void setWGTippedCone(int wGTippedCone) {
        this.wGTippedCone = wGTippedCone;
    }

    @Config.NumberSlider(name = "W Ground Cube", defaultValue = WRIST.kGroundIntakeCubeDegrees, min = 0, max = 5, blockIncrement = 1, rowIndex = 2, columnIndex = 7, height = 1, width = 1)
    public void setWGCube(int wGCube) {
        this.wGCube = wGCube;
    }

    @Config.NumberSlider(name = "W Sub Cone", defaultValue = WRIST.kSubstationIntakeConeDegrees, min = 0, max = 5, blockIncrement = 1, rowIndex = 3, columnIndex = 7, height = 1, width = 1)
    public void setWSCone(int wSCone) {
        this.wSCone = wSCone;
    }

    @Config.NumberSlider(name = "W Sub Cube", defaultValue = WRIST.kSubstationIntakeCubeDegrees, min = 0, max = 5, blockIncrement = 1, rowIndex = 4, columnIndex = 7, height = 1, width = 1)
    public void setWSCube(int wSCube) {
        this.wSCube = wSCube;
    }

    // #endregion

    // #region Stowed Positions Column 8
    // Column 8, Rows 0-1

    @Config.NumberSlider(name = "Arm Stowed", defaultValue = ARM.kStowedDegrees, min = 0, max = 5, blockIncrement = 1, rowIndex = 5, columnIndex = 6, height = 1, width = 1)
    public void setAStowed(int aStowed) {
        this.aStowed = aStowed;
    }

    @Config.NumberSlider(name = "Wrist Stowed", defaultValue = WRIST.kStowedDegrees, min = 0, max = 5, blockIncrement = 1, rowIndex = 3, columnIndex = 5, height = 1, width = 1)
    public void setWStowed(int wStowed) {
        this.wStowed = wStowed;
    }

    // #endregion

    // #endregion
}