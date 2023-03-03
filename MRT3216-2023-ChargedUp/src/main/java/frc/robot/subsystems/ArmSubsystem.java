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
    private ProfiledPIDController armPidController;
    protected boolean enabled;

    private CANSparkMax leftTopMotor;
    private CANSparkMax leftMiddleMotor;
    private CANSparkMax leftBottomMotor;
    private CANSparkMax rightTopMotor;
    private CANSparkMax rightMiddleMotor;
    private CANSparkMax rightBottomMotor;
    private CANSparkMax leadMotor;
    private CANSparkMax wristMotor;
    private SparkMaxAbsoluteEncoder encoder;

    private final ArmFeedforward wristFeedforward;

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

    private int aHCone = ARM.kHighConeScoringDegrees;
    private int aHCube = ARM.kHighCubeScoringDegrees;
    private int aMCone = ARM.kMidConeScoringDegrees;
    private int aMCube = ARM.kMidCubeScoringDegrees;
    private int aHybrid = ARM.kHybridScoringDegrees;
    private int aGUprightCone = ARM.kGroundIntakeUprightConeDegrees;
    private int aGTippedCone = ARM.kGroundIntakeTippedConeDegrees;
    private int aGCube = ARM.kGroundIntakeUprightCubeDegrees;
    private int aSCone = ARM.kSubstationIntakeConeDegrees;
    private int aSCube = ARM.kSubstationIntakeCubeDegrees;
    private int aStowed = ARM.kStowedDegrees;

    // #endregion

    // #region Wrist Positions

    private int wHCone = WRIST.kHighConeScoringDegrees;
    private int wHCube = WRIST.kHighCubeScoringDegrees;
    private int wMCone = WRIST.kMidConeScoringDegrees;
    private int wMCube = WRIST.kMidCubeScoringDegrees;
    private int wHybrid = WRIST.kHybridScoringDegrees;
    private int wGUprightCone = WRIST.kGroundIntakeUprightConeDegrees;
    private int wGTippedCone = WRIST.kGroundIntakeTippedConeDegrees;
    private int wGCube = WRIST.kGroundIntakeUprightCubeDegrees;
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

        encoder = leadMotor.getAbsoluteEncoder(Type.kDutyCycle);
        leadMotor.getPIDController().setFeedbackDevice(encoder);

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
        System.out.println("Setpoint before reset:" + armPidController.getSetpoint().position);
        System.out.println("Resetting PIDController; current degrees: " + getArmDegrees());
        armPidController.reset(getArmDegrees());
        System.out.println("Setpoint after reset:" + armPidController.getSetpoint().position);
        System.out.println("Initial Goal: " + armPidController.getGoal().position);
        // endregion

        // #region Wrist PID

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

            // double ff = wristFeedforward.calculate(,);
        }
    }

    /** Enables the PID control. Resets the controller. */
    public void enable() {
        enabled = true;
        armPidController.reset(getArmDegrees());
    }

    /** Disables the PID control. Sets output to zero. */
    public void disable() {
        this.enabled = false;
        // This wasn't in ProfiledPIDSubsystem, but seems reasonable
        armPidController.setGoal(getArmDegrees());
        runArmMotors(0);
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

    public static double calculateWristDegreesWrtArm(double nativeUnits) {
        return (nativeUnits - WRIST.kZeroOffset) * WRIST.kScaleFactor;
    }

    public void runWristMotor(double speed) {
        wristMotor.set(speed);
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

    @Log.NumberBar(name = "Arm Encoder", rowIndex = 0, columnIndex = 0, height = 1, width = 1)
    public double getEncoderPosition() {
        return this.encoder.getPosition();
    }

    @Log.NumberBar(name = "Arm Degrees", rowIndex = 1, columnIndex = 0, height = 1, width = 1)
    public double getArmDegrees() {
        return calculateArmDegrees(encoder.getPosition());
    }

    @Log.NumberBar(name = "Goal", rowIndex = 3, columnIndex = 0, height = 1, width = 1)
    public double getGoal() {
        return armPidController.getGoal().position;
    }

    @Log.NumberBar(name = "Setpoint", rowIndex = 2, columnIndex = 0, height = 1, width = 1)
    public double getSetpoint() {
        return armPidController.getSetpoint().position;
    }

    // #region Setters

    @Config.NumberSlider(name = "Arm P", defaultValue = ARM.kArmKp, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 0, columnIndex = 2, height = 1, width = 1)
    public void setArmKp(double armKp) {
        this.armKp = armKp;
    }

    @Config.NumberSlider(name = "Arm I", defaultValue = ARM.kArmKi, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 1, columnIndex = 2, height = 1, width = 1)
    public void setArmKi(double armKi) {
        this.armKi = armKi;
    }

    @Config.NumberSlider(name = "Arm D", defaultValue = ARM.kArmKd, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 2, columnIndex = 2, height = 1, width = 1)
    public void setArmKd(double armKd) {
        this.armKd = armKd;
    }

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

    @Config.NumberSlider(name = "Arm High Cone", defaultValue = ARM.kHighConeScoringDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 4, columnIndex = 2, height = 1, width = 1)
    public void setAHCone(int aHCone) {
        this.aHCone = aHCone;
    }

    @Config.NumberSlider(name = "Arm High Cube", defaultValue = ARM.kHighCubeScoringDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 4, columnIndex = 2, height = 1, width = 1)
    public void setAHCube(int aHCube) {
        this.aHCube = aHCube;
    }

    @Config.NumberSlider(name = "Arm Mid Cone", defaultValue = ARM.kMidConeScoringDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 5, columnIndex = 3, height = 1, width = 1)
    public void setAMCone(int aMCone) {
        this.aMCone = aMCone;
    }

    @Config.NumberSlider(name = "Arm Mid Cube", defaultValue = ARM.kMidCubeScoringDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setAMCube(int aMCube) {
        this.aMCube = aMCube;
    }

    @Config.NumberSlider(name = "Arm Hybrid", defaultValue = ARM.kHybridScoringDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setAHybrid(int aHybrid) {
        this.aHybrid = aHybrid;
    }

    @Config.NumberSlider(name = "Arm Ground Up Cone", defaultValue = ARM.kGroundIntakeUprightConeDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setAGUprightCone(int aGUprightCone) {
        this.aGUprightCone = aGUprightCone;
    }

    @Config.NumberSlider(name = "Arm Ground Down Cone", defaultValue = ARM.kGroundIntakeTippedConeDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setAGTippedCone(int aGTippedCone) {
        this.aGTippedCone = aGTippedCone;
    }

    @Config.NumberSlider(name = "Arm Ground Cube", defaultValue = ARM.kGroundIntakeUprightCubeDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setAGCube(int aGCube) {
        this.aGCube = aGCube;
    }

    @Config.NumberSlider(name = "Arm Sub Cone", defaultValue = ARM.kSubstationIntakeConeDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setASCone(int aSCone) {
        this.aSCone = aSCone;
    }

    @Config.NumberSlider(name = "Arm Sub Cone", defaultValue = ARM.kSubstationIntakeCubeDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setASCube(int aSCube) {
        this.aSCube = aSCube;
    }

    @Config.NumberSlider(name = "Arm Stowed", defaultValue = ARM.kStowedDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setAStowed(int aStowed) {
        this.aStowed = aStowed;
    }

    @Config.NumberSlider(name = "Wrist High Cone", defaultValue = WRIST.kHighConeScoringDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setWHCone(int wHCone) {
        this.wHCone = wHCone;
    }

    @Config.NumberSlider(name = "Wrist High Cube", defaultValue = WRIST.kHighCubeScoringDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setWHCube(int wHCube) {
        this.wHCube = wHCube;
    }

    @Config.NumberSlider(name = "Wrist Mid Cone", defaultValue = WRIST.kMidConeScoringDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setWMCone(int wMCone) {
        this.wMCone = wMCone;
    }

    @Config.NumberSlider(name = "Wrist Mid Cube", defaultValue = WRIST.kMidCubeScoringDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setWMCube(int wMCube) {
        this.wMCube = wMCube;
    }

    @Config.NumberSlider(name = "Wrist Hybrid", defaultValue = WRIST.kHybridScoringDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setWHybrid(int wHybrid) {
        this.wHybrid = wHybrid;
    }

    @Config.NumberSlider(name = "Wrist Ground Up Cone", defaultValue = WRIST.kGroundIntakeUprightConeDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setWGUprightCone(int wGUprightCone) {
        this.wGUprightCone = wGUprightCone;
    }

    @Config.NumberSlider(name = "Wrist Ground Down Cone", defaultValue = WRIST.kGroundIntakeTippedConeDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setWGTippedCone(int wGTippedCone) {
        this.wGTippedCone = wGTippedCone;
    }

    @Config.NumberSlider(name = "Wrist Ground Cube", defaultValue = WRIST.kGroundIntakeUprightCubeDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setWGCube(int wGCube) {
        this.wGCube = wGCube;
    }

    @Config.NumberSlider(name = "Wrist Sub Cone", defaultValue = WRIST.kSubstationIntakeConeDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setWSCone(int wSCone) {
        this.wSCone = wSCone;
    }

    @Config.NumberSlider(name = "Wrist Sub Cube", defaultValue = WRIST.kSubstationIntakeCubeDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setWSCube(int wSCube) {
        this.wSCube = wSCube;
    }

    @Config.NumberSlider(name = "Wrist Stowed", defaultValue = WRIST.kStowedDegrees, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setWStowed(int wStowed) {
        this.wStowed = wStowed;
    }

    // #endregion

    // #endregion
}