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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;
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
    private NetworkTable streamDeckNT;

    // #region Arm Motors

    private ProfiledPIDController armPidController;
    private CANSparkMax leftTopMotor;
    private CANSparkMax leftMiddleMotor;
    private CANSparkMax leftBottomMotor;
    private CANSparkMax rightTopMotor;
    private CANSparkMax rightMiddleMotor;
    private CANSparkMax rightBottomMotor;
    private CANSparkMax leadMotor;
    private SparkMaxAbsoluteEncoder armEncoder;

    // #endregion

    // #region Wrist Motors

    private ProfiledPIDController wristPidController;
    private ArmFeedforward wristFeedforward;
    private CANSparkMax wristMotor;
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
    private double wristKg = WRIST.kWristKg;
    private double lastSpeed = 0;
    private double lastTime = Timer.getFPGATimestamp();

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
        NetworkTable table = NetworkTableInstance.getDefault().getTable(Constants.StreamDeck.NTtable);
        this.streamDeckNT = table;

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
        armEncoder.setInverted(true);
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
        wristMotor.setSmartCurrentLimit(WRIST.kMotorCurrentLimit);

        wristEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
        wristMotor.getPIDController().setFeedbackDevice(wristEncoder);

        wristMotor.setSoftLimit(SoftLimitDirection.kReverse, WRIST.kReverseLimit);
        wristMotor.setSoftLimit(SoftLimitDirection.kForward, WRIST.kForwardLimit);
        wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        wristMotor.burnFlash();
        wristFeedforward = new ArmFeedforward(
                WRIST.kWristKs, this.wristKg, WRIST.kWristKv, WRIST.kWristKa);

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
            if (Math.abs(armPidController.getSetpoint().position - getArmDegrees()) < 20) {
                double armPidVoltage = -armPidController.calculate(getArmDegrees());
                leadMotor.setVoltage(armPidVoltage);
            } else {
                this.setArmGoal(armPidController.getGoal().position);
            }

            if (Math.abs(wristPidController.getSetpoint().position - getWristDegreesWrtArm()) < 20) {
                double wristPidVoltage = -wristPidController.calculate(getWristDegreesWrtArm());
                // TODO: Finish this
                // Calculate the acceleration based on the speed at the last time stamp
                // double acceleration = (wristPidController.getSetpoint().velocity - lastSpeed)
                // / (Timer.getFPGATimestamp() - lastTime);
                // Calculate the feedforward based on the current velocity and acceleration
                double setpoint = calculateWristDegreesWrtGround(getArmDegrees(),
                        wristPidController.getSetpoint().position);
                double ff = -wristFeedforward.calculate(setpoint, wristPidController.getSetpoint().velocity);
                wristMotor.setVoltage(wristPidVoltage);
                System.out.println("Wrist: " + wristPidVoltage);
                System.out.println("FF: " + ff);

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
        armPidController.reset(getArmDegrees());
        wristPidController.reset(getWristDegreesWrtArm());
    }

    /** Disables the PID control. Sets output to zero. */
    public void disable() {
        this.enabled = false;
        // This wasn't in ProfiledPIDSubsystem, but seems reasonable
        armPidController.setGoal(getArmDegrees());
        wristPidController.setGoal(getWristDegreesWrtArm());
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
        System.out.println("Arm Goal Degrees before min and max: " + degrees);
        degrees = Math.min(ARM.kMaxLimitDegrees, Math.max(degrees, ARM.kMinLimitDegrees));
        System.out.println("Arm Goal Degrees: " + degrees);
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

    public static double calculateWristDegreesWrtGround(double armDegrees, double wristDegrees) {
        return -armDegrees - wristDegrees + 170;
    }

    public void stopArmMotorsAndResetPID() {
        armPidController.reset(getArmDegrees());
        leadMotor.stopMotor();
    }

    public double getArmDegreesByPosition(ARM.Position position) {
        switch (position) {
            case ScoringHighCone:
                return this.aHCone;
            case ScoringHighCube:
                return this.aHCube;
            case ScoringMidCone:
                return this.aMCone;
            case ScoringMidCube:
                return this.aMCube;
            case ScoringHybrid:
                return this.aHybrid;
            case GroundIntakeUprightCone:
                return this.aGUprightCone;
            case GroundIntakeTippedCone:
                return this.aGTippedCone;
            case GroundIntakeCube:
                return this.aGCube;
            case SubstationIntakeCone:
                return this.aSCone;
            case SubstationIntakeCube:
                return this.aSCube;
            // Stowed
            default:
                return this.aStowed;
        }
    }

    // #endregion

    // #region Wrist

    public void setWristGoal(double degrees) {
        degrees = Math.min(WRIST.kForwardLimitDegrees, Math.max(degrees, WRIST.kReverseLimitDegrees));
        System.out.println("Wrist Goal Degrees: " + degrees);
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
            // ARM.Positions.Stowed
            default:
                return this.wStowed;
        }
    }

    // #endregion

    // #region Command Factories

    // Score piece - needs piece and height
    public Command getScoringCommand() {
        return this.getCommand(getArmAndWristScoringPosition());
    }

    // Ground intake - needs piece
    public Command getGroundIntakeCommand() {
        return this.getCommand(getArmAndWristIntakePosition(ARM.IntakePosition.Ground));
    }

    // Ground tipped cone intake - needs nothing
    public Command getGroundTippedConeIntakeCommand() {
        return getArmAndWristGotoCommand(this.aGTippedCone, this.wGTippedCone);
    }

    // Substation pickup -- needs piece
    public Command getSubstationIntakeCommand() {
        return this.getCommand(getArmAndWristIntakePosition(ARM.IntakePosition.Substation));
    }

    public Command getStowedCommand() {
        return getCommand(ARM.Position.Stowed);
    }

    // TODO: Finish getting the wrist position WrtArm to wristpositionWrtGround
    public Command getWristHorizontalCommand(double degrees) {
        return getWristGotoCommand(calculateWristDegreesWrtGround(degrees, degrees));
    }

    private Command getCommand(ARM.Position position) {
        return getArmAndWristGotoCommand(getArmDegreesByPosition(position), getWristDegreesByPosition(position));
    }

    private Command getArmAndWristGotoCommand(double armDegrees, double wristDegrees) {
        return Commands.print("Setting arm goal - arm: " + armDegrees + " wrist: " + wristDegrees)
                .andThen(Commands.runOnce(() -> {
                    setArmGoal(armDegrees);
                    setWristGoal(wristDegrees);
                    this.enable();
                }, this))
                .andThen(Commands.waitUntil(() -> armAtGoal() && wristAtGoal()))
                .andThen(Commands.print("Arm and wrist at goal"));
    }

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

    // #region Determining Arm and Wrist Positions

    private ARM.Position getArmAndWristScoringPosition() {
        ARM.ScoringHeight sH = this.getScoringHeight();
        ARM.GamePiece gP = this.getScoringGamePiece();

        if (sH == ARM.ScoringHeight.High) {
            return gP == ARM.GamePiece.Cone ? ARM.Position.ScoringHighCone : ARM.Position.ScoringHighCube;
        } else if (sH == ARM.ScoringHeight.Mid) {
            return gP == ARM.GamePiece.Cone ? ARM.Position.ScoringMidCone : ARM.Position.ScoringMidCube;
        } else if (sH == ARM.ScoringHeight.Hybrid) {
            return ARM.Position.ScoringHybrid;
        }

        return ARM.Position.Stowed;
    }

    private ARM.Position getArmAndWristIntakePosition(ARM.IntakePosition iP) {
        ARM.GamePiece gP = this.getScoringGamePiece();

        if (iP == ARM.IntakePosition.Ground) {
            return gP == ARM.GamePiece.Cone ? ARM.Position.GroundIntakeUprightCone : ARM.Position.ScoringHighCube;
        } else if (iP == ARM.IntakePosition.Substation) {
            return gP == ARM.GamePiece.Cone ? ARM.Position.SubstationIntakeCone : ARM.Position.SubstationIntakeCone;
        }

        return ARM.Position.Stowed;
    }

    public ARM.ScoringHeight getScoringHeight() {
        return ARM.ScoringHeight.valueOf((int) streamDeckNT.getEntry(Constants.StreamDeck.scoringHeight)
                .getInteger(Constants.ARM.kStowedDegrees));
    }

    public ARM.GamePiece getScoringGamePiece() {
        return ARM.GamePiece.valueOf((int) streamDeckNT.getEntry(Constants.StreamDeck.gamePiece)
                .getInteger(Constants.ARM.kStowedDegrees));
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

    @Log.NumberBar(name = "Wrist Encoder", rowIndex = 0, columnIndex = 1, height = 1, width = 1)
    public double getEncoderPosition() {
        return this.wristEncoder.getPosition();
    }

    @Log.NumberBar(name = "Wrist Degrees Wrt A", rowIndex = 1, columnIndex = 1, height = 1, width = 1)
    public double getWristDegreesWrtArm() {
        return calculateWristDegreesWrtArm(wristEncoder.getPosition());
    }

    @Log.NumberBar(name = "Wrist Degrees Wrt G", rowIndex = 4, columnIndex = 1, height = 1, width = 1)
    public double getWristDegreesWrtGround() {
        return calculateWristDegreesWrtGround(getArmDegrees(), getWristDegreesWrtArm());
    }

    @Log.NumberBar(name = "Wrist Goal", rowIndex = 2, columnIndex = 1, height = 1, width = 1)
    public double getWristGoal() {
        return wristPidController.getGoal().position;
    }

    @Log.NumberBar(name = "Wrist Setpoint", rowIndex = 3, columnIndex = 1, height = 1, width = 1)
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
        this.wristPidController.setPID(wristKp, wristKi, wristKd);
        System.out.println("Changing wrist kp to " + wristKp);
    }

    @Config.NumberSlider(name = "Wrist I", defaultValue = WRIST.kWristKi, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 1, columnIndex = 3, height = 1, width = 1)
    public void setWristKi(double wristKi) {
        this.wristKi = wristKi;
    }

    @Config.NumberSlider(name = "Wrist D", defaultValue = WRIST.kWristKd, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 2, columnIndex = 3, height = 1, width = 1)
    public void setWristKd(double wristKd) {
        this.wristKd = wristKd;
    }

    @Config.NumberSlider(name = "Wrist G", defaultValue = WRIST.kWristKg, min = 0, max = 5, blockIncrement = 0.01, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setWristKg(double wristKg) {
        this.wristKg = wristKg;
        this.wristFeedforward = new ArmFeedforward(
                WRIST.kWristKs, this.wristKg, WRIST.kWristKv, WRIST.kWristKa);
    }

    // #endregion

    // #region Arm Scoring Positions Column 4
    // Column 4, Rows 0-4

    @Config.NumberSlider(name = "Arm High Cone", defaultValue = ARM.kScoringHighConeDegrees, min = 2, max = 130, blockIncrement = 1, rowIndex = 0, columnIndex = 4, height = 1, width = 1)
    public void setAHCone(int aHCone) {
        this.aHCone = aHCone;
    }

    @Config.NumberSlider(name = "Arm High Cube", defaultValue = ARM.kScoringHighCubeDegrees, min = 2, max = 130, blockIncrement = 1, rowIndex = 1, columnIndex = 4, height = 1, width = 1)
    public void setAHCube(int aHCube) {
        this.aHCube = aHCube;
    }

    @Config.NumberSlider(name = "Arm Mid Cone", defaultValue = ARM.kScoringMidConeDegrees, min = 2, max = 130, blockIncrement = 1, rowIndex = 2, columnIndex = 4, height = 1, width = 1)
    public void setAMCone(int aMCone) {
        this.aMCone = aMCone;
    }

    @Config.NumberSlider(name = "Arm Mid Cube", defaultValue = ARM.kScoringMidCubeDegrees, min = 2, max = 130, blockIncrement = 1, rowIndex = 3, columnIndex = 4, height = 1, width = 1)
    public void setAMCube(int aMCube) {
        this.aMCube = aMCube;
    }

    @Config.NumberSlider(name = "Arm Hybrid", defaultValue = ARM.kScoringHybridDegrees, min = 2, max = 130, blockIncrement = 1, rowIndex = 4, columnIndex = 4, height = 1, width = 1)
    public void setAHybrid(int aHybrid) {
        this.aHybrid = aHybrid;
    }

    // #endregion

    // #region Wrist Scoring Positions Column 5
    // Column 5, Rows 0-4

    @Config.NumberSlider(name = "Wrist High Cone", defaultValue = WRIST.kScoringHighConeDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 0, columnIndex = 5, height = 1, width = 1)
    public void setWHCone(int wHCone) {
        this.wHCone = wHCone;
    }

    @Config.NumberSlider(name = "Wrist High Cube", defaultValue = WRIST.kScoringHighCubeDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 1, columnIndex = 5, height = 1, width = 1)
    public void setWHCube(int wHCube) {
        this.wHCube = wHCube;
    }

    @Config.NumberSlider(name = "Wrist Mid Cone", defaultValue = WRIST.kScoringMidConeDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 2, columnIndex = 5, height = 1, width = 1)
    public void setWMCone(int wMCone) {
        this.wMCone = wMCone;
    }

    @Config.NumberSlider(name = "Wrist Mid Cube", defaultValue = WRIST.kScoringMidCubeDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 3, columnIndex = 5, height = 1, width = 1)
    public void setWMCube(int wMCube) {
        this.wMCube = wMCube;
    }

    @Config.NumberSlider(name = "Wrist Hybrid", defaultValue = WRIST.kScoringHybridDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 4, columnIndex = 5, height = 1, width = 1)
    public void setWHybrid(int wHybrid) {
        this.wHybrid = wHybrid;
    }

    // #endregion

    // #region Arm Pickup Positions Column 6
    // Column 6, Rows 0-5

    @Config.NumberSlider(name = "A G Up Cone", defaultValue = ARM.kGroundIntakeUprightConeDegrees, min = 2, max = 130, blockIncrement = 1, rowIndex = 0, columnIndex = 6, height = 1, width = 1)
    public void setAGUprightCone(int aGUprightCone) {
        this.aGUprightCone = aGUprightCone;
    }

    @Config.NumberSlider(name = "A G Down Cone", defaultValue = ARM.kGroundIntakeTippedConeDegrees, min = 2, max = 130, blockIncrement = 1, rowIndex = 1, columnIndex = 6, height = 1, width = 1)
    public void setAGTippedCone(int aGTippedCone) {
        this.aGTippedCone = aGTippedCone;
    }

    @Config.NumberSlider(name = "A Ground Cube", defaultValue = ARM.kGroundIntakeCubeDegrees, min = 2, max = 130, blockIncrement = 1, rowIndex = 2, columnIndex = 6, height = 1, width = 1)
    public void setAGCube(int aGCube) {
        this.aGCube = aGCube;
    }

    @Config.NumberSlider(name = "A Sub Cone", defaultValue = ARM.kSubstationIntakeConeDegrees, min = 2, max = 130, blockIncrement = 1, rowIndex = 3, columnIndex = 6, height = 1, width = 1)
    public void setASCone(int aSCone) {
        this.aSCone = aSCone;
    }

    @Config.NumberSlider(name = "A Sub Cube", defaultValue = ARM.kSubstationIntakeCubeDegrees, min = 2, max = 130, blockIncrement = 1, rowIndex = 4, columnIndex = 6, height = 1, width = 1)
    public void setASCube(int aSCube) {
        this.aSCube = aSCube;
    }

    // #endregion

    // #region Wrist Pickup Positions Column 7
    // Column 7, Rows

    @Config.NumberSlider(name = "W G Up Cone", defaultValue = WRIST.kGroundIntakeUprightConeDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 0, columnIndex = 7, height = 1, width = 1)
    public void setWGUprightCone(int wGUprightCone) {
        this.wGUprightCone = wGUprightCone;
    }

    @Config.NumberSlider(name = "W G Down Cone", defaultValue = WRIST.kGroundIntakeTippedConeDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 1, columnIndex = 7, height = 1, width = 1)
    public void setWGTippedCone(int wGTippedCone) {
        this.wGTippedCone = wGTippedCone;
    }

    @Config.NumberSlider(name = "W Ground Cube", defaultValue = WRIST.kGroundIntakeCubeDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 2, columnIndex = 7, height = 1, width = 1)
    public void setWGCube(int wGCube) {
        this.wGCube = wGCube;
    }

    @Config.NumberSlider(name = "W Sub Cone", defaultValue = WRIST.kSubstationIntakeConeDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 3, columnIndex = 7, height = 1, width = 1)
    public void setWSCone(int wSCone) {
        this.wSCone = wSCone;
    }

    @Config.NumberSlider(name = "W Sub Cube", defaultValue = WRIST.kSubstationIntakeCubeDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 4, columnIndex = 7, height = 1, width = 1)
    public void setWSCube(int wSCube) {
        this.wSCube = wSCube;
    }

    // #endregion

    // #region Stowed Positions Column 8
    // Column 8, Rows 0-1

    @Config.NumberSlider(name = "Arm Stowed", defaultValue = ARM.kStowedDegrees, min = 2, max = 130, blockIncrement = 1, rowIndex = 0, columnIndex = 8, height = 1, width = 1)
    public void setAStowed(int aStowed) {
        this.aStowed = aStowed;
    }

    @Config.NumberSlider(name = "Wrist Stowed", defaultValue = WRIST.kStowedDegrees, min = 2, max = 190, blockIncrement = 1, rowIndex = 1, columnIndex = 8, height = 1, width = 1)
    public void setWStowed(int wStowed) {
        this.wStowed = wStowed;
    }

    @Config.ToggleButton(name = "Cone?", rowIndex = 2, columnIndex = 8, height = 1, width = 1)
    public void setGamePiece(boolean isCone) {

    }

    @Config.NumberSlider(name = "Scoring Height", rowIndex = 3, columnIndex = 8, height = 1, width = 1)
    public void setScoringHeight(int scoringHeight) {

    }

    // #endregion

    // #endregion
}