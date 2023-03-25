package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.ARM;
import frc.robot.settings.Constants.ARM.GamePiece;
import frc.robot.settings.Constants.ARM.ScoringHeight;
import frc.robot.settings.Constants.ARM.Substation;
import frc.robot.settings.RobotMap.ROBOT;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class ArmSubsystem extends SubsystemBase implements Loggable {
    // #region Fields

    private static ArmSubsystem instance;

    @Log.Exclude
    @Config.Exclude
    private WristSubsystem wristSubsystem;
    protected boolean enabled;
    private NetworkTable streamDeckNT;
    private boolean isArmZeroed;

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

    // #region Arm Positions

    private int aHCone = ARM.kScoringHighConeDegrees;
    private int aHCube = ARM.kScoringHighCubeDegrees;
    private int aMCone = ARM.kScoringMidConeDegrees;
    private int aMCube = ARM.kScoringMidCubeDegrees;
    private int aHyCone = ARM.kScoringHybridConeDegrees;
    private int aHyCube = ARM.kScoringHybridCubeDegrees;
    private int aGUprightCone = ARM.kGroundIntakeUprightConeDegrees;
    private int aGTippedCone = ARM.kGroundIntakeTippedConeDegrees;
    private int aGCube = ARM.kGroundIntakeCubeDegrees;
    private int aSSCone = ARM.kSingleSubstationIntakeConeDegrees;
    private int aSSCube = ARM.kSingleSubstationIntakeCubeDegrees;
    private int aDSCone = ARM.kDoubleSubstationIntakeConeDegrees;
    private int aDSCube = ARM.kDoubleSubstationIntakeCubeDegrees;
    private int aStowed = ARM.kStowedDegrees;
    private int aStart = ARM.kStartDegrees;
    private static double armOffset = ARM.kZeroOffsetInDegrees;

    // #endregion

    private GamePiece gp;
    private ScoringHeight sH;
    private Substation sub;

    // #endregion

    // #region ArmSubsystem

    private ArmSubsystem() {
        this.enabled = false;
        this.isArmZeroed = false;
        this.wristSubsystem = WristSubsystem.getInstance();
        NetworkTable table = NetworkTableInstance.getDefault().getTable(Constants.StreamDeck.NTtable);
        this.streamDeckNT = table;

        this.gp = GamePiece.Cube;
        this.sH = ScoringHeight.High;
        this.sub = Substation.Double;

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

        // leadMotor.setSoftLimit(SoftLimitDirection.kReverse, ARM.kReverseLimit);
        // leadMotor.setSoftLimit(SoftLimitDirection.kForward, ARM.kForwardLimit);
        // leadMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        // leadMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        leadMotor.setSmartCurrentLimit(ARM.kMotorCurrentLimit);

        leadMotor.burnFlash();

        // endregion

        // #region Arm PID

        // The arm ProfiledPIDController
        armPidController = new ProfiledPIDController(
                ARM.kArmKp,
                ARM.kArmKi,
                ARM.kArmKd,
                // The motion profile constraints
                new TrapezoidProfile.Constraints(ARM.kArmMaxVelocity,
                        ARM.kArmMaxAcceleration));

        armPidController.setGoal(getArmDegrees());

        armPidController.setTolerance(ARM.kArmPositionTolerance);// , ARM.kArmVelocityTolerance);
        if (Constants.showPrintStatements) {
            System.out.println("Arm Setpoint before reset:" + armPidController.getSetpoint().position);
            System.out.println("Arm Resetting PIDController; current degrees: " + getArmDegrees());
        }
        armPidController.reset(getArmDegrees());
        if (Constants.showPrintStatements) {
            System.out.println("Arm Setpoint after reset:" + armPidController.getSetpoint().position);
            System.out.println("Arm Initial Goal: " + armPidController.getGoal().position);
        }

        // Shuffleboard.getTab("ArmSubsystem")
        // .add("Arm PID", armPidController)
        // .withSize(2, 2)
        // .withPosition(4, 0);

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
        }
    }

    /** Enables the PID control. Resets the controller. */
    /** Also enables the wrist subsystem */
    public void enable() {
        enabled = true;
        armPidController.reset(getArmDegrees());
        wristSubsystem.enable();
    }

    /** Disables the PID control. Sets output to zero. */
    public void disable() {
        this.enabled = false;
        // This wasn't in ProfiledPIDSubsystem, but seems reasonable
        armPidController.setGoal(getArmDegrees());
        wristSubsystem.disable();
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

    public void setArmWristGoal(double armDegrees, double wristDegrees) {
        setArmGoal(armDegrees);
        wristSubsystem.setWristGoal(wristDegrees);
    }

    public boolean armWristAtGoal() {
        return armAtGoal() && wristSubsystem.wristAtGoal();
    }

    // #endregion

    // #region Arm

    public void setArmGoal(double degrees) {
        degrees = Math.min(ARM.kMaxLimitDegrees, Math.max(degrees, ARM.kMinLimitDegrees));
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
        return (nativeUnits - armOffset) * ARM.kScaleFactor;
    }

    public static double calculateNativeUnitsFromDegrees(double degrees) {
        return degrees / ARM.kScaleFactor;
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
            case ScoringHybridCone:
                return this.aHyCone;
            case ScoringHybridCube:
                return this.aHyCube;
            case GroundIntakeUprightCone:
                return this.aGUprightCone;
            case GroundIntakeTippedCone:
                return this.aGTippedCone;
            case GroundIntakeCube:
                return this.aGCube;
            case SingleSubstationIntakeCone:
                return this.aSSCone;
            case SingleSubstationIntakeCube:
                return this.aSSCube;
            case DoubleSubstationIntakeCone:
                return this.aDSCone;
            case DoubleSubstationIntakeCube:
                return this.aDSCube;
            case Start:
                return this.aStart;
            // Stowed
            default:
                return this.aStowed;
        }
    }

    public void resetArmEncoderPosition() {
        armEncoder.setZeroOffset(ARM.kLimitSwitchOffset);

        if (!isArmZeroed) {
            isArmZeroed = true;
        }
    }

    public boolean isArmZeroed() {
        return isArmZeroed;
    }

    // #endregion

    // #region Command Factories

    // Score piece - needs piece and height
    public Command getScoringCommand() {
        return this.getCommand(getArmAndWristScoringPosition());
    }

    // Ground intake - needs piece
    public Command getGroundIntakeCommand() {
        return this.getCommand(getArmAndWristIntakePosition(ARM.IntakePosition.Ground), false);
    }

    // // Ground tipped cone intake - needs nothing
    public Command getGroundTippedConeIntakeCommand() {
        return this.getCommand(ARM.Position.GroundIntakeTippedCone, false);
    }

    // Substation pickup -- needs piece
    public Command getSubstationIntakeCommand() {
        if (this.sub == Substation.Single) {
            return this.getCommand(getArmAndWristIntakePosition(ARM.IntakePosition.SingleSubstation));
        } else {
            return this.getCommand(getArmAndWristIntakePosition(ARM.IntakePosition.DoubleSubstation));
        }
    }

    public Command getStowedCommand() {
        return getCommand(ARM.Position.Stowed, false);
    }

    public Command getStartCommand() {
        return getCommand(ARM.Position.Start);
    }

    public Command getCommand(ARM.Position position) {
        return getCommand(position, true);
    }

    public Command getCommand(ARM.Position position, boolean wait) {
        return getArmAndWristGotoCommand(
                getArmDegreesByPosition(position),
                wristSubsystem.getWristDegreesByPosition(position), wait);
    }

    private Command getArmAndWristGotoCommand(double armDegrees, double wristDegrees, boolean wait) {
        return new ProxyCommand(
                () -> Commands.print("Setting arm goal - arm: " + armDegrees + " wrist: " + wristDegrees)
                        .andThen(Commands.runOnce(() -> {
                            setArmGoal(armDegrees);
                            wristSubsystem.setWristGoal(wristDegrees);
                            this.enable();
                        }, this),
                                Commands.waitUntil(() -> (armAtGoal() && wristSubsystem.wristAtGoal()))
                                        .unless(() -> !wait),
                                Commands.print("Arm and wrist at goal")));
    }

    public Command getWristGotoCommand(double wristDegrees) {
        return Commands.print("Setting wrist goal")
                .andThen(Commands.runOnce(() -> {
                    wristSubsystem.setWristGoal(wristDegrees);
                    this.enable();
                }, this))
                .andThen(Commands.waitUntil(() -> wristSubsystem.wristAtGoal()))
                .andThen(Commands.print("Wrist at goal"));
    }

    // #endregion

    // #region Determining Arm and Wrist Positions

    private ARM.Position getArmAndWristScoringPosition() {
        ARM.ScoringHeight sH = this.getScoringHeight();
        ARM.GamePiece gP = this.getGamePiece();

        if (sH == ARM.ScoringHeight.High) {
            return gP == ARM.GamePiece.Cone ? ARM.Position.ScoringHighCone : ARM.Position.ScoringHighCube;
        } else if (sH == ARM.ScoringHeight.Mid) {
            return gP == ARM.GamePiece.Cone ? ARM.Position.ScoringMidCone : ARM.Position.ScoringMidCube;
        } else if (sH == ARM.ScoringHeight.Hybrid) {
            return gP == ARM.GamePiece.Cone ? ARM.Position.ScoringHybridCone : ARM.Position.ScoringHybridCube;
        }

        return ARM.Position.Stowed;
    }

    public ARM.Position getArmAndWristIntakePosition(ARM.IntakePosition iP) {
        ARM.GamePiece gP = this.getGamePiece();

        if (iP == ARM.IntakePosition.Ground) {
            return gP == ARM.GamePiece.Cone ? ARM.Position.GroundIntakeUprightCone : ARM.Position.GroundIntakeCube;
        } else if (iP == ARM.IntakePosition.SingleSubstation) {
            return gP == ARM.GamePiece.Cone ? ARM.Position.SingleSubstationIntakeCone
                    : ARM.Position.SingleSubstationIntakeCube;
        } else if (iP == ARM.IntakePosition.DoubleSubstation) {
            return gP == ARM.GamePiece.Cone ? ARM.Position.DoubleSubstationIntakeCone
                    : ARM.Position.DoubleSubstationIntakeCube;
        }

        return ARM.Position.Stowed;
    }

    public ARM.ScoringHeight getScoringHeight() {
        /*
         * NetworkTableEntry entry =
         * streamDeckNT.getEntry(Constants.StreamDeck.scoringHeight);
         * String scoringHeightNTEntry = entry.getString("nothing").trim();
         * 
         * if (!"nothing".equals(scoringHeightNTEntry)) {
         * this.sH = ARM.ScoringHeight.valueOf((int)
         * entry.getInteger(Constants.ARM.kStowedDegrees));
         * }
         */
        return this.sH;
    }

    public ARM.GamePiece getGamePiece() {
        /*
         * NetworkTableEntry entry =
         * streamDeckNT.getEntry(Constants.StreamDeck.gamePiece);
         * String gamePieceNTEntry = entry.getString("nothing").trim();
         * 
         * if (!"nothing".equals(gamePieceNTEntry.trim())) {
         * this.gp = ARM.GamePiece.valueOf((int)
         * entry.getInteger(Constants.ARM.kStowedDegrees));
         * }
         */
        return this.gp;
    }

    public void setScoringHeight(ScoringHeight sH) {
        System.out.println("Setting scoring height to " + sH);
        this.sH = sH;
    }

    public void setGamePiece(GamePiece gp) {
        this.gp = gp;
    }

    public void toggleGamePiece() {
        if (this.gp == GamePiece.Cone) {
            this.gp = GamePiece.Cube;
        } else {
            this.gp = GamePiece.Cone;
        }

        System.out.println("Setting game piece to " + gp);
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

    // #region Arm Scoring Positions Column 1
    // Column 1, Rows 0-5

    @Config(name = "Arm High Cone", defaultValueNumeric = ARM.kScoringHighConeDegrees, rowIndex = 0, columnIndex = 1, height = 1, width = 1)
    public void setAHCone(int aHCone) {
        this.aHCone = aHCone;
    }

    @Config(name = "Arm High Cube", defaultValueNumeric = ARM.kScoringHighCubeDegrees, rowIndex = 1, columnIndex = 1, height = 1, width = 1)
    public void setAHCube(int aHCube) {
        this.aHCube = aHCube;
    }

    @Config(name = "Arm Mid Cone", defaultValueNumeric = ARM.kScoringMidConeDegrees, rowIndex = 2, columnIndex = 1, height = 1, width = 1)
    public void setAMCone(int aMCone) {
        this.aMCone = aMCone;
    }

    @Config(name = "Arm Mid Cube", defaultValueNumeric = ARM.kScoringMidCubeDegrees, rowIndex = 3, columnIndex = 1, height = 1, width = 1)
    public void setAMCube(int aMCube) {
        this.aMCube = aMCube;
    }

    @Config(name = "A Hybrid Cone", defaultValueNumeric = ARM.kScoringHybridConeDegrees, rowIndex = 4, columnIndex = 1, height = 1, width = 1)
    public void setAHyCone(int aHyCone) {
        this.aHyCone = aHyCone;
    }

    @Config(name = "A Hybrid Cube", defaultValueNumeric = ARM.kScoringHybridCubeDegrees, rowIndex = 5, columnIndex = 1, height = 1, width = 1)
    public void setAHyCube(int aHyCube) {
        this.aHyCube = aHyCube;
    }

    // #endregion

    // #region Arm Pickup Positions Column 2
    // Column 2, Rows 0-4

    @Config(name = "A G Up Cone", defaultValueNumeric = ARM.kGroundIntakeUprightConeDegrees, rowIndex = 0, columnIndex = 2, height = 1, width = 1)
    public void setAGUprightCone(int aGUprightCone) {
        this.aGUprightCone = aGUprightCone;
    }

    @Config(name = "A G Down Cone", defaultValueNumeric = ARM.kGroundIntakeTippedConeDegrees, rowIndex = 1, columnIndex = 2, height = 1, width = 1)
    public void setAGTippedCone(int aGTippedCone) {
        this.aGTippedCone = aGTippedCone;
    }

    @Config(name = "A Ground Cube", defaultValueNumeric = ARM.kGroundIntakeCubeDegrees, rowIndex = 2, columnIndex = 2, height = 1, width = 1)
    public void setAGCube(int aGCube) {
        this.aGCube = aGCube;
    }

    @Config(name = "A SSub Cone", defaultValueNumeric = ARM.kSingleSubstationIntakeConeDegrees, rowIndex = 3, columnIndex = 2, height = 1, width = 1)
    public void setASSCone(int aSSCone) {
        this.aSSCone = aSSCone;
    }

    @Config(name = "A SSub Cube", defaultValueNumeric = ARM.kSingleSubstationIntakeCubeDegrees, rowIndex = 4, columnIndex = 2, height = 1, width = 1)
    public void setASSCube(int aSSCube) {
        this.aSSCube = aSSCube;
    }

    // #endregion

    // #region Start and Stowed Positions and Double Substation Column 3
    // Column 3, Rows 0-1, 3-4

    @Config(name = "Arm Start", defaultValueNumeric = ARM.kStartDegrees, rowIndex = 0, columnIndex = 3, height = 1, width = 1)
    public void setAStart(int aStart) {
        this.aStart = aStart;
    }

    @Config(name = "Arm Stowed", defaultValueNumeric = ARM.kStowedDegrees, rowIndex = 1, columnIndex = 3, height = 1, width = 1)
    public void setAStowed(int aStowed) {
        this.aStowed = aStowed;
    }

    @Config(name = "A DSub Cone", defaultValueNumeric = ARM.kDoubleSubstationIntakeConeDegrees, rowIndex = 3, columnIndex = 3, height = 1, width = 1)
    public void setADSCone(int aDSCone) {
        this.aDSCone = aDSCone;
    }

    @Config(name = "A DSub Cube", defaultValueNumeric = ARM.kDoubleSubstationIntakeCubeDegrees, rowIndex = 4, columnIndex = 3, height = 1, width = 1)
    public void setADSCube(int aDSCube) {
        this.aDSCube = aDSCube;
    }

    // #endregion

    // #region Driver

    @Config(name = "Arm Offset", tabName = "Driver", defaultValueNumeric = ARM.kZeroOffsetInDegrees, rowIndex = 2, columnIndex = 6, height = 1, width = 1)
    public void setArmOffsetInDegrees(double offset) {
        armOffset = calculateNativeUnitsFromDegrees(offset);
    }

    @Config.ToggleSwitch(name = "Double Substation?", tabName = "Driver", defaultValue = true, rowIndex = 1, columnIndex = 7, height = 1, width = 2)
    public void setSubstation(boolean isDouble) {
        this.sub = isDouble ? ARM.Substation.Double : ARM.Substation.Single;
    }

    // #endregion

    // #endregion
}