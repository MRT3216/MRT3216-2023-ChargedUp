package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.ARM;
import frc.robot.settings.RobotMap.ROBOT;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ArmSubsystem extends SubsystemBase implements Loggable {
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
    private SparkMaxAbsoluteEncoder encoder;

    public ArmSubsystem() {
        this.enabled = false;

        leftTopMotor = new CANSparkMax(ROBOT.ARM.LEFT_TOP, MotorType.kBrushless);
        leftMiddleMotor = new CANSparkMax(ROBOT.ARM.LEFT_MIDDLE, MotorType.kBrushless);
        leftBottomMotor = new CANSparkMax(ROBOT.ARM.LEFT_BOTTOM, MotorType.kBrushless);

        rightTopMotor = new CANSparkMax(ROBOT.ARM.RIGHT_TOP, MotorType.kBrushless);
        rightMiddleMotor = new CANSparkMax(ROBOT.ARM.RIGHT_MIDDLE, MotorType.kBrushless);
        rightBottomMotor = new CANSparkMax(ROBOT.ARM.RIGHT_BOTTOM, MotorType.kBrushless);

        leadMotor = rightBottomMotor;
        leadMotor.restoreFactoryDefaults();
        leadMotor.clearFaults();

        leftTopMotor.follow(leadMotor, ARM.kLeftMotorsInverted);
        leftMiddleMotor.follow(leadMotor, ARM.kLeftMotorsInverted);
        leftBottomMotor.follow(leadMotor, ARM.kLeftMotorsInverted);
        rightTopMotor.follow(leadMotor, ARM.kRightMotorsInverted);
        rightMiddleMotor.follow(leadMotor, ARM.kRightMotorsInverted);
        rightBottomMotor.setInverted(ARM.kRightMotorsInverted);

        encoder = leadMotor.getAbsoluteEncoder(Type.kDutyCycle);
        leadMotor.getPIDController().setFeedbackDevice(encoder);

        leadMotor.setSoftLimit(SoftLimitDirection.kReverse, ARM.kReverseLimit);
        leadMotor.setSoftLimit(SoftLimitDirection.kForward, ARM.kForwardLimit);
        leadMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        leadMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        leadMotor.setSmartCurrentLimit(ARM.kMotorCurrentLimit);

        leadMotor.burnFlash();

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
        System.out.println("Setpoint before reset:" + armPidController.getSetpoint().position);
        System.out.println("Resetting PIDController; current degrees: " + getArmDegrees());
        armPidController.reset(getArmDegrees());
        System.out.println("Setpoint after reset:" + armPidController.getSetpoint().position);
        System.out.println("Initial Goal: " + armPidController.getGoal().position);
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
        }
    }

    public Command getGotoCommand(double armDegrees) {
        return Commands.print("Setting goal")
                .andThen(Commands.runOnce(() -> {
                    setArmGoal(armDegrees);
                    this.enable();
                }, this))
                .andThen(Commands.waitUntil(() -> armAtGoal()))
                .andThen(Commands.print("Arm at goal"));
    }

    @Log.NumberBar(name = "Arm Degrees", rowIndex = 0, columnIndex = 1, height = 1, width = 1)
    public double getArmDegrees() {
        return calculateArmDegrees(encoder.getPosition());
    }

    public static double calculateArmDegrees(double nativeUnits) {
        return (nativeUnits - ARM.kZeroOffset) * ARM.kScaleFactor;
    }

    public void setArmGoal(double degrees) {
        degrees = Math.min(ARM.kForwardLimitDegrees, Math.max(degrees, ARM.kReverseLimitDegrees));
        System.out.println("Goal Degrees: " + degrees);
        armPidController.setGoal(degrees);
    }

    public boolean armAtGoal() {
        return armPidController.getPositionTolerance() >= Math
                .abs(getArmDegrees() - armPidController.getGoal().position);
    }

    public void runMotors(double speed) {
        leadMotor.set(speed);
    }

    public void stop() {
        leadMotor.stopMotor();
    }

    @Log.NumberBar(name = "Arm Encoder", rowIndex = 0, columnIndex = 0, height = 1, width = 1)
    public double getEncoderPosition() {
        return this.encoder.getPosition();
    }

    @Log.NumberBar(name = "Lead Current", rowIndex = 0, columnIndex = 2, height = 1, width = 1)
    public double getCurrent() {
        return this.leadMotor.getOutputCurrent();
    }

    @Log.NumberBar(name = "Goal", rowIndex = 1, columnIndex = 0, height = 1, width = 1)
    public double getGoal() {
        return armPidController.getGoal().position;
    }

    @Log.NumberBar(name = "Setpoint", rowIndex = 1, columnIndex = 1, height = 1, width = 1)
    public double getSetpoint() {
        return armPidController.getSetpoint().position;
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
        runMotors(0);
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
}