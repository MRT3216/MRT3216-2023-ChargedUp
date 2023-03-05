package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.OI.OIUtils;
import frc.robot.commands.TeleDrive;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.Auto;
import frc.robot.settings.Constants.Drivetrain;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

// endregion

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// #region Fields

	private static RobotContainer instance;

	@Log.BooleanBox(name = "Gyro Con.", methodName = "gyroConnected", rowIndex = 1, columnIndex = 3, width = 1, height = 1)
	private SwerveSubsystem driveSystem;

	private AutoChooser autoChooser;
	private double autoStartDelayTime;
	private double translationExpo;
	private double rotationExpo;
	private CommandXboxController controller;
	private ArmSubsystem armSystem;
	private IntakeSubsystem intakeSystem;

	// #endregion

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	private RobotContainer() {
		this.controller = new CommandXboxController(0);
		this.autoStartDelayTime = Constants.Auto.kStartDelayTime;
		this.translationExpo = Constants.OI.kTranslationExpo;
		this.rotationExpo = Constants.OI.kRotationnExpo;

		this.initSubsystems();

		// The first argument is the root container
		// The second argument is whether logging and config should be given separate
		// tabs
		Logger.configureLoggingAndConfig(this, false);
		// Logger.configureLogging(this);

		// Configure the button bindings
		configureButtonBindings();
	}

	public void initSubsystems() {
		this.driveSystem = SwerveSubsystem.getInstance();
		this.autoChooser = AutoChooser.getInstance();
		this.armSystem = ArmSubsystem.getInstance();
		this.intakeSystem = IntakeSubsystem.getInstance();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
	 * it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		if (driveSystem != null && controller != null) {
			driveSystem.setDefaultCommand(
					new TeleDrive(
							driveSystem,
							() -> OIUtils.modifyAxis(-controller.getLeftY(), this.translationExpo)
									* Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
							() -> OIUtils.modifyAxis(-controller.getLeftX(), this.translationExpo)
									* Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
							() -> OIUtils.modifyAxis(controller.getRightX(), this.rotationExpo)
									* Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
							true));
		}

		controller.leftTrigger().whileTrue(
				Commands.run(() -> this.armSystem.runArmMotors(-controller.getLeftTriggerAxis() / 10),
						armSystem)
						.finallyDo(
								(end) -> {
									this.armSystem.stopArmMotors();
									this.armSystem.setArmGoal(this.armSystem.getArmDegrees());
								}));

		controller.rightTrigger().whileTrue(
				Commands.run(() -> this.armSystem.runArmMotors(controller.getRightTriggerAxis() / 10),
						armSystem)
						.finallyDo((end) -> {
							this.armSystem.stopArmMotors();
							this.armSystem.setArmGoal(this.armSystem.getArmDegrees());
						}));

		controller.leftBumper().whileTrue(Commands.run(() -> this.armSystem.runWristMotor(.1)).finallyDo((end) -> {
			this.armSystem.stopWristMotors();
			//this.armSystem.setWristGoal(this.armSystem.getArmDegrees());
		}));

		controller.rightBumper().whileTrue(Commands.run(() -> this.armSystem.runWristMotor(-.1)).finallyDo((end) -> {
			this.armSystem.stopWristMotors();
			//this.armSystem.setWristGoal(this.armSystem.getWristDegreesWrtArmDegrees());
		}));

		controller.a().onTrue(armSystem.getGroundIntakeCommand());
		controller.b().onTrue(armSystem.getGroundTippedConeIntakeCommand());
		controller.x().onTrue(armSystem.getScoringCommand());
		controller.y().onTrue(armSystem.getSubstationIntakeCommand());

		controller.leftStick().onTrue(armSystem.getStowedCommand());

		// Place piece
		controller.leftBumper().whileTrue(intakeSystem.getConeCommand(false));
		// Intake
		controller.rightBumper().whileTrue(intakeSystem.getConeCommand(true));
	}

	public void disablePIDSubsystems() {
		armSystem.disable();
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return this.autoChooser.getAutoCommand();
	}

	public static RobotContainer getInstance() {
		if (instance == null) {
			// if instance is null, initialize
			instance = new RobotContainer();
		}
		return instance;
	}

	public SwerveSubsystem getDriveSystem() {
		return driveSystem;
	}

	@Config(name = "Auto Delay", tabName = "Tuning", methodName = "setStartDelayTime", defaultValueNumeric = Auto.kStartDelayTime, methodTypes = {
			double.class }, rowIndex = 1, columnIndex = 1)
	public void setStartDelayTime(double startDelayTime) {
		this.autoStartDelayTime = startDelayTime;
	}

	public double getAutoStartDelayTime() {
		return this.autoStartDelayTime;
	}

	@Config.NumberSlider(name = "Trans. Expo", tabName = "Tuning", defaultValue = Constants.OI.kTranslationExpo, min = 0, max = 100, blockIncrement = 1, rowIndex = 2, columnIndex = 0, height = 1, width = 1)
	public void setTranslationExpo(double expo) {
		this.translationExpo = expo;
	}

	@Config.NumberSlider(name = "Rotation Expo", tabName = "Tuning", defaultValue = Constants.OI.kRotationnExpo, min = 0, max = 100, blockIncrement = 1, rowIndex = 2, columnIndex = 1, height = 1, width = 1)
	public void setRotationExpo(double expo) {
		this.rotationExpo = expo;
	}
}
