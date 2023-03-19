package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.OI.OIUtils;
import frc.robot.commands.TeleDrive;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.ARM.GamePiece;
import frc.robot.settings.Constants.ARM.ScoringHeight;
import frc.robot.settings.Constants.AUTO;
import frc.robot.settings.Constants.Drivetrain;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;
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
	@Log.BooleanBox(name = "Gyro Con.", tabName = "Driver", methodName = "gyroConnected", rowIndex = 1, columnIndex = 6, width = 1, height = 1)
	private SwerveSubsystem driveSystem;
	private AutoChooser autoChooser;
	private double autoStartDelayTime;
	private double translationExpo;
	private double rotationExpo;
	private CommandXboxController controller;
	private ArmSubsystem armSystem;
	private WristSubsystem wristSubsystem;
	private IntakeSubsystem intakeSystem;

	// #endregion

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	private RobotContainer() {
		this.controller = new CommandXboxController(0);
		this.autoStartDelayTime = Constants.AUTO.kStartDelayTime;
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
		this.wristSubsystem = WristSubsystem.getInstance();
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
							() -> OIUtils.modifyAxis(controller.getLeftY(), this.translationExpo)
									* Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
							() -> OIUtils.modifyAxis(controller.getLeftX(), this.translationExpo)
									* Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
							() -> OIUtils.modifyAxis(-controller.getRightX(), this.rotationExpo)
									* Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
							true));
		}

		controller.leftTrigger().whileTrue(
				Commands.run(() -> this.armSystem.runArmMotors(-controller.getLeftTriggerAxis() / 7),
						armSystem)
						.finallyDo(
								(end) -> {
									this.armSystem.enable();
									this.armSystem.stopArmMotorsAndResetPID();
									this.armSystem.setArmGoal(this.armSystem.getArmDegrees());
								}));

		controller.rightTrigger().whileTrue(
				Commands.run(() -> this.armSystem.runArmMotors(controller.getRightTriggerAxis() / 7),
						armSystem)
						.finallyDo((end) -> {
							this.armSystem.enable();
							this.armSystem.stopArmMotorsAndResetPID();
							this.armSystem.setArmGoal(this.armSystem.getArmDegrees());
						}));

		// controller.leftBumper().whileTrue(Commands.run(() ->
		// this.armSystem.runWristMotor(.1)).finallyDo((end) -> {
		// this.armSystem.enable();
		// this.armSystem.stopWristMotorAndResetPID();
		// this.armSystem.setWristGoal(this.armSystem.getWristDegreesWrtArm());
		// }));

		// controller.rightBumper().whileTrue(Commands.run(() ->
		// this.armSystem.runWristMotor(-.1)).finallyDo((end) -> {
		// this.armSystem.enable();
		// this.armSystem.stopWristMotorAndResetPID();
		// this.armSystem.setWristGoal(this.armSystem.getWristDegreesWrtArm());
		// }));

		//controller.a().onTrue(new ProxyCommand(armSystem::getGroundIntakeCommand));
		controller.a().onTrue(autoChooser.autoBalance()).onFalse(Commands.runOnce(() -> driveSystem.stop(), driveSystem));
		// controller.b().onTrue(new
		// ProxyCommand(armSystem::getGroundTippedConeIntakeCommand));
		controller.b().onTrue(new ProxyCommand(armSystem::getStowedCommand));
		controller.x().onTrue(new ProxyCommand(armSystem::getScoringCommand));
		controller.y().onTrue(new ProxyCommand(armSystem::getSubstationIntakeCommand));
		controller.start().onTrue(new ProxyCommand(armSystem::getStartCommand));

		// controller.rightStick().onTrue(new
		// ProxyCommand(armSystem::getStowedCommand));

		controller.povLeft().onTrue(Commands.runOnce(() -> this.armSystem.setScoringHeight(ScoringHeight.Hybrid)));
		controller.povUp().onTrue(Commands.runOnce(() -> this.armSystem.setScoringHeight(ScoringHeight.Mid)));
		controller.povRight().onTrue(Commands.runOnce(() -> this.armSystem.setScoringHeight(ScoringHeight.High)));
		controller.povDown().onTrue(Commands.runOnce(() -> this.armSystem.toggleGamePiece()));

		controller.leftStick().onTrue(Commands.runOnce(() -> this.armSystem.setGamePiece(GamePiece.Cone)));
		controller.rightStick().onTrue(Commands.runOnce(() -> this.armSystem.setGamePiece(GamePiece.Cube)));

		// Place piece
		controller.leftBumper()
				.whileTrue(new ProxyCommand(
						() -> intakeSystem.getCommand(false, armSystem.getGamePiece(), armSystem.getScoringHeight())));
		// Intake
		controller.rightBumper()
				.whileTrue(new ProxyCommand(
						() -> intakeSystem.getCommand(true, armSystem.getGamePiece(), armSystem.getScoringHeight())));
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

	@Config(name = "Auto Delay", tabName = "Tuning", methodName = "setStartDelayTime", defaultValueNumeric = AUTO.kStartDelayTime, methodTypes = {
			double.class }, rowIndex = 1, columnIndex = 0)
	public void setStartDelayTime(double startDelayTime) {
		this.autoStartDelayTime = startDelayTime;
	}

	public double getAutoStartDelayTime() {
		return this.autoStartDelayTime;
	}

	@Config.NumberSlider(name = "Trans. Expo", tabName = "Tuning", defaultValue = Constants.OI.kTranslationExpo, min = 0, max = 100, blockIncrement = 1, rowIndex = 0, columnIndex = 0, height = 1, width = 1)
	public void setTranslationExpo(double expo) {
		this.translationExpo = expo;
	}

	@Config.NumberSlider(name = "Rotation Expo", tabName = "Tuning", defaultValue = Constants.OI.kRotationnExpo, min = 0, max = 100, blockIncrement = 1, rowIndex = 1, columnIndex = 0, height = 1, width = 1)
	public void setRotationExpo(double expo) {
		this.rotationExpo = expo;
	}

	@Log.BooleanBox(name = "Game Piece", tabName = "Driver", rowIndex = 0, columnIndex = 0, height = 3, width = 3, colorWhenTrue = "#EFBE00", colorWhenFalse = "#7450E8")
	public boolean getGamePiece() {
		return this.armSystem.getGamePiece() == GamePiece.Cone;
	}

	@Log.BooleanBox(name = "High", tabName = "Driver", rowIndex = 0, columnIndex = 3, height = 1, width = 3)
	public boolean isScoringHeightHigh() {
		return this.armSystem.getScoringHeight() == ScoringHeight.High;
	}

	@Log.BooleanBox(name = "Mid", tabName = "Driver", rowIndex = 1, columnIndex = 3, height = 1, width = 3)
	public boolean isScoringHeightMid() {
		return this.armSystem.getScoringHeight() == ScoringHeight.Mid;
	}

	@Log.BooleanBox(name = "Hybrid", tabName = "Driver", rowIndex = 2, columnIndex = 3, height = 1, width = 3)
	public boolean isScoringHeightHybrid() {
		return this.armSystem.getScoringHeight() == ScoringHeight.Hybrid;
	}

	@Log.BooleanBox(name = "Wrist Zeroed", tabName = "Driver", rowIndex = 0, columnIndex = 6, height = 1, width = 1)
	public boolean isWristZeroed() {
		return this.wristSubsystem.isWristZeroed();
	}
}
