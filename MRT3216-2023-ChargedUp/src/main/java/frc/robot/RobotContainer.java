package frc.robot;

// region Imports

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.OI.OIUtils;
import frc.robot.commands.TeleDrive;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.Auto;
import frc.robot.settings.Constants.Drivetrain;
import frc.robot.subsystems.SwerveSubsystem;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

// endregion

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
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
        Timer.delay(1);
        this.driveSystem = SwerveSubsystem.getInstance();
        this.autoChooser = AutoChooser.getInstance();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        if (driveSystem != null && controller != null) {
            driveSystem.setDefaultCommand(new TeleDrive(
                    driveSystem,
                    () -> OIUtils.modifyAxis(-controller.getLeftY(), this.translationExpo)
                            * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                    () -> OIUtils.modifyAxis(-controller.getLeftX(), this.translationExpo)
                            * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                    () -> OIUtils.modifyAxis(-controller.getRightX(), this.rotationExpo)
                            * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    true));
        }

        // Resets the Robots Odometry and Gyro values
        // controller.Y.whenPressed(() ->
        // RobotContainer.getInstance().getDriveSystem().resetGyroAndOdometry(true),
        // driveSystem);
        controller.y().onTrue(runOnce(() -> this.driveSystem.resetGyroAndOdometry(true)));

    }

    public void disablePIDSubsystems() {

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