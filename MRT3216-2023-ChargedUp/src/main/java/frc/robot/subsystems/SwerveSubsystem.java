// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.settings.Constants.Drivetrain.LEFT_FRONT_STEER_OFFSET;
import static frc.robot.settings.Constants.Drivetrain.LEFT_REAR_STEER_OFFSET;
import static frc.robot.settings.Constants.Drivetrain.RIGHT_FRONT_STEER_OFFSET;
import static frc.robot.settings.Constants.Drivetrain.RIGHT_REAR_STEER_OFFSET;
import static frc.robot.settings.Constants.Drivetrain.TRACKWIDTH_METERS;
import static frc.robot.settings.Constants.Drivetrain.WHEELBASE_METERS;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.LEFT_FRONT_ANGLE;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.LEFT_FRONT_CANCODER;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.LEFT_FRONT_DRIVE;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.LEFT_REAR_ANGLE;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.LEFT_REAR_CANCODER;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.LEFT_REAR_DRIVE;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.RIGHT_FRONT_ANGLE;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.RIGHT_FRONT_CANCODER;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.RIGHT_FRONT_DRIVE;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.RIGHT_REAR_ANGLE;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.RIGHT_REAR_CANCODER;
import static frc.robot.settings.RobotMap.ROBOT.DRIVETRAIN.RIGHT_REAR_DRIVE;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.Drivetrain;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class SwerveSubsystem extends SubsystemBase implements Loggable {
	private static SwerveSubsystem instance;
	private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			// Front left
			new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
			// Front right
			new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
			// Back left
			new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
			// Back right
			new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0));

	// connected over USB
	private final AHRS navx;

	// These are our modules. We initialize them in the constructor.
	private final SwerveModule frontLeftModule;
	private final SwerveModule frontRightModule;
	private final SwerveModule backLeftModule;
	private final SwerveModule backRightModule;
	private final SwerveModule[] swerveModules;

	public final SwerveDrivePoseEstimator poseEstimator;
	// public final Field2d field2d;

	private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	private SwerveSubsystem() {
		navx = new AHRS(SerialPort.Port.kUSB1);

		// ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
		MkModuleConfiguration moduleConfig = MkModuleConfiguration.getDefaultSteerNEO();
		moduleConfig.setDriveCurrentLimit(40.0);
		moduleConfig.setSteerCurrentLimit(30.0);

		this.frontLeftModule = new MkSwerveModuleBuilder(moduleConfig)
				// .withLayout(
				// shuffleboardTab
				// .getLayout("Front Left Module", BuiltInLayouts.kList)
				// .withSize(2, 4)
				// .withPosition(0, 0))
				.withGearRatio(SdsModuleConfigurations.MK4I_L2)
				.withDriveMotor(MotorType.NEO, LEFT_FRONT_DRIVE)
				.withSteerMotor(MotorType.NEO, LEFT_FRONT_ANGLE)
				.withSteerEncoderPort(LEFT_FRONT_CANCODER)
				.withSteerOffset(LEFT_FRONT_STEER_OFFSET)
				.build();

		this.frontRightModule = new MkSwerveModuleBuilder()
				// .withLayout(
				// shuffleboardTab
				// .getLayout("Front Right Module", BuiltInLayouts.kList)
				// .withSize(2, 4)
				// .withPosition(2, 0))
				.withGearRatio(SdsModuleConfigurations.MK4I_L2)
				.withDriveMotor(MotorType.NEO, RIGHT_FRONT_DRIVE)
				.withSteerMotor(MotorType.NEO, RIGHT_FRONT_ANGLE)
				.withSteerEncoderPort(RIGHT_FRONT_CANCODER)
				.withSteerOffset(RIGHT_FRONT_STEER_OFFSET)
				.build();

		this.backLeftModule = new MkSwerveModuleBuilder()
				// .withLayout(
				// shuffleboardTab
				// .getLayout("Back Left Module", BuiltInLayouts.kList)
				// .withSize(2, 4)
				// .withPosition(6, 0))
				.withGearRatio(SdsModuleConfigurations.MK4I_L2)
				.withDriveMotor(MotorType.NEO, LEFT_REAR_DRIVE)
				.withSteerMotor(MotorType.NEO, LEFT_REAR_ANGLE)
				.withSteerEncoderPort(LEFT_REAR_CANCODER)
				.withSteerOffset(LEFT_REAR_STEER_OFFSET)
				.build();

		this.backRightModule = new MkSwerveModuleBuilder()
				// .withLayout(
				// shuffleboardTab
				// .getLayout("Back Right Module", BuiltInLayouts.kList)
				// .withSize(2, 4)
				// .withPosition(4, 0))
				.withGearRatio(SdsModuleConfigurations.MK4I_L2)
				.withDriveMotor(MotorType.NEO, RIGHT_REAR_DRIVE)
				.withSteerMotor(MotorType.NEO, RIGHT_REAR_ANGLE)
				.withSteerEncoderPort(RIGHT_REAR_CANCODER)
				.withSteerOffset(RIGHT_REAR_STEER_OFFSET)
				.build();

		this.swerveModules = new SwerveModule[] {
				this.frontLeftModule, this.frontRightModule, this.backLeftModule, this.backRightModule
		};

		this.poseEstimator = new SwerveDrivePoseEstimator(
				kinematics, getGyroscopeRotation(), getPositions(), new Pose2d());

		// this.field2d = new Field2d();
	}

	@Override
	public void periodic() {
		this.poseEstimator.update(getGyroscopeRotation(), getPositions());
		// this.field2d.setRobotPose(poseEstimator.getEstimatedPosition());

		final double zeroDeadzone = 0.001;

		// Set deadzone on translation
		if (Math.abs(this.chassisSpeeds.vxMetersPerSecond) < zeroDeadzone) {
			this.chassisSpeeds.vxMetersPerSecond = 0;
		}
		if (Math.abs(this.chassisSpeeds.vyMetersPerSecond) < zeroDeadzone) {
			this.chassisSpeeds.vyMetersPerSecond = 0;
		}

		// Hockey-lock if stopped by setting rotation to realllly low number
		/*
		 * if (this.chassisSpeeds.vxMetersPerSecond == 0 &&
		 * this.chassisSpeeds.vyMetersPerSecond == 0 &&
		 * Math.abs(this.chassisSpeeds.omegaRadiansPerSecond) < zeroDeadzone) {
		 * this.chassisSpeeds.omegaRadiansPerSecond = 0.00001;
		 * }
		 */

		SwerveModuleState[] states = this.kinematics.toSwerveModuleStates(this.chassisSpeeds);

		setModuleStates(states);
	}

	public void setModuleStates(SwerveModuleState[] states) {
		SwerveDriveKinematics.desaturateWheelSpeeds(states, Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);

		SwerveModulePosition[] positions = getPositions();
		for (int i = 0; i < states.length; i++) {
			states[i] = SwerveModuleState.optimize(states[i], positions[i].angle);
		}

		double flVoltage = states[0].speedMetersPerSecond;
		double frVoltage = states[1].speedMetersPerSecond;
		double blVoltage = states[2].speedMetersPerSecond;
		double brVoltage = states[3].speedMetersPerSecond;

		flVoltage = flVoltage / Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Drivetrain.MAX_VOLTAGE;
		frVoltage = frVoltage / Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Drivetrain.MAX_VOLTAGE;
		blVoltage = blVoltage / Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Drivetrain.MAX_VOLTAGE;
		brVoltage = brVoltage / Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Drivetrain.MAX_VOLTAGE;

		this.frontLeftModule.set(flVoltage, states[0].angle.getRadians());
		this.frontRightModule.set(frVoltage, states[1].angle.getRadians());
		this.backLeftModule.set(blVoltage, states[2].angle.getRadians());
		this.backRightModule.set(brVoltage, states[3].angle.getRadians());
	}

	public void setModuleStatesStraight(){
		this.frontLeftModule.set(0, 0);
		this.frontRightModule.set(0, 0);
		this.backLeftModule.set(0, 0);
		this.backRightModule.set(0, 0);
	}

	/**
	 * Sets the gyroscope angle to zero. This can be used to set the direction the
	 * robot is currently
	 * facing to the 'forwards' direction.
	 */
	public void zeroGyroscope() {
		System.out.println("Zeroing Gyroscope");
		this.navx.reset();
		setCurrentRobotPose(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), new Rotation2d()));
		/*
		 * Lande - I can't remember why we had this last year, but I'm removing it for
		 * now
		 * // Reset the odometry with new 0 heading but same position.
		 * this.odometry.resetPosition(
		 * Rotation2d.fromDegrees(this.navx.getFusedHeading()),
		 * new SwerveModulePosition[] { this.frontLeftModule.getPosition(),
		 * this.frontRightModule.getPosition(),
		 * this.backLeftModule.getPosition(), this.backRightModule.getPosition() },
		 * new Pose2d(this.odometry.getPoseMeters().getTranslation(),
		 * Rotation2d.fromDegrees(0.0)));
		 */
	}

	/**
	 * Calibrates the gyroscope. This should only be called on robotinit because it
	 * takes some time to
	 * run.
	 */
	public void calibrateGyroscope() {
		this.navx.calibrate();
	}

	public Rotation2d getGyroscopeRotation() {
		// // We have to invert the angle of the NavX so that rotating the robot
		// counter-clockwise makes the angle increase.
		return Rotation2d.fromDegrees(-this.navx.getYaw());
	};

	public Pose2d getCurrentRobotPose() {
		return this.poseEstimator.getEstimatedPosition();
	}

	public void setCurrentRobotPose(Pose2d pose) {
		System.out.println("Setting Pose: " + pose.getRotation().getDegrees());

		this.poseEstimator
				.resetPosition(getGyroscopeRotation(),
						new SwerveModulePosition[] {
								this.frontLeftModule.getPosition(),
								this.frontRightModule.getPosition(),
								this.backLeftModule.getPosition(),
								this.backRightModule.getPosition() },
						pose);
	}

	public void stop() {
		for (SwerveModule swerveModule : this.swerveModules) {
			swerveModule.set(0, swerveModule.getSteerAngle());
		}
	}

	public SwerveModulePosition[] getPositions() {
		return new SwerveModulePosition[] {
				this.frontLeftModule.getPosition(),
				this.frontRightModule.getPosition(),
				this.backLeftModule.getPosition(),
				this.backRightModule.getPosition()
		};
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		this.chassisSpeeds = chassisSpeeds;
	}

	public void driveFieldRelative(ChassisSpeeds chassisSpeeds) {
		this.drive(
				ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds,
						this.getGyroscopeRotation()));
	}

	/**
	 * Returns the current state of the module.
	 *
	 * @return The current state of the module.
	 */
	public SwerveModuleState getState(SwerveModule module) {
		return new SwerveModuleState(module.getDriveVelocity(), new Rotation2d(module.getSteerAngle()));
	}

	public boolean gyroConnected() {
		return this.navx.isConnected();
	}

	// #region Logging

	// #region Column 0

	@Log.BooleanBox(name = "Gyro Int?", rowIndex = 0, columnIndex = 0)
	public boolean getGyroInterference() {
		return !this.navx.isMagneticDisturbance();
	}

	// #endregion

	// #region Column 1

	@Log.Dial(name = "FL Angle", min = -90, max = 90, rowIndex = 0, columnIndex = 1, height = 1, width = 1)
	public double getFrontLeftAngle() {
		return Math.IEEEremainder(Math.toDegrees(this.frontLeftModule.getSteerAngle()), 180);
	}

	@Log.Dial(name = "BL Angle", min = -90, max = 90, rowIndex = 1, columnIndex = 1, height = 1, width = 1)
	public double getBackLeftAngle() {
		return Math.IEEEremainder(Math.toDegrees(this.backLeftModule.getSteerAngle()), 180);
	}

	@Log.NumberBar(name = "FL Speed", min = -15, max = 15, rowIndex = 2, columnIndex = 1, height = 1, width = 1)
	public double getFrontLeftVoltage() {
		return this.frontLeftModule.getState().speedMetersPerSecond;
	}

	// #endregion

	// #region Column 2

	@Log.NumberBar(name = "FL Velocity", min = -5, max = 5, rowIndex = 0, columnIndex = 2, height = 1, width = 1)
	public double getFrontLeftSpeed() {
		return this.frontLeftModule.getDriveVelocity();
	}

	@Log.NumberBar(name = "BL Velocity", min = -5, max = 5, rowIndex = 1, columnIndex = 2, height = 1, width = 1)
	public double getBackLeftSpeed() {
		return this.backLeftModule.getDriveVelocity();
	}

	@Log.Graph(name = "Gyro Angle", width = 4, height = 2, rowIndex = 2, columnIndex = 2)
	public double getGyroDegrees() {
		return this.getGyroscopeRotation().getDegrees();
	}

	// #endregion

	// #region Column 3-4

	@Log.Gyro(name = "Robot Angle", rowIndex = 0, columnIndex = 3)
	private AHRS getGyro() {
		return this.navx;
	}

	// #endregion

	// #region Column 5

	@Log.NumberBar(name = "FR Velocity", min = -5, max = 5, rowIndex = 0, columnIndex = 5, height = 1, width = 1)
	public double getFrontRightSpeed() {
		return this.frontRightModule.getDriveVelocity();
	}

	@Log.NumberBar(name = "BR Velocity", min = -5, max = 5, rowIndex = 1, columnIndex = 5, height = 1, width = 1)
	public double getBackRightSpeed() {
		return this.backRightModule.getDriveVelocity();
	}

	// #endregion

	// #region Column 6

	@Log.Dial(name = "FR Angle", min = -90, max = 90, rowIndex = 0, columnIndex = 6, height = 1, width = 1)
	public double getFrontRightAngle() {
		return Math.IEEEremainder(Math.toDegrees(this.frontRightModule.getSteerAngle()), 180);
	}

	@Log.Dial(name = "BR Angle", min = -90, max = 90, rowIndex = 1, columnIndex = 6, height = 1, width = 1)
	public double getBackRightAngle() {
		return Math.IEEEremainder(Math.toDegrees(this.backRightModule.getSteerAngle()), 180);
	}

	@Log(name = "x-Position", rowIndex = 2, columnIndex = 6, height = 1, width = 1)
	public double getXPos() {
		return this.poseEstimator.getEstimatedPosition().getY();
	}

	@Log(name = "y-Position", rowIndex = 3, columnIndex = 6, height = 1, width = 1)
	public double getYPos() {
		return this.poseEstimator.getEstimatedPosition().getX();
	}

	@Log(name = "theta-Position", rowIndex = 4, columnIndex = 6, height = 1, width = 1)
	public double getThetaPos() {
		return this.poseEstimator.getEstimatedPosition().getRotation().getDegrees();
	}

	// #endregion

	// #region Column 7

	@Log(name = "x-Velocity", rowIndex = 2, columnIndex = 7, height = 1, width = 1)
	public double getRobotXVelocity() {
		return this.chassisSpeeds.vxMetersPerSecond;
	}

	@Log(name = "y-Velocity", rowIndex = 3, columnIndex = 7, height = 1, width = 1)
	public double getRobotYVelocity() {
		return this.chassisSpeeds.vyMetersPerSecond;
	}

	@Log(name = "theta-Velocity", rowIndex = 4, columnIndex = 7, height = 1, width = 1)
	public double getRobotThetaVelocity() {
		return this.chassisSpeeds.omegaRadiansPerSecond;
	}

	// #endregion

	// @Log.Field2d(name = "Field2D", tabName = "Field", rowIndex = 0, columnIndex =
	// 0, height = 4, width = 8)
	// public Field2d getField2D() {
	// return this.field2d;
	// }

	// #endregion

	// #region Command Factory

	// Assuming this method is part of a drivetrain subsystem that provides the
	// necessary methods
	// public Command getFollowTrajectoryCommand(PathPlannerTrajectory traj, boolean
	// isFirstPath) {
	// return new SequentialCommandGroup(
	// new InstantCommand(() -> {
	// // Reset odometry for the first path you run during auto
	// if (isFirstPath) {
	// PathPlannerTrajectory transformed =
	// PathPlannerTrajectory.transformTrajectoryForAlliance(traj,
	// DriverStation.getAlliance());
	// this.setCurrentRobotPose(transformed.getInitialHolonomicPose());
	// }
	// }),
	// new PPSwerveControllerCommand(
	// traj,
	// this::getCurrentRobotPose, // Pose supplier
	// this.kinematics, // SwerveDriveKinematics
	// this.autoXController, // X controller. Tune these values for your robot.
	// Leaving them 0 will
	// // only use feedforwards.
	// this.autoYController, // Y controller (usually the same values as X
	// controller)
	// this.autoThetaController, // Rotation controller. Tune these values for your
	// robot. Leaving them
	// // 0
	// // will
	// // only use feedforwards.
	// this::setModuleStates, // Module states consumer
	// false, // Should the path be automatically mirrored depending on alliance
	// color.
	// // Optional, defaults to true
	// this // Requires this drive subsystem
	// ));
	// }

	// #endregion

	public static SwerveSubsystem getInstance() {
		if (instance == null) {
			// if instance is null, initialize
			instance = new SwerveSubsystem();
		}
		return instance;
	}
}
