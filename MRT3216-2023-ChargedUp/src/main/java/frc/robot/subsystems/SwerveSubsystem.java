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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.Auto;
import frc.robot.settings.Constants.Drivetrain;
import frc.robot.settings.Gains;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
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

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private Gains thetaGains;

    private SwerveSubsystem() {
        navx = new AHRS(SerialPort.Port.kUSB1);

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        MkModuleConfiguration moduleConfig = MkModuleConfiguration.getDefaultSteerNEO();
        moduleConfig.setDriveCurrentLimit(40.0);
        moduleConfig.setSteerCurrentLimit(30.0);

        this.frontLeftModule = new MkSwerveModuleBuilder(moduleConfig)
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.NEO, LEFT_FRONT_DRIVE)
                .withSteerMotor(MotorType.NEO, LEFT_FRONT_ANGLE)
                .withSteerEncoderPort(LEFT_FRONT_CANCODER)
                .withSteerOffset(LEFT_FRONT_STEER_OFFSET)
                .build();

        this.frontRightModule = new MkSwerveModuleBuilder()
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.NEO, RIGHT_FRONT_DRIVE)
                .withSteerMotor(MotorType.NEO, RIGHT_FRONT_ANGLE)
                .withSteerEncoderPort(RIGHT_FRONT_CANCODER)
                .withSteerOffset(RIGHT_FRONT_STEER_OFFSET)
                .build();

        this.backLeftModule = new MkSwerveModuleBuilder()
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.NEO, LEFT_REAR_DRIVE)
                .withSteerMotor(MotorType.NEO, LEFT_REAR_ANGLE)
                .withSteerEncoderPort(LEFT_REAR_CANCODER)
                .withSteerOffset(LEFT_REAR_STEER_OFFSET)
                .build();

        this.backRightModule = new MkSwerveModuleBuilder()
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.NEO, RIGHT_REAR_DRIVE)
                .withSteerMotor(MotorType.NEO, RIGHT_REAR_ANGLE)
                .withSteerEncoderPort(RIGHT_REAR_CANCODER)
                .withSteerOffset(RIGHT_REAR_STEER_OFFSET)
                .build();

        this.swerveModules = new SwerveModule[] {
                this.frontLeftModule,
                this.frontRightModule,
                this.backLeftModule,
                this.backRightModule };

        this.poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroscopeRotation(), getPositions(),
                new Pose2d());

        this.thetaGains = Auto.kAutoThetaGains;
    }

    @Override
    public void periodic() {
        this.poseEstimator.update(getGyroscopeRotation(), getPositions());

        final double zeroDeadzone = 0.001;

        // Set deadzone on translation
        if (Math.abs(this.chassisSpeeds.vxMetersPerSecond) < zeroDeadzone) {
            this.chassisSpeeds.vxMetersPerSecond = 0;
        }
        if (Math.abs(this.chassisSpeeds.vyMetersPerSecond) < zeroDeadzone) {
            this.chassisSpeeds.vyMetersPerSecond = 0;
        }

        // Hockey-lock if stopped by setting rotation to realllly low number
        if (this.chassisSpeeds.vxMetersPerSecond == 0 &&
                this.chassisSpeeds.vyMetersPerSecond == 0 &&
                Math.abs(this.chassisSpeeds.omegaRadiansPerSecond) < zeroDeadzone) {
            this.chassisSpeeds.omegaRadiansPerSecond = 0.00001;
        }

        /*
         * SmartDashboard.putNumber("DT X spd", m_chassisSpeeds.vxMetersPerSecond);
         * SmartDashboard.putNumber("DT Y spd", m_chassisSpeeds.vyMetersPerSecond);
         * SmartDashboard.putNumber("DT O rot", m_chassisSpeeds.omegaRadiansPerSecond);
         */

        SwerveModuleState[] states = this.kinematics.toSwerveModuleStates(this.chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);

        SwerveModulePosition[] positions = getPositions();
        for (int i = 0; i < states.length; i++) {
            states[i] = SwerveModuleState.optimize(states[i], positions[i].angle);
        }

        double flVoltage;
        double frVoltage;
        double blVoltage;
        double brVoltage;

        flVoltage = states[0].speedMetersPerSecond;
        frVoltage = states[1].speedMetersPerSecond;
        blVoltage = states[2].speedMetersPerSecond;
        brVoltage = states[3].speedMetersPerSecond;

        // flVoltage = MathUtil.clamp(flVoltage, 0, MAX_VELOCITY_METERS_PER_SECOND);
        // frVoltage = MathUtil.clamp(frVoltage, 0, MAX_VELOCITY_METERS_PER_SECOND);
        // blVoltage = MathUtil.clamp(blVoltage, 0, MAX_VELOCITY_METERS_PER_SECOND);
        // brVoltage = MathUtil.clamp(brVoltage, 0, MAX_VELOCITY_METERS_PER_SECOND);

        // SmartDashboard.putNumber("Front Left Velocity", flVoltage);
        // SmartDashboard.putNumber("Front Right Velocity", frVoltage);
        // SmartDashboard.putNumber("Back Left Velocity", blVoltage);
        // SmartDashboard.putNumber("Back Right Velocity", brVoltage);

        flVoltage = flVoltage / Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Drivetrain.MAX_VOLTAGE;
        frVoltage = frVoltage / Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Drivetrain.MAX_VOLTAGE;
        blVoltage = blVoltage / Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Drivetrain.MAX_VOLTAGE;
        brVoltage = brVoltage / Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Drivetrain.MAX_VOLTAGE;

        this.frontLeftModule.set(flVoltage, states[0].angle.getRadians());
        this.frontRightModule.set(frVoltage, states[1].angle.getRadians());
        this.backLeftModule.set(blVoltage, states[2].angle.getRadians());
        this.backRightModule.set(brVoltage, states[3].angle.getRadians());

        /*
         * XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
         * This is what we had before. Trying similar code from Team 5431 (uses
         * democat's library)
         * SwerveModuleState[] states =
         * this.kinematics.toSwerveModuleStates(this.chassisSpeeds);
         * SwerveDriveKinematics.desaturateWheelSpeeds(states,
         * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);
         * 
         * for (int i = 0; i < this.swerveModules.length; i++) {
         * this.swerveModules[i].set(
         * states[i].speedMetersPerSecond / Drivetrain.MAX_VELOCITY_METERS_PER_SECOND *
         * Drivetrain.MAX_VOLTAGE,
         * states[i].angle.getRadians());
         * }
         * 
         * var gyroAngle = this.getGyroscopeRotation();
         * 
         * // Update the pose
         * this.odometry.update(gyroAngle, getPositions());
         * XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
         */
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the 'forwards' direction.
     */
    public void zeroGyroscope() {
        System.out.println("Zeroing Gyroscope");
        this.navx.reset();

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
     * Calibrates the gyroscope. This should only be called on robotinit because
     * it takes some time to run.
     */
    public void calibrateGyroscope() {
        this.navx.calibrate();
    }

    public Rotation2d getGyroscopeRotation() {
        // // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(-this.navx.getYaw());
    };

    /*
     * public Pose2d getCurrentRobotPose() {
     * return this.odometry.getPoseMeters();
     * }
     * 
     * 
     * public void setCurrentRobotPose(Pose2d pose) {
     * this.odometry
     * .resetPosition(getGyroscopeRotation(),
     * new SwerveModulePosition[] { this.frontLeftModule.getPosition(),
     * this.frontRightModule.getPosition(),
     * this.backLeftModule.getPosition(), this.backRightModule.getPosition() },
     * pose);
     * }
     */

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

    public Gains getThetaGains() {
        return this.thetaGains;
    }

    // @Config.ToggleButton(name = "ResetGyroAndOdometry", defaultValue = false,
    // rowIndex = 3, columnIndex = 0, height = 1, width = 2)
    public void resetGyroAndOdometry(boolean _input) {
        if (_input) {
            this.zeroGyroscope();
            _input = false;
        }
    }

    public boolean navXIsConnected() {
        return this.navx.isConnected();
    }

    // region Logging

    @Log.Gyro(name = "Robot Angle", rowIndex = 0, columnIndex = 3)
    private AHRS getGyro() {
        return this.navx;
    }

    @Log.NumberBar(name = "FL Velocity", min = -5, max = 5, rowIndex = 0, columnIndex = 2, height = 1, width = 1)
    public double getFrontLeftSpeed() {
        return this.frontLeftModule.getDriveVelocity();
    }

    @Log.Dial(name = "FL Angle", min = -90, max = 90, rowIndex = 0, columnIndex = 1, height = 1, width = 1)
    public double getFrontLeftAngle() {
        return Math.IEEEremainder(Math.toDegrees(this.frontLeftModule.getSteerAngle()),
                180);
    }

    @Log.NumberBar(name = "FR Velocity", min = -5, max = 5, rowIndex = 0, columnIndex = 5, height = 1, width = 1)
    public double getFrontRightSpeed() {
        return this.frontRightModule.getDriveVelocity();
    }

    @Log.Dial(name = "FR Angle", min = -90, max = 90, rowIndex = 0, columnIndex = 6, height = 1, width = 1)
    public double getFrontRightAngle() {
        return Math.IEEEremainder(Math.toDegrees(this.frontRightModule.getSteerAngle()),
                180);
    }

    @Log.Dial(name = "BL Angle", min = -90, max = 90, rowIndex = 1, columnIndex = 1, height = 1, width = 1)
    public double getBackLeftAngle() {
        return Math.IEEEremainder(Math.toDegrees(this.backLeftModule.getSteerAngle()),
                180);
    }

    @Log.NumberBar(name = "BL Velocity", min = -5, max = 5, rowIndex = 1, columnIndex = 2, height = 1, width = 1)
    public double getBackLeftSpeed() {
        return this.backLeftModule.getDriveVelocity();
    }

    @Log.NumberBar(name = "BR Velocity", min = -5, max = 5, rowIndex = 1, columnIndex = 5, height = 1, width = 1)
    public double getBackRightSpeed() {
        return this.backRightModule.getDriveVelocity();
    }

    @Log.Dial(name = "BR Angle", min = -90, max = 90, rowIndex = 1, columnIndex = 6, height = 1, width = 1)
    public double getBackRightAngle() {
        return Math.IEEEremainder(Math.toDegrees(this.backRightModule.getSteerAngle()),
                180);
    }

    @Log(name = "x-Position", rowIndex = 2, columnIndex = 6, height = 1, width = 1)
    public double getYPos() {
        return this.poseEstimator.getEstimatedPosition().getY();
    }

    @Log(name = "y-Position", rowIndex = 3, columnIndex = 6, height = 1, width = 1)
    public double getXPos() {
        return this.poseEstimator.getEstimatedPosition().getX();
    }

    @Log(name = "theta-Position", rowIndex = 4, columnIndex = 6, height = 1, width = 1)
    public double getThetaPos() {
        return this.poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    }

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

    @Log.BooleanBox(name = "Gyro Int?", rowIndex = 0, columnIndex = 0)
    public boolean getGyroInterference() {
        return this.navx.isMagneticDisturbance();
    }

    @Config.ToggleButton(name = "ResetPosition", defaultValue = false, rowIndex = 4, columnIndex = 0, height = 1, width = 2)
    public void resetPosition(boolean _input) {
        if (_input) {
            // Reset the odometry with new 0 heading and zero Position.
            // m_odometry.resetPosition(new Pose2d(), new Rotation2d());
            _input = false;
        }
    }

    @Config.NumberSlider(name = "Theta P", tabName = "Tuning", defaultValue = Auto.kThetaP, min = 0, max = 20, rowIndex = 5, columnIndex = 0, height = 1, width = 1)
    public void setThetaP(double thetaP) {
        this.thetaGains.kP = thetaP;
    }

    @Config.NumberSlider(name = "Theta I", tabName = "Tuning", defaultValue = Auto.kThetaI, min = 0, max = 1, rowIndex = 5, columnIndex = 1, height = 1, width = 1)
    public void setThetaI(double thetaI) {
        this.thetaGains.kI = thetaI;
    }

    @Config.NumberSlider(name = "Theta D", tabName = "Tuning", defaultValue = Auto.kThetaD, min = 0, max = 1, rowIndex = 5, columnIndex = 2, height = 1, width = 1)
    public void setThetaD(double thetaD) {
        this.thetaGains.kD = thetaD;
    }

    @Log.Graph(name = "Gyro Angle", width = 4, height = 2, rowIndex = 2, columnIndex = 2)
    public double getGyroDegrees() {
        return this.getGyroscopeRotation().getDegrees();
    }

    public static SwerveSubsystem getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new SwerveSubsystem();
        }
        return instance;
    }
}