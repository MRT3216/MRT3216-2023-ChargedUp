/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.settings;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.wpilibj.Filesystem;

public final class Constants {
    public static final class Drivetrain {
        // TODO: set these values
        public static final double LEFT_FRONT_STEER_OFFSET = -Math.toRadians(-0.35); //rotating inverse of other wheels
        public static final double RIGHT_FRONT_STEER_OFFSET = -Math.toRadians(77.87);
        public static final double LEFT_REAR_STEER_OFFSET = -Math.toRadians(-0.26);
        public static final double RIGHT_REAR_STEER_OFFSET = -Math.toRadians(-68.20);

        // TODO: set these values
        public static final double WHEELBASE_METERS = 0.5461;
        public static final double TRACKWIDTH_METERS = 0.5588;

        /**
         * The maximum voltage that will be delivered to the drive motors.
         * <p>
         * This can be reduced to cap the robot's maximum speed. Typically, this is
         * useful during initial testing of the robot.%
         */
        public static final double MAX_VOLTAGE = 12.0;
        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
        // pi
        // By default this value is setup for a Mk3 standard module using Falcon500s to
        // drive.
        // An example of this constant for a Mk4 L2 module with NEOs to drive is:
        // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
        /**
         * The maximum velocity of the robot in meters per second.
         * <p>
         * This is a measure of how fast the robot should be able to drive in a straight
         * line.
         */
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
                * SdsModuleConfigurations.MK4I_L2.getDriveReduction()
                * SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
                / Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_PER_SECOND = Math.PI;
    }

    public static final class Auto {
        // Proportional gain
        public static final double kPositionP = 0.001;
        // Integral gain
        public static final double kPositionI = 0;
        // Derivative gain
        public static final double kPositionD = 0;
        // Proportional gain
        public static final double kThetaP = 20;
        // Integral gain
        public static final double kThetaI = 0;
        // Derivative gain
        public static final double kThetaD = 0.6;

        public static final Gains kAutoPositionGains = new Gains(kPositionP, kPositionI, kThetaD);
        public static final Gains kAutoThetaGains = new Gains(kThetaP, kThetaI, kThetaD);

        public static final double kMaxTurnError = .1; // degrees
        public static final double kMaxTurnRateError = 1; // Degrees per second

        public static final double kMaxTurnErrorAuto = 5;
        public static final double kMaxTurnRateErrorAuto = 5;

        public static final int kMaxFetchVelocity = 8;
        public static final int kMaxFetchAcc = kMaxFetchVelocity / 2;

        public static final double kStartDelayTime = 0;
        public static final double kDriveToShootDelay = 0; // seconds
        public static final double kMaxShootTime = 2; // seconds
    }

    public static final class OI {
        public static final double kJoystickDeadband = 0.1;
        public static final double kTranslationExpo = 75;
        public static final double kRotationnExpo = 75;
    }

    public static final class Directories {
        public static final String deployDirectory = Filesystem.getDeployDirectory().getAbsolutePath();
        public static final String pathsDirectory = deployDirectory + "/paths/";
    }
}