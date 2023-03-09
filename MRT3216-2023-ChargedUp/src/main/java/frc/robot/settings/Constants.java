/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.settings;

import java.util.HashMap;
import java.util.Map;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.ArmSubsystem;

public final class Constants {
	public static final class Drivetrain {
		// TODO: set these values
		public static final double LEFT_FRONT_STEER_OFFSET = -Math.toRadians(206.54);
		public static final double RIGHT_FRONT_STEER_OFFSET = -Math.toRadians(81.56);
		public static final double LEFT_REAR_STEER_OFFSET = -Math.toRadians(20.13);
		public static final double RIGHT_REAR_STEER_OFFSET = -Math.toRadians(292.41);

		public static final double WHEELBASE_METERS = Units.inchesToMeters(23.058); // 0.5461; 
		public static final double TRACKWIDTH_METERS = Units.inchesToMeters(18.914); //  0.5588;

		/**
		 * The maximum voltage that will be delivered to the drive motors.
		 *
		 * <p>
		 * This can be reduced to cap the robot's maximum speed. Typically, this is
		 * useful during
		 * initial testing of the robot.%
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
		 *
		 * <p>
		 * This is a measure of how fast the robot should be able to drive in a straight
		 * line.
		 */
		public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0
				/ 60.0
				* SdsModuleConfigurations.MK4I_L2.getDriveReduction()
				* SdsModuleConfigurations.MK4I_L2.getWheelDiameter()
				* Math.PI;

		/**
		 * The maximum angular velocity of the robot in radians per second.
		 *
		 * <p>
		 * This is a measure of how fast the robot can rotate in place.
		 */
		// Here we calculate the theoretical maximum angular velocity. You can also
		// replace this with a measured amount.
		public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
				/ Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

		public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_PER_SECOND = Math.PI;
	}

	public static final class LimeLight {
		public static final String NTtable = "limelight";

		// modes:
		// 0 = use the LED Mode set in the current pipeline
		// 1 = force off
		// 2 = force blink
		// 3 = force on
		public enum LEDMode {
			PIPELINE,
			OFF,
			BLINK,
			ON
		}

		// modes:
		// 0 = vision processor
		// 1 = driver camera
		public enum CameraMode {
			VISION,
			DRIVER
		}

		// Set stream:
		// 0 = Standard - Side-by-side streams if a webcam is attached to Limelight
		// 1 = PiP Main - The secondary camera stream is placed in the lower-right
		// corner of the primary camera stream
		// 2 = PiP Secondary - The primary camera stream is placed in the lower-right
		// corner of the secondary camera stream
		public enum CameraStream {
			Standard,
			PiPMain,
			PiPSecondary
		}
	}

	public static final class ARM {
		public static final boolean kLeftMotorsInverted = true;
		public static final boolean kRightMotorsInverted = false;
		public static final float kReverseLimit = .02f;
		public static final float kForwardLimit = .74f;
		public static final double kMaxLimitDegrees = ArmSubsystem.calculateArmDegrees(kForwardLimit);
		public static final double kMinLimitDegrees = ArmSubsystem.calculateArmDegrees(kReverseLimit);
		public static final int kMotorCurrentLimit = 40;
		public static final double kZeroOffset = .0f;
		public static final double kScaleFactor = 192.86;

		/*
		 * ----------------------------------------------------------------------------
		 * PID Constants
		 */
		public static final double kArmKp = .2;
		public static final double kArmKi = 0;
		public static final double kArmKd = 0;
		public static final double kArmPositionTolerance = 2;
		public static final double kArmVelocityTolerance = 20;

		/*
		 * ----------------------------------------------------------------------------
		 * Constraint Constants
		 */
		public static final double kArmMaxVelocity = 150; // degrees/s
		public static final double kArmMaxAcceleration = 60; // degrees/s^2
		public static final double kArmStartingPos = 80; // 60 degrees wrt hortizontal

		/*
		 * ----------------------------------------------------------------------------
		 * Position Constants
		 */
		public static final int kScoringHighConeDegrees = 100;
		public static final int kScoringHighCubeDegrees = 90;
		public static final int kScoringMidConeDegrees = 80;
		public static final int kScoringMidCubeDegrees = 70;
		public static final int kScoringHybridDegrees = 30;
		public static final int kGroundIntakeUprightConeDegrees = 20;
		public static final int kGroundIntakeTippedConeDegrees = 15;
		public static final int kGroundIntakeCubeDegrees = 10;
		public static final int kSubstationIntakeConeDegrees = 124;
		public static final int kSubstationIntakeCubeDegrees = 133;
		public static final int kStowedDegrees = 60;

		// #region Enums

		public enum Position {
			ScoringHighCone(0),
			ScoringHighCube(1),
			ScoringMidCone(2),
			ScoringMidCube(3),
			ScoringHybrid(4),
			GroundIntakeUprightCone(5),
			GroundIntakeTippedCone(6),
			GroundIntakeCube(7),
			SubstationIntakeCone(8),
			SubstationIntakeCube(9),
			Stowed(10);

			private int value;
			private static Map<Integer, Position> map = new HashMap<>();

			private Position(int value) {
				this.value = value;
			}

			static {
				for (Position position : Position.values()) {
					map.put(position.value, position);
				}
			}

			public static Position valueOf(int position) {
				return (Position) map.get(position);
			}

			public int getValue() {
				return value;
			}
		}

		public enum GamePiece {
			Cone(0),
			Cube(1);

			private int value;
			private static Map<Integer, GamePiece> map = new HashMap<>();

			private GamePiece(int value) {
				this.value = value;
			}

			static {
				for (GamePiece gamePiece : GamePiece.values()) {
					map.put(gamePiece.value, gamePiece);
				}
			}

			public static GamePiece valueOf(int gamePiece) {
				return (GamePiece) map.get(gamePiece);
			}

			public int getValue() {
				return value;
			}
		}

		public enum ScoringHeight {
			High(0),
			Mid(1),
			Hybrid(2);

			private int value;
			private static Map<Integer, ScoringHeight> map = new HashMap<>();

			private ScoringHeight(int value) {
				this.value = value;
			}

			static {
				for (ScoringHeight scoringPiece : ScoringHeight.values()) {
					map.put(scoringPiece.value, scoringPiece);
				}
			}

			public static ScoringHeight valueOf(int scoringPiece) {
				return (ScoringHeight) map.get(scoringPiece);
			}

			public int getValue() {
				return value;
			}
		}

		public enum IntakePosition {
			Ground(0),
			Substation(1);

			private int value;
			private static Map<Integer, IntakePosition> map = new HashMap<>();

			private IntakePosition(int value) {
				this.value = value;
			}

			static {
				for (IntakePosition position : IntakePosition.values()) {
					map.put(position.value, position);
				}
			}

			public static IntakePosition valueOf(int position) {
				return (IntakePosition) map.get(position);
			}

			public int getValue() {
				return value;
			}
		}
		
		// #endregion
	}

	public static final class WRIST {
		// TODO add in actual limits for manipulator encoder
		public static final float kForwardLimit = .99f;
		public static final float kReverseLimit = 0.01f;

		public static final int kMotorCurrentLimit = 40;

		public static final boolean kMotorInverted = false;

		public static final double kReverseLimitDegrees = ArmSubsystem.calculateWristDegreesWrtArm(kReverseLimit) + 2;
		public static final double kForwardLimitDegrees = ArmSubsystem.calculateWristDegreesWrtArm(kForwardLimit) - 2;

		public static final double kZeroOffset = 0;
		public static final double kScaleFactor = 180;

		public static final double kLimitSwitchPosition = 0;

		/*
		 * ----------------------------------------------------------------------------
		 * PID Constants
		 */
		public static final double kWristKp = .2;
		public static final double kWristKi = 0;
		public static final double kWristKd = 0;
		public static final double kWristPositionTolerance = 2;
		public static final double kWristVelocityTolerance = 20;

		// Feed-Forward from SysID
		public static final double kWristKs = 0.29522;
		public static final double kWristKv = 0.0095926;
		public static final double kWristKa = 0.0032297;
		public static final double kWristKg = 0.52135 * 3;

		public static final double kVVoltSecondPerRad = 0.5;

		/*
		 * ----------------------------------------------------------------------------
		 * Constraint Constants
		 */
		public static final double kWristMaxVelocity = 60; // degrees/s
		public static final double kWristMaxAcceleration = 30; // degrees/s^2
		public static final double kWristStartingPos = 0; // 60 degrees wrt arm

		/*
		 * ----------------------------------------------------------------------------
		 * Position Constants
		 */
		public static final int kScoringHighConeDegrees = 100;
		public static final int kScoringHighCubeDegrees = 90;
		public static final int kScoringMidConeDegrees = 80;
		public static final int kScoringMidCubeDegrees = 70;
		public static final int kScoringHybridDegrees = 30;
		public static final int kGroundIntakeUprightConeDegrees = 189;//
		public static final int kGroundIntakeTippedConeDegrees = 15;
		public static final int kGroundIntakeCubeDegrees = 189;//
		public static final int kSubstationIntakeConeDegrees = 25;//
		public static final int kSubstationIntakeCubeDegrees = 9;//
		public static final int kStowedDegrees = 60;

	}

	public static final class INTAKE {
		public static final boolean kMotorInverted = true;
		public static final int kMotorCurrentLimit = 30;
		public static final double kConeIntakeSpeed = 0.7;
		public static final double kConeOuttakeSpeed = -0.5;
		public static final double kCubeIntakeSpeed = -0.5;
		public static final double kCubeOuttakeSpeed = 0.5;
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

	public static final class StreamDeck {
		public static final String NTtable = "StreamDeck";
		public static final String scoringHeight = "scoringHeight";
		public static final String gamePiece = "gamePiece";
	}

	public static final class Directories {
		public static final String deployDirectory = Filesystem.getDeployDirectory().getAbsolutePath();
		public static final String pathsDirectory = deployDirectory + "/paths/";
	}
}
