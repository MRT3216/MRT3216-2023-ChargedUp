/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.settings;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.ArmSubsystem;

public final class Constants {
	public static final class DRIVETRAIN {
		// TODO: set these values
		// public static final double LEFT_FRONT_STEER_OFFSET = -Math.toRadians(206.54);
		// public static final double RIGHT_FRONT_STEER_OFFSET = -Math.toRadians(81.56);
		// public static final double LEFT_REAR_STEER_OFFSET = -Math.toRadians(20.15);
		// public static final double RIGHT_REAR_STEER_OFFSET = -Math.toRadians(292.41);
		public static final double LEFT_FRONT_STEER_OFFSET = -Math.toRadians(115.31 + 180);
		public static final double RIGHT_FRONT_STEER_OFFSET = -Math.toRadians(202.14 + 180);
		public static final double LEFT_REAR_STEER_OFFSET = -Math.toRadians(260.33 + 180);
		public static final double RIGHT_REAR_STEER_OFFSET = -Math.toRadians(24.78 + 180);

		public static final double WHEELBASE_METERS = Units.inchesToMeters(23.058); // 0.5461;
		public static final double TRACKWIDTH_METERS = Units.inchesToMeters(18.914); // 0.5588;

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
		 */ // 4.578870701248506
		public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0
				/ 60.0
				* SdsModuleConfigurations.MK4I_L2.getDriveReduction()
				* SdsModuleConfigurations.MK4I_L2.getWheelDiameter()
				* Math.PI;

		public static final double MAX_ACCELERATION_METERS_PER_SECONDS_SQUARED = MAX_VELOCITY_METERS_PER_SECOND;

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

	public static final class ARM {
		public static final boolean kLeftMotorsInverted = true;
		public static final boolean kRightMotorsInverted = false;
		public static final float kReverseLimit = .01f;
		public static final float kForwardLimit = .65f;
		public static final double kMaxLimitDegrees = ArmSubsystem.calculateArmDegrees(kForwardLimit);
		public static final double kMinLimitDegrees = ArmSubsystem.calculateArmDegrees(kReverseLimit);
		public static final int kMotorCurrentLimit = 40;
		public static final double kZeroOffset = 0.0;
		public static final double kZeroOffsetInDegrees = 0.0;
		public static final double kScaleFactor = 192.86;
		public static final double kLimitSwitchOffset = 0.3170284;

		/*
		 * ----------------------------------------------------------------------------
		 * PID Constants
		 */
		public static final double kArmKp = 0.2;
		public static final double kArmKi = 0;
		public static final double kArmKd = 0;
		public static final double kArmPositionTolerance = 4;
		public static final double kArmVelocityTolerance = 20;

		/*
		 * ----------------------------------------------------------------------------
		 * Constraint Constants
		 */
		public static final double kArmMaxVelocity = 175; // degrees/s
		public static final double kArmMaxAcceleration = 125; // degrees/s^2

		/*
		 * ----------------------------------------------------------------------------
		 * Position Constants
		 */
		public static final int kScoringHighConeDegrees = 120;
		public static final int kScoringHighCubeDegrees = 115;
		public static final int kScoringMidConeDegrees = 107;
		public static final int kScoringMidCubeDegrees = 110;
		public static final int kScoringHybridConeDegrees = 30;
		public static final int kScoringHybridCubeDegrees = 40;
		public static final int kGroundIntakeUprightConeDegrees = 22;
		public static final int kGroundIntakeTippedConeDegrees = 15;
		public static final int kGroundIntakeCubeDegrees = 7;
		public static final int kSingleSubstationIntakeConeDegrees = 43;
		public static final int kSingleSubstationIntakeCubeDegrees = 43;
		public static final int kDoubleSubstationIntakeConeDegrees = 55;
		public static final int kDoubleSubstationIntakeCubeDegrees = 55;
		public static final int kStowedDegrees = 50;
		public static final int kStartDegrees = 65;

		// #region Enums

		public enum Position {
			ScoringHighCone(0),
			ScoringHighCube(1),
			ScoringMidCone(2),
			ScoringMidCube(3),
			ScoringHybridCone(4),
			ScoringHybridCube(5),
			GroundIntakeUprightCone(6),
			GroundIntakeTippedCone(7),
			GroundIntakeCube(8),
			SingleSubstationIntakeCone(9),
			SingleSubstationIntakeCube(10),
			DoubleSubstationIntakeCone(11),
			DoubleSubstationIntakeCube(12),
			Stowed(13),
			Start(14);

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
			SingleSubstation(1),
			DoubleSubstation(2);

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

		public enum Substation {
			Single(0),
			Double(1);

			private int value;
			private static Map<Integer, Substation> map = new HashMap<>();

			private Substation(int value) {
				this.value = value;
			}

			static {
				for (Substation position : Substation.values()) {
					map.put(position.value, position);
				}
			}

			public static Substation valueOf(int position) {
				return (Substation) map.get(position);
			}

			public int getValue() {
				return value;
			}
		}

		// #endregion
	}

	public static final class WRIST {
		// TODO add in actual limits for manipulator encoder
		// public static final float kForwardLimit = -1f;
		// public static final float kReverseLimit = .35f;

		public static final int kMotorCurrentLimit = 40;

		public static final boolean kMotorInverted = false;

		// public static final double kReverseLimitDegrees =
		// WristSubsystem.calculateWristDegreesWrtArm(kReverseLimit) + 2;
		// public static final double kForwardLimitDegrees =
		// WristSubsystem.calculateWristDegreesWrtArm(kForwardLimit) - 2;

		public static final double kZeroOffset = 0;
		public static final double kScaleFactor = 180;

		public static final double kLimitSwitchPosition = 0;

		/*
		 * ----------------------------------------------------------------------------
		 * PID Constants
		 */
		public static final double kWristKp = .25;
		public static final double kWristKi = 0;
		public static final double kWristKd = 0;
		public static final double kWristPositionTolerance = 4;
		public static final double kWristVelocityTolerance = 20;

		// Feed-Forward from SysID
		// public static final double kWristKs = 0.29522;
		// public static final double kWristKv = 0.0095926;
		// public static final double kWristKa = 0.0032297;
		// public static final double kWristKg = 0.52135 * 3;

		// public static final double kVVoltSecondPerRad = 0.5;

		/*
		 * ----------------------------------------------------------------------------
		 * Constraint Constants
		 */
		public static final double kWristMaxVelocity = 225; // degrees/s
		public static final double kWristMaxAcceleration = 225; // degrees/s^2
		// public static final double kWristStartingPos = 0; // 60 degrees wrt arm

		/*
		 * ----------------------------------------------------------------------------
		 * Position Constants
		 */
		public static final int kScoringHighConeDegrees = -135;
		public static final int kScoringHighCubeDegrees = -190;
		public static final int kScoringMidConeDegrees = -70;
		public static final int kScoringMidCubeDegrees = -115;
		public static final int kScoringHybridConeDegrees = 43;
		public static final int kScoringHybridCubeDegrees = -150;
		public static final int kGroundIntakeUprightConeDegrees = 65;
		public static final int kGroundIntakeTippedConeDegrees = 15;
		public static final int kGroundIntakeCubeDegrees = 25;
		public static final int kSingleSubstationIntakeConeDegrees = 110;
		public static final int kSingleSubstationIntakeCubeDegrees = 90;
		public static final int kDoubleSubstationIntakeConeDegrees = 110;
		public static final int kDoubleSubstationIntakeCubeDegrees = -20;
		public static final int kStowedDegrees = 0;
		public static final int kStartDegrees = 20;
	}

	public static final class INTAKE {
		public static final boolean kMotorInverted = true;
		public static final int kMotorCurrentLimit = 30;
		public static final double kConeIntakeSpeed = -.53;
		public static final double kConeOuttakeSpeed = 0.7;
		public static final double kCubeIntakeSpeed = 0.52;
		public static final double kCubeOuttakeSpeed = -0.5;
		public static final double kCubeShootSpeed = -1.0;
	}

	public static final class AUTO {
		public static final boolean usePhotonVision = false;

		// Proportional gain
		public static final double kPositionP = 4;
		// Integral gain
		public static final double kPositionI = 0;
		// Derivative gain
		public static final double kPositionD = 0;
		// Proportional gain
		public static final double kThetaP = 8;
		// Integral gain
		public static final double kThetaI = 0;
		// Derivative gain
		public static final double kThetaD = 0;

		public static final double kStartDelayTime = 0;

		public static final double kMaxIntakeTime = 1;
		public static final double kMaxOuttakeTime = 0.25;

		public static final double kBalanceClimbingAngle = 12.5;
		public static final double kBalanceTippingAngle = 9.5;

		public static final PathConstraints kFastPath = new PathConstraints(3.0, 3);
		public static final PathConstraints kMediumPath = new PathConstraints(2.5, 2);
		public static final PathConstraints kMedium3Piece = new PathConstraints(2.75, 2.25);
		public static final PathConstraints kMediumSlowPath = new PathConstraints(2, 1.75);
		public static final PathConstraints kNotAsSlowPath = new PathConstraints(2, 1.75);
		public static final PathConstraints kSlowPath = new PathConstraints(1, 1);
		public static final PathConstraints kReallySlowPath = new PathConstraints(.75, .75);
	}

	public static final class AUTO_BALANCE {
		/**********
		 * CONFIG *
		 **********/
		// Speed the robot drived while scoring/approaching station, default = 0.4
		public static final double kRobotSpeedFast = 0.25;

		// Speed the robot drives while balancing itself on the charge station.
		// Should be roughly half the fast speed, to make the robot more accurate,
		// default = 0.2
		public static final double kRobotSpeedSlow = 0.15;

		// Angle where the robot knows it is on the charge station, default = 13.0
		public static final double kOnChargeStationDegree = 11.0;

		// Angle where the robot can assume it is level on the charging station
		// Used for exiting the drive forward sequence as well as for auto balancing,
		// default = 6.0
		public static final double kLevelDegree = 4.0;

		// Amount of time a sensor condition needs to be met before changing states in
		// seconds
		// Reduces the impact of sensor noice, but too high can make the auto run
		// slower, default = 0.2
		public static final double kDebounceTime = 0.05;

		// Speed at which the robot reverses once charge station tips forward
		public static final double kReverseCorrectionSpeed = -0.07;
		// Speed it goes the other way when it tips backs
		public static final double kForwardCorrectionSpeed = 0.05;
	}

	public static final class LIMELIGHT {
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

	public static final class PHOTONVISION {
		// TODO: Set these to our robot
		public static final Transform3d robotToLeftCam = new Transform3d(
				new Translation3d(0.5, 0.0, 0.5),
				new Rotation3d(
						0, 0,
						0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
								// from center.
		public static final Transform3d robotToRighttCam = new Transform3d(
				new Translation3d(0.5, 0.0, 0.5),
				new Rotation3d(
						0, 0,
						0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
								// from center.

		// TODO: Fill in camera names from PhotonVision
		public static final String leftCameraName = "YOUR CAMERA NAME";
		public static final String rightCameraName = "YOUR CAMERA NAME";
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
