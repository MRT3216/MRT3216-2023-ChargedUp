// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Dictionary;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.ARM.GamePiece;
import frc.robot.settings.Constants.ARM.Position;
import frc.robot.settings.Constants.AUTO;
import frc.robot.settings.Constants.Directories;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class AutoChooser implements Loggable {
	private static AutoChooser instance;
	private static HashMap<String, Command> eventMap;
	@Log.Exclude
	@Config.Exclude
	private static SwerveAutoBuilder autoBuilder;
	@Log.Exclude
	@Config.Exclude
	private static ArmSubsystem armSubsystem;
	private Dictionary<String, Trajectory> trajectories;
	private SendableChooser<Supplier<Command>> chooser;
	@Log.Exclude
	@Config.Exclude
	private SwerveSubsystem swerveSubsystem;
	@Log.Exclude
	@Config.Exclude
	private static IntakeSubsystem intakeSubsystem;

	// #region Logging
	// @Log
	// private double maxTranslationError = 0;
	// @Log
	// private double maxRotationError = 0;
	// @Log
	// private double curentTranslationError = 0;
	// @Log
	// private double currentRotationError = 0;
	// @Log
	// private double translationSetpoint = 0;
	// @Log
	// private double rotationSetpoint = 0;
	// @Log
	// private double currentXError = 0;
	// @Log
	// private double currentYError = 0;
	// @Log
	// private double xSetpoint = 0;
	// @Log
	// private double ySetpoint = 0;
	// #endregion

	private AutoChooser() {
		swerveSubsystem = SwerveSubsystem.getInstance();
		armSubsystem = ArmSubsystem.getInstance();
		intakeSubsystem = IntakeSubsystem.getInstance();

		this.populateAutoChooser();

		Shuffleboard.getTab("Driver")
				.add("Auto Mode", chooser)
				.withSize(2, 1)
				.withPosition(7, 0);
	}

	public void init() {
		eventMap = buildEventMapReal();

		autoBuilder = new SwerveAutoBuilder(
				swerveSubsystem::getCurrentRobotPose,
				swerveSubsystem::setCurrentRobotPose,
				new PIDConstants(AUTO.kPositionP, AUTO.kPositionI, AUTO.kPositionD),
				new PIDConstants(AUTO.kThetaP, AUTO.kThetaI, AUTO.kThetaD),
				swerveSubsystem::driveFieldRelative,
				eventMap,
				true,
				swerveSubsystem);

		// #region Logging
		// PPSwerveControllerCommand.setLoggingCallbacks(
		// (PathPlannerTrajectory activeTrajectory) -> {
		// // Log current trajectory
		// },
		// (Pose2d targetPose) -> {
		// // Log target pose
		// rotationSetpoint = targetPose.getRotation().getDegrees();
		// xSetpoint = targetPose.getX();
		// ySetpoint = targetPose.getY();
		// },
		// (ChassisSpeeds setpointSpeeds) -> {
		// // Log setpoint ChassisSpeeds
		// },
		// (Translation2d translationError, Rotation2d rotationError) -> {
		// curentTranslationError = translationError.getNorm();
		// currentRotationError = rotationError.getDegrees();
		// maxTranslationError = Math.max(maxTranslationError,
		// translationError.getNorm());
		// maxRotationError = Math.max(maxRotationError, rotationError.getDegrees());
		// currentXError = translationError.getX();
		// currentYError = translationError.getY();
		// });
		// #endregion
	}

	private static HashMap<String, Command> buildEventMapReal() {
		return new HashMap<>(
				Map.ofEntries(
						Map.entry("armToHighCone", armSubsystem.getCommand(Position.ScoringHighCone)),
						Map.entry("armToMidCone", armSubsystem.getCommand(Position.ScoringMidCone)),
						Map.entry("armToHybridCone", armSubsystem.getCommand(Position.ScoringHybridCone)),
						Map.entry("armToHighCube", armSubsystem.getCommand(Position.ScoringHighCube)),
						Map.entry("armToMidCube", armSubsystem.getCommand(Position.ScoringMidCube)),
						Map.entry("armToHybridCube", armSubsystem.getCommand(Position.ScoringHybridCube)),
						Map.entry("armToIntakeCone", armSubsystem.getCommand(Position.GroundIntakeTippedCone)),
						Map.entry("armToIntakeCube", armSubsystem.getCommand(Position.GroundIntakeCube)),
						Map.entry("intakeCone", intakeSubsystem.getAutoConeCommand(true)),
						Map.entry("intakeCube", intakeSubsystem.getAutoCubeCommand(true)),
						Map.entry("ejectCone", intakeSubsystem.getAutoConeCommand(false)),
						Map.entry("ejectCube", intakeSubsystem.getAutoCubeCommand(false)),
						Map.entry("start", armSubsystem.getStartCommand()),
						Map.entry("stow", armSubsystem.getStowedCommand())));
	}

	private void populateAutoChooser() {
		chooser = new SendableChooser<>();

		chooser.setDefaultOption("Do Nothing",
				() -> new WaitCommand(0));

		chooser.addOption("A-Leave",
				() -> autoBuilder.fullAuto(PathPlanner.loadPathGroup("A-Cn-Leave",
						AUTO.kReallySlowPath)));

		chooser.addOption("A-PlaceCone",
				() -> armSubsystem.getCommandAndWait(Position.ScoringHighCone)
						.andThen(
								intakeSubsystem.getAutoEjectConeCommand()));

		chooser.addOption("A-Cn-Leave",
				() -> armSubsystem.getCommandAndWait(Position.ScoringHighCone)
						.andThen(
								intakeSubsystem.getAutoEjectConeCommand(),
								autoBuilder.fullAuto(PathPlanner.loadPathGroup("A-Cn-Leave",
										AUTO.kReallySlowPath))));

		chooser.addOption("M-Cn+Cb-Balance",
				() -> armSubsystem.getCommandAndWait(Position.ScoringHighCone)
						.andThen(
								intakeSubsystem.getAutoEjectConeCommand(),
								autoBuilder.fullAuto(PathPlanner.loadPathGroup("M-Cn+Cb-Balance", AUTO.kSlowPath)),
								AutoBalance.getInstance().getAutoBalanceCommand(true),
								Commands.run(() -> swerveSubsystem.setModuleStatesHockeyStop(), swerveSubsystem))
						.finallyDo(
								(end) -> armSubsystem.setGamePiece(GamePiece.Cube)));

		chooser.addOption("C-Cn+Cb-Dock",
				() -> armSubsystem.getCommandAndWait(Position.ScoringHighCone)
						.andThen(
								intakeSubsystem.getAutoEjectConeCommand(),
								autoBuilder.fullAuto(PathPlanner.loadPathGroup("C-Cn+Cb-Dock", AUTO.kMediumPath)),
								AutoBalance.getInstance().getAutoBalanceCommand(true),
								Commands.run(() -> swerveSubsystem.setModuleStatesHockeyStop(), swerveSubsystem))
						.finallyDo(
								(end) -> armSubsystem.setGamePiece(GamePiece.Cube)));

		chooser.addOption("C-CnCb-Leave",
				() -> armSubsystem.getCommandAndWait(Position.ScoringHighCone)
						.andThen(
								intakeSubsystem.getAutoEjectConeCommand(),
								autoBuilder.fullAuto(PathPlanner.loadPathGroup("C-CnCb-Leave", AUTO.kMediumPath))));

		chooser.addOption("C-CnCb+Cb",
				() -> armSubsystem.getCommandAndWait(Position.ScoringHighCone)
						.andThen(
								intakeSubsystem.getAutoEjectConeCommand(),
								autoBuilder.fullAuto(PathPlanner.loadPathGroup("C-CnCb+Cb", AUTO.kMediumPath)))
						.finallyDo(
								(end) -> armSubsystem.setGamePiece(GamePiece.Cube)));

		chooser.addOption("S-CnCb-Balance",
				() -> armSubsystem.getCommandAndWait(Position.ScoringHighCone)
						.andThen(
								intakeSubsystem.getAutoEjectConeCommand(),
								autoBuilder.fullAuto(PathPlanner.loadPathGroup("S-CnCb-Balance", AUTO.kFastPath)),
								AutoBalance.getInstance().getAutoBalanceCommand(false),
								Commands.run(() -> swerveSubsystem.setModuleStatesHockeyStop(), swerveSubsystem)));

		chooser.addOption("S-CnCbCb-Leave",
				() -> armSubsystem.getCommandAndWait(Position.ScoringHighCone)
						.andThen(
								intakeSubsystem.getAutoEjectConeCommand(),
								autoBuilder.fullAuto(PathPlanner.loadPathGroup("S-CnCbCb-Leave", AUTO.kFastPath))));

		chooser.addOption("S-CnCbCb-PrepCn",
				() -> armSubsystem.getCommandAndWait(Position.ScoringHighCone)
						.andThen(
								intakeSubsystem.getAutoEjectConeCommand(),
								autoBuilder.fullAuto(PathPlanner.loadPathGroup("S-CnCbCb-PrepCn", AUTO.kFastPath))));

		chooser.addOption("S-CnCbCb-Dock",
				() -> armSubsystem.getCommandAndWait(Position.ScoringHighCone)
						.andThen(
								intakeSubsystem.getAutoEjectConeCommand(),
								autoBuilder.fullAuto(PathPlanner.loadPathGroup("S-CnCbCb-Dock", AUTO.kFastestPath)),
								Commands.run(() -> swerveSubsystem.setModuleStatesHockeyStop(), swerveSubsystem)));

		chooser.addOption("S-CnCbCb-Balance",
				() -> armSubsystem.getCommandAndWait(Position.ScoringHighCone)
						.andThen(
								intakeSubsystem.getAutoEjectConeCommand(),
								autoBuilder.fullAuto(PathPlanner.loadPathGroup("S-CnCbCb-Balance", AUTO.kFastestPath)),
								AutoBalance.getInstance().getAutoBalanceCommand(false),
								Commands.run(() -> swerveSubsystem.setModuleStatesHockeyStop(), swerveSubsystem)));

		// chooser.addOption("C-CnCb-Dock",
		// () -> armSubsystem.getCommand(Position.ScoringHighCone, true)
		// .andThen(
		// intakeSubsystem.getAutoConeCommand(false),
		// autoBuilder.fullAuto(PathPlanner.loadPathGroup("C-CnCb-Dock",
		// AUTO.kMediumSlowPath)),
		// AutoBalance.getInstance().getAutoBalanceCommand(false)));
	}

	public Command getAutoCommand() {
		return chooser.getSelected().get();
	}

	public void readTrajectories() {
		File pathsDirectory = new File(Directories.pathsDirectory);
		File[] files = pathsDirectory.listFiles();
		this.trajectories = new Hashtable<String, Trajectory>();
		for (int i = 0; i < files.length; i++) {
			try {
				Path trajectoryPath = files[i].toPath();
				if (Constants.showPrintStatements)
					System.out.println("PATH " + i + ": " + trajectoryPath.toString());
				if (files[i].isFile()) {
					trajectories.put(files[i].getName(), TrajectoryUtil.fromPathweaverJson(trajectoryPath));
				}
			} catch (IOException ex) {
				DriverStation.reportError(
						"Unable to open trajectory: " + files[i].toString(), ex.getStackTrace());
			}
		}
	}

	public static AutoChooser getInstance() {
		if (instance == null) {
			instance = new AutoChooser();
		}
		return instance;
	}
}