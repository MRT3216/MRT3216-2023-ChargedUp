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

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.settings.Constants.ARM.Position;
import frc.robot.settings.Constants.AUTO;
import frc.robot.settings.Constants.Directories;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

/** Add your docs here. */
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
	@Log
	private double maxTranslationError = 0;
	@Log
	private double maxRotationError = 0;
	@Log
	private double curentTranslationError = 0;
	@Log
	private double currentRotationError = 0;
	@Log
	private double translationSetpoint = 0;
	@Log
	private double rotationSetpoint = 0;
	@Log
	private double currentXError = 0;
	@Log
	private double currentYError = 0;
	@Log
	private double xSetpoint = 0;
	@Log
	private double ySetpoint = 0;

	private AutoChooser() {
		swerveSubsystem = SwerveSubsystem.getInstance();
		armSubsystem = ArmSubsystem.getInstance();
		intakeSubsystem = IntakeSubsystem.getInstance();

		this.populateAutoChooser();

		Shuffleboard.getTab("Driver")
				.add("Auto Mode", chooser)
				.withSize(2, 1) // make the widget 2x1
				.withPosition(7, 0); // place it in the top-left corner
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

		PPSwerveControllerCommand.setLoggingCallbacks(
				(PathPlannerTrajectory activeTrajectory) -> {
					// Log current trajectory
				},
				(Pose2d targetPose) -> {
					// Log target pose
					rotationSetpoint = targetPose.getRotation().getDegrees();
					xSetpoint = targetPose.getX();
					ySetpoint = targetPose.getY();
				},
				(ChassisSpeeds setpointSpeeds) -> {
					// Log setpoint ChassisSpeeds
				},
				(Translation2d translationError, Rotation2d rotationError) -> {
					curentTranslationError = translationError.getNorm();
					currentRotationError = rotationError.getDegrees();
					maxTranslationError = Math.max(maxTranslationError, translationError.getNorm());
					maxRotationError = Math.max(maxRotationError, rotationError.getDegrees());
					currentXError = translationError.getX();
					currentYError = translationError.getY();

					// Log path following error
					// System.out.println("Translation Error: " + translationError.getNorm());
					// System.out.println("Rotation Error: " + rotationError.getDegrees());
				});
	}

	// TODO: Add all of the keys into the map
	// TODO: Add the commands for controlling the systems
	private static HashMap<String, Command> buildEventMapTest() {
		return new HashMap<>(
				Map.ofEntries(
						Map.entry("placeHighCone",
								Commands.print("Placing High Cone")
										.andThen(Commands.waitSeconds(1).andThen(Commands.print("Finished placing")))),

						Map.entry("placeMidCone",
								Commands.print("Placing Mid Cone")
										.andThen(Commands.waitSeconds(1).andThen(Commands.print("Finished placing")))),

						Map.entry("placeHybridCone",
								Commands.print("Placing Hybrid Cone")
										.andThen(Commands.waitSeconds(1).andThen(Commands.print("Finished placing")))),

						Map.entry("placeHighCube",
								Commands.print("Placing High Cube")
										.andThen(Commands.waitSeconds(1).andThen(Commands.print("Finished placing")))),

						Map.entry("placeMidCube",
								Commands.print("Placing Mid Cube")
										.andThen(Commands.waitSeconds(1).andThen(Commands.print("Finished placing")))),

						Map.entry("placeHybridCube",
								Commands.print("Placing Hybrid Cube")
										.andThen(Commands.waitSeconds(1).andThen(Commands.print("Finished placing")))),

						Map.entry("intakeCone",
								Commands.print("Intaking Cone")
										.andThen(Commands.waitSeconds(1).andThen(Commands.print("Finished placing")))),

						Map.entry("intakeCube", Commands.print("Intaking Cube")
								.andThen(Commands.waitSeconds(1).andThen(Commands.print("Finished placing")))),

						Map.entry("stow", Commands.print("Stowing arm")
								.andThen(Commands.print("Finished stowing")))));
	}

	private static HashMap<String, Command> buildEventMapReal() {
		return new HashMap<>(
				Map.ofEntries(
						Map.entry("armToHighCone",
								Commands.print("Moving arm to High Cone")
										.andThen(new ProxyCommand(
												() -> armSubsystem.getCommand(Position.ScoringHighCone, true))
												.andThen(Commands.print("Finished moving arm")))),

						Map.entry("armToMidCone",
								Commands.print("Moving arm to Mid Cone")
										.andThen(new ProxyCommand(
												() -> armSubsystem.getCommand(Position.ScoringMidCone, true))
												.andThen(Commands.print("Finished moving arm")))),

						Map.entry("armToHybridCone",
								Commands.print("Moving arm to Hybrid Cone")
										.andThen(new ProxyCommand(
												() -> armSubsystem.getCommand(Position.ScoringHybrid, true))
												.andThen(Commands.print("Finished moving arm")))),

						Map.entry("armToHighCube",
								Commands.print("Moving arm to High Cube")
										.andThen(new ProxyCommand(
												() -> armSubsystem.getCommand(Position.ScoringHighCube, true))
												.andThen(Commands.print("Finished moving arm")))),

						Map.entry("armToMidCube",
								Commands.print("Moving arm to Mid Cube")
										.andThen(new ProxyCommand(
												() -> armSubsystem.getCommand(Position.ScoringMidCube, true))
												.andThen(Commands.print("Finished moving arm")))),

						Map.entry("armToHybridCube",
								Commands.print("Moving arm to Hybrid Cube")
										.andThen(new ProxyCommand(
												() -> armSubsystem.getCommand(Position.ScoringHybrid, true))
												.andThen(Commands.print("Finished moving arm")))),

						Map.entry("armToIntakeCone",
								Commands.print("Moving arm to Intake Cone")
										.andThen(new ProxyCommand(
												() -> armSubsystem.getCommand(Position.GroundIntakeUprightCone, true))
												.andThen(Commands.print("Finished moving arm")))),

						Map.entry("armToIntakeCube",
								Commands.print("Moving arm to Intake Cube")
										.andThen(new ProxyCommand(
												() -> armSubsystem.getCommand(Position.GroundIntakeCube, true))
												.andThen(Commands.print("Finished moving arm")))),

						Map.entry("intakeCone",
								Commands.print("Intaking Cone")
										.andThen(new ProxyCommand(() -> intakeSubsystem.getAutoConeCommand(true))
												.andThen(Commands.print("Finished intaking")))),

						Map.entry("intakeCube",
								Commands.print("Intaking Cube")
										.andThen(new ProxyCommand(() -> intakeSubsystem.getAutoCubeCommand(true))
												.andThen(Commands.print("Finished intaking")))),

						Map.entry("ejectCone",
								Commands.print("Ejecting Cone")
										.andThen(new ProxyCommand(() -> intakeSubsystem.getAutoConeCommand(false))
												.andThen(Commands.print("Finished ejecting")))),

						Map.entry("ejectCube",
								Commands.print("Eject Cube")
										.andThen(() -> intakeSubsystem.getAutoCubeCommand(false)
												.andThen(Commands.print("Finished ejecting")))),

						Map.entry("stow",
								Commands.print("Stowing arm")
										.andThen(new ProxyCommand(armSubsystem::getStowedCommand)
												.andThen(Commands.print("Finished stowing"))))));
	}

	private void populateAutoChooser() {
		chooser = new SendableChooser<>();

		chooser.setDefaultOption("A-Cn-Leave",
				() -> armSubsystem.getCommand(Position.ScoringHighCone, true)
						.andThen(intakeSubsystem.getAutoConeCommand(false)
								.andThen(autoBuilder
										.fullAuto(PathPlanner.loadPathGroup("A-Cn-Leave", AUTO.kSlowPath)))));

		chooser.addOption("S-CnCb-Leave",
				() -> getScoreHighConeCommand()
						.andThen(autoBuilder.fullAuto(PathPlanner.loadPathGroup("S-CnCb-Leave", AUTO.kFastPath))));

		chooser.addOption("S-CnCb+Cb-Dock",
				() -> getScoreHighConeCommand()
						.andThen(autoBuilder.fullAuto(PathPlanner.loadPathGroup("S-CnCb+Cb-Dock", AUTO.kFastPath))
								.andThen(AutoBalance.getInstance().getAutoBalanceCommand(true))));

		chooser.addOption("M-Cn+Cb-Dock",
				() -> getScoreHighConeCommand()
						.andThen(autoBuilder.fullAuto(PathPlanner.loadPathGroup("M-Cn+Cb-Dock", AUTO.kSlowPath))
								.andThen(AutoBalance.getInstance().getAutoBalanceCommand(false))));

		chooser.addOption("C-CnCb-Leave",
				() -> getScoreHighConeCommand()
						.andThen(autoBuilder.fullAuto(PathPlanner.loadPathGroup("C-CnCb-Leave", AUTO.kFastPath))));

		chooser.addOption("C-CnCb-Dock",
				() -> getScoreHighConeCommand()
						.andThen(autoBuilder.fullAuto(PathPlanner.loadPathGroup("C-CnCb-Dock", AUTO.kFastPath))
								.andThen(AutoBalance.getInstance().getAutoBalanceCommand(false))));
		// TODO: Add the speed constraints to this option to make the speed change over
		// the cable
		chooser.addOption("C-CnCb-LeaveCopy",
				() -> getScoreHighConeCommand()
						.andThen(autoBuilder.fullAuto(PathPlanner.loadPathGroup("S-CnCb-Leave", AUTO.kFastPath))));

		chooser.addOption("C-Test-Dock",
				() -> autoBuilder.fullAuto(PathPlanner.loadPathGroup("C-Test-Dock",
						new PathConstraints(3, 3)))
						.andThen(AutoBalance.getInstance().getAutoBalanceCommand(true)));
	}

	private Command getScoreHighConeCommand() {
		return Commands.print("Scoring high cone")
				.andThen(() -> armSubsystem.getCommand(Position.ScoringHighCone, false)
						.andThen(intakeSubsystem.getAutoConeCommand(false)
								.andThen(Commands.print("Finished scoring"))));
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