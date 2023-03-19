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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.settings.Constants.AUTO;
import frc.robot.settings.Constants.Directories;
import frc.robot.settings.Constants.ARM.Position;
import frc.robot.subsystems.ArmSubsystem;
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
	private ArmSubsystem armSubsystem;
	private Dictionary<String, Trajectory> trajectories;
	private SendableChooser<Supplier<Command>> chooser;
	@Log.Exclude
	@Config.Exclude
	private SwerveSubsystem swerveSubsystem;
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

		this.populateAutoChooser();

		Shuffleboard.getTab("Driver")
				.add("Auto Mode", chooser)
				.withSize(2, 1) // make the widget 2x1
				.withPosition(7, 0); // place it in the top-left corner
	}

	public void init() {
		eventMap = buildEventMap();

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
	private static HashMap<String, Command> buildEventMap() {
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
								.andThen(Commands.waitSeconds(1).andThen(Commands.print("Finished placing"))))));

		// Map.entry("stow", AutoBalance.getInstance().getAutoBalanceCommand()
		// .andThen(Commands.waitSeconds(1).andThen(Commands.print("Finished
		// stowing")))),

		// Map.entry("autoBalance", AutoBalance.getInstance().getAutoBalanceCommand()
		// .andThen(Commands.waitSeconds(0).andThen(Commands.print("Finished
		// balancing"))))));
	}

	// TODO: Find a naming system and put the names of the files and tabs to that
	// TODO: Look into not needing the ".andThen" for docking
	private void populateAutoChooser() {
		chooser = new SendableChooser<>();

		chooser.setDefaultOption("A-Cn-Leave",
				() -> armSubsystem.getCommand(Position.ScoringHighCone)
						.andThen(autoBuilder.fullAuto(PathPlanner.loadPath("A-Cn-Leave", AUTO.kSlowPath))));

		chooser.addOption("S-CnCb-Leave",
				() -> armSubsystem.getCommand(Position.ScoringHighCone)
						.andThen(autoBuilder.fullAuto(PathPlanner.loadPath("S-CnCb-Leave", AUTO.kFastPath))));

		chooser.addOption("S-CnCb+Cb-Dock",
				() -> armSubsystem.getCommand(Position.ScoringHighCone)
						.andThen(autoBuilder.fullAuto(PathPlanner.loadPath("S-CnCb+Cb-Dock", AUTO.kFastPath))
								.andThen(AutoBalance.getInstance().getAutoBalanceCommand(false))));

		chooser.addOption("M-Cn+Cb-Dock",
				() -> armSubsystem.getCommand(Position.ScoringHighCone)
						.andThen(autoBuilder.fullAuto(PathPlanner.loadPath("M-Cn+Cb-Dock", AUTO.kMediumPath))
								.andThen(AutoBalance.getInstance().getAutoBalanceCommand(false))));

		chooser.addOption("C-CnCb-Leave",
				() -> armSubsystem.getCommand(Position.ScoringHighCone)
						.andThen(autoBuilder.fullAuto(PathPlanner.loadPath("C-CnCb-Leave", AUTO.kFastPath))));

		chooser.addOption("C-CnCb-Dock",
				() -> armSubsystem.getCommand(Position.ScoringHighCone)
						.andThen(autoBuilder.fullAuto(PathPlanner.loadPath("C-CnCb-Dock", AUTO.kFastPath))
								.andThen(AutoBalance.getInstance().getAutoBalanceCommand(false))));

		chooser.addOption("C-Test-Dock",
				() -> autoBuilder.fullAuto(PathPlanner.loadPathGroup("C-Test-Dock",
						new PathConstraints(3, 3)))
						.andThen(AutoBalance.getInstance().getAutoBalanceCommand(false)));
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