// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Dictionary;
import java.util.Hashtable;
import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoCommands;
import frc.robot.settings.Constants.Directories;

/** Add your docs here. */
public class AutoChooser {

	private static AutoChooser instance;
	private Dictionary<String, Trajectory> trajectories;
	private SendableChooser<Supplier<Command>> chooser;
	private AutoCommands auto;

	private AutoChooser() {
		chooser = new SendableChooser<>();
		chooser.setDefaultOption("Do Nothing", () -> new WaitCommand(0));
		auto = AutoCommands.getInstance();

		Shuffleboard.getTab("Driver")
				.add("Auto Mode", chooser)
				.withSize(2, 1) // make the widget 2x1
				.withPosition(7, 0); // place it in the top-left corner
	}

	public static AutoChooser getInstance() {
		if (instance == null) {
			instance = new AutoChooser();
		}
		return instance;
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

	public void populateAutoChooser() {
		chooser.addOption("Drive Backwards", () -> auto.getDriveBackwardsCommand());
		chooser.addOption("Place and Drive", () -> auto.getPlaceAndDriveCommand());
		/*
		 * chooser.addOption("SS 2 Cone Charge",
		 * () -> new ConditionalCommand(getAutoCommand(), getAutoCommand(), null)
		 * );
		 * 
		 * chooser.addOption("W 2 Cone Charge",
		 * () -> new ConditionalCommand(getAutoCommand(), getAutoCommand(), null)
		 * );
		 * 
		 * chooser.addOption("SS Cone Cube Charge",
		 * () -> new ConditionalCommand(getAutoCommand(), getAutoCommand(), null)
		 * );
		 * 
		 * chooser.addOption("W Cone Cube Charge",
		 * () -> new ConditionalCommand(getAutoCommand(), getAutoCommand(), null)
		 * );
		 */

		// chooser.addOption("High Cube",
		// () -> new PlaceHighCube(this.swerveSystem, this.armSystem, this.intakeSystem,
		// this.limelightSystem));

		// chooser.addOption("High Cone",
		// () -> new PlaceHighCone(this.swerveSystem, this.armSystem, this.intakeSystem,
		// this.limelightSystem));

		// chooser.addOption("M Top Cone and Charge",
		// () -> new SequentialCommandGroup(
		// new MCharge(this.swerveSystem, this.armSystem, this.intakeSystem,
		// this.limelightSystem)));

		// chooser.addOption("Place Cube and Drive",
		// () -> new SequentialCommandGroup(
		// new PlaceCubeAndDrive(this.swerveSystem, this.armSystem, this.intakeSystem,
		// this.limelightSystem)));

		// chooser.addOption("Place Cone and Drive",
		// () -> new SequentialCommandGroup(
		// new PlaceConeAndDrive(this.swerveSystem, this.armSystem, this.intakeSystem,
		// this.limelightSystem)));

		SmartDashboard.putData(chooser);
	}

	public Command getAutoCommand() {
		return chooser.getSelected().get();
	}
}
