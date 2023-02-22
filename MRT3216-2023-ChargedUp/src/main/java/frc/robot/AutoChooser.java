// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.settings.Constants.Directories;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Dictionary;
import java.util.Hashtable;
import java.util.function.Supplier;

/** Add your docs here. */
public class AutoChooser {

	private static AutoChooser instance;
	private Dictionary<String, Trajectory> trajectories;
	private SendableChooser<Supplier<Command>> chooser;
	private SwerveSubsystem swerveSystem;

	private AutoChooser() {
		chooser = new SendableChooser<>();
		chooser.setDefaultOption("Do Nothing", () -> new WaitCommand(0));
		this.swerveSystem = SwerveSubsystem.getInstance();
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

		SmartDashboard.putData(chooser);
	}

	public Command getAutoCommand() {
		return chooser.getSelected().get();
	}
}
