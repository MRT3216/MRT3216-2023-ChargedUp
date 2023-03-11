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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.autoProcedures.PlaceConeAndDrive;
import frc.robot.commands.auto.autoProcedures.PlaceCubeAndDrive;
import frc.robot.commands.auto.autoProcedures.PlaceHighCone;
import frc.robot.commands.auto.autoProcedures.PlaceHighCube;
import frc.robot.settings.Constants.Directories;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import io.github.oblarg.oblog.annotations.Config;

/** Add your docs here. */
public class AutoChooser {

	private static AutoChooser instance;
	private Dictionary<String, Trajectory> trajectories;
	@Config(name = "Auto Chooser", tabName = "Driver", rowIndex = 3, columnIndex = 0, width = 3, height = 1)
	private SendableChooser<Supplier<Command>> chooser;
	private SwerveSubsystem swerveSystem;
	private ArmSubsystem armSystem;
	private LimelightSubsystem limelightSystem;
	private IntakeSubsystem intakeSystem;

	private AutoChooser() {
		chooser = new SendableChooser<>();
		chooser.setDefaultOption("Do Nothing", () -> new WaitCommand(0));
		this.swerveSystem = SwerveSubsystem.getInstance();
		this.armSystem = ArmSubsystem.getInstance();
		this.limelightSystem = LimelightSubsystem.getInstance();
		this.intakeSystem = IntakeSubsystem.getInstance();
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

		chooser.addOption("High Cube",
				() -> new PlaceHighCube(this.swerveSystem, this.armSystem, this.intakeSystem, this.limelightSystem));

		chooser.addOption("High Cone",
				() -> new PlaceHighCone(this.swerveSystem, this.armSystem, this.intakeSystem, this.limelightSystem));

		// chooser.addOption("M Top Cone and Charge",
		// () -> new SequentialCommandGroup(
		// new MCharge(this.swerveSystem, this.armSystem, this.intakeSystem,
		// this.limelightSystem)));

		chooser.addOption("Place Cube and Drive",
				() -> new SequentialCommandGroup(
						new PlaceCubeAndDrive(this.swerveSystem, this.armSystem, this.intakeSystem,
								this.limelightSystem)));

		chooser.addOption("Place Cone and Drive",
				() -> new SequentialCommandGroup(
						new PlaceConeAndDrive(this.swerveSystem, this.armSystem, this.intakeSystem,
								this.limelightSystem)));

		SmartDashboard.putData(chooser);
	}

	public Command getAutoCommand() {
		return chooser.getSelected().get();
	}
}
