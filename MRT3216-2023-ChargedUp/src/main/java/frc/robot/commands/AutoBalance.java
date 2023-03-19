package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.settings.Constants.AUTO_BALANCE;
import frc.robot.settings.Constants.Drivetrain;
import frc.robot.subsystems.SwerveSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class AutoBalance implements Loggable {
    private static AutoBalance instance;
    private int state;
    private int debounceCount;
    private double robotSpeedSlow;
    private double robotSpeedFast;
    private double onChargeStationDegree;
    private double levelDegree;
    private double debounceTime;
    private double forwardCorrectionSpeed;
    private double reverseCorrectionSpeed;
    private boolean isFieldSide;
    private int direction;
    @Log.Exclude
    @Config.Exclude
    private SwerveSubsystem swerveSubsystem;

    private AutoBalance() {
        swerveSubsystem = SwerveSubsystem.getInstance();

        robotSpeedSlow = AUTO_BALANCE.kRobotSpeedSlow;
        robotSpeedFast = AUTO_BALANCE.kRobotSpeedFast;
        onChargeStationDegree = AUTO_BALANCE.kOnChargeStationDegree;
        levelDegree = AUTO_BALANCE.kLevelDegree;
        debounceTime = AUTO_BALANCE.kDebounceTime;
        forwardCorrectionSpeed = AUTO_BALANCE.kForwardCorrectionSpeed;
        reverseCorrectionSpeed = AUTO_BALANCE.kReverseCorrectionSpeed;
        state = 0;
        debounceCount = 0;
    }

    public CommandBase getAutoBalanceCommand(boolean isFieldSide) {
        this.isFieldSide = isFieldSide;
        direction = isFieldSide ? 1 : -1;

        System.out.println("robotSpeedSlow: " + robotSpeedSlow);
        return new ProxyCommand(() -> Commands.run(
                () -> swerveSubsystem.drive(
                        new ChassisSpeeds(direction * autoBalanceRoutine() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                                0, 0))));
    }

    public double getPitch() {
        // Pitch has to be reversed when approaching from the field side
        return isFieldSide ? -swerveSubsystem.getPitch() : swerveSubsystem.getPitch();
    }

    public double getRoll() {
        return swerveSubsystem.getRoll();
    }

    // returns the magnititude of the robot's tilt calculated by the root of
    // pitch^2 + roll^2, used to compensate for diagonally mounted rio
    @Log
    public double getTilt() {
        double pitch = getPitch();
        double roll = getRoll();
        if ((pitch + roll) >= 0) {
            return Math.sqrt(pitch * pitch + roll * roll);
        } else {
            return -Math.sqrt(pitch * pitch + roll * roll);
        }
    }

    public int secondsToTicks(double time) {
        return (int) (time * 50);
    }

    // routine for automatically driving onto and engaging the charge station.
    // returns a value from -1.0 to 1.0, which left and right motors should be set
    // to.
    public double autoBalanceRoutine() {
        switch (state) {
            // drive forwards to approach station, exit when tilt is detected
            case 0:
                System.out.println("Case 0: drive forwards to approach station, exit when tilt is detected");
                System.out.println("robotSpeedFast in case 0: " + robotSpeedFast);
                if (getTilt() > onChargeStationDegree) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 1;
                    debounceCount = 0;
                    return robotSpeedSlow;
                }
                return robotSpeedFast;
            // driving up charge station, drive slower, stopping when level
            case 1:
                System.out.println("Case 1: driving up charge station, drive slower, stopping when level");
                if (getTilt() < levelDegree) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 2;
                    debounceCount = 0;
                    return 0;
                }
                return robotSpeedSlow;
            // on charge station, stop motors and wait for end of auto
            case 2:
                System.out.println("Case 2: on charge station, stop motors and wait for end of auto");
                if (Math.abs(getTilt()) <= levelDegree / 2) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    debounceCount = 0;
                    return 0;
                }
                if (getTilt() >= levelDegree) {
                    return forwardCorrectionSpeed;
                } else if (getTilt() <= -levelDegree) {
                    return reverseCorrectionSpeed;
                }
            case 3:
                return 0;
        }
        return 0;
    }

    public void reset() {
        state = 0;
    }

    // #region Logging

    @Config
    public void setRobotSpeedSlow(double robotSpeedSlow) {
        this.robotSpeedSlow = robotSpeedSlow;
    }

    @Config
    public void setRobotSpeedFast(double robotSpeedFast) {
        this.robotSpeedFast = robotSpeedFast;
    }

    @Config
    public void setOnChargeStationDegree(double onChargeStationDegree) {
        this.onChargeStationDegree = onChargeStationDegree;
    }

    @Config
    public void setLevelDegree(double levelDegree) {
        this.levelDegree = levelDegree;
    }

    @Config
    public void setDebounceTime(double debounceTime) {
        this.debounceTime = debounceTime;
    }

    @Config
    public void setForwardCorrectionSpeed(double forwardCorrectionSpeed) {
        this.forwardCorrectionSpeed = forwardCorrectionSpeed;
    }

    @Config
    public void setReverseCorrectionSpeed(double reverseCorrectionSpeed) {
        this.reverseCorrectionSpeed = reverseCorrectionSpeed;
    }

    // #endregion

    public static AutoBalance getInstance() {
        if (instance == null) {
            // if instance is null, initialize
            instance = new AutoBalance();
        }
        return instance;
    }
}
