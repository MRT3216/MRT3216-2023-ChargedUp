package frc.robot.settings;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class Configurations {
    private TalonFXConfiguration swerveDriveMotorConfiguration;
    private TalonFXConfiguration swerveAngleMotorConfiguration;
    private static Configurations instance;

    private Configurations() {
    }

    public static Configurations getInstance() {
        if (instance == null) {
            instance = new Configurations();
        }

        return instance;
    }


    public TalonFXConfiguration getSwerveDriveMotorConfiguration() {
        if (swerveDriveMotorConfiguration == null) {
            swerveDriveMotorConfiguration = new TalonFXConfiguration();
        }

        return swerveDriveMotorConfiguration;
    }

    public TalonFXConfiguration getSwerveAngleMotorConfiguration() {
        if (swerveAngleMotorConfiguration == null) {
            swerveAngleMotorConfiguration = new TalonFXConfiguration();
        }

        return swerveAngleMotorConfiguration;
    }
}