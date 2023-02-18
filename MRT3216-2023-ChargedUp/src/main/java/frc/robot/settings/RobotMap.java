/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.settings;

import edu.wpi.first.wpilibj.I2C;

/** Add your docs here. */
public final class RobotMap {
  public static final class ROBOT {
    public static class DRIVETRAIN {
      // NEO
      public static final int LEFT_FRONT_DRIVE = 3;
      public static final int LEFT_FRONT_ANGLE = 4;
      public static final int LEFT_FRONT_CANCODER = 5;
      public static final int RIGHT_FRONT_DRIVE = 6;
      public static final int RIGHT_FRONT_ANGLE = 7;
      public static final int RIGHT_FRONT_CANCODER = 8;
      public static final int LEFT_REAR_DRIVE = 9;
      public static final int LEFT_REAR_ANGLE = 10;
      public static final int LEFT_REAR_CANCODER = 11;
      public static final int RIGHT_REAR_DRIVE = 12;
      public static final int RIGHT_REAR_ANGLE = 13;
      public static final int RIGHT_REAR_CANCODER = 14;
    }

    public static class SENSORS {
      public static final I2C.Port COLOR_SENSOR = I2C.Port.kMXP;
    }
  }

  public static final class DRIVE_STATION {
    public static final int USB_XBOX_CONTROLLER = 0;
    public static final int USB_JOYSTICK = 1;
  }
}
