// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

   
  }
  public static class DriveTrainContants {
    public static final int frontLeftMotorId = 7;
    public static final int backLeftMotorId = 6;
    public static final int frontRightMotorId = 8;
    public static final int backRightMotorId = 5;
    public static final int kDriverControllerPort = 1;
    public static final int JoystickID = 0;
  }

  public static class DriveTrainConstants {

    public static final int frontLeftMotorID = 3;
    public static final int frontRightMotorID = 4;
    public static final int backLeftMotorID = 1;
    public static final int backRightMotorID = 2;
  }
public static class ShooterConstants {
  public static final int frontShooterId = 1;
  public static final int backShooterId = 2;

  public static final double frontShooterSpeed = .5;
  public static final double frontShooterSpeedReverse = -.5;

  public static final double backIndexSpeed = .5;
  public static final double backIndexSpeedReverse = -.5;

  public static final double innerSpeed = .5;
  public static final double outerSpeed = .5;

  public static final double innerSpeedReversed = -.3;
  public static final double outerSpeedReversed = -.3;


}

}
