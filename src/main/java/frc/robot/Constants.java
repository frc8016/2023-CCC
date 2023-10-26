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
    public static final int JoystickID = 0;
    public static int JOYSTICK_PORT = 1;
    public static int JOYSTICK_Y_AXIS = 1;
    public static int JOYSTICK_X_AXIS = 0;
  }

  public static class DriveTrainConstants {
    public static final int frontLeftMotorID = 3;
    public static final int backLeftMotorID = 1;
    public static final int frontRightMotorID = 4;
    public static final int backRightMotorID = 2;
  }

  public static class ShooterConstants {
    // left motor
    public static final int frontShooterId = 7;
    // right motor
    public static final int backShooterId = 8;

    public static final double frontShooterSpeed = .5;

    public static final double backIndexSpeed = 1;

    public static final double innerOuterSpeed = 1;

    public static final double innerOuterSpeedReversed = -.1;
  }

  public static class ArmConstants {
    // motors
    public static final int leftArmMotorID = 5;
    public static final int rightArmMotorID = 6;

    public static final int MOTOR_CURRENT_LIMIT = 20;
    // encoders
    public static final int RELATIVE_ENCODER_A = 9;
    public static final int RELATIVE_ENCODER_B = 7;
    public static final int ABSOLUTE_ENCODER_PORT = 8;

    public static final double relativeEncoderDistancePerPulse = 2 * Math.PI / 1024;
    public static final double dutyCycleEncoderDistancePerRotation = 2 * Math.PI;

    public static final double kp = 0.;
    public static final double ki = 0.;
    public static final double kd = 0;

    public static final double kMaxVelocityRadPerSecond = Math.PI / 4;
    public static final double kMaxAccelerationRadPerSecondSquared = 1;
    public static final double kAngleOfOffset = 0;

    public static final double ks = 0.16993;
    public static final double kv = 3;
    public static final double ka = 0.0094614;
    public static final double kg = 0.15;
  }
}
