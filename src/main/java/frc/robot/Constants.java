// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DriverControllerPort = 0;
    public static final int OperatorControllerPort = 1;
  }

  public final static class SwerveBase {
    public final static int DRIVEFRONTLEFT = 1;
    public final static int DRIVEFRONTRIGHT = 2;
    public final static int DRIVEBACKLEFT = 3;
    public final static int DRIVEBACKRIGHT = 4;

    public final static int ROTATIONFRONTLEFT = 21;
    public final static int ROTATIONFRONTRIGHT = 22;
    public final static int ROTATIONBACKLEFT = 23;
    public final static int ROTATIONBACKRIGHT = 24;

    public final static int ENCODERFRONTLEFT = 31;
    public final static int ENCODERFRONTRIGHT = 32;
    public final static int ENCODERBACKLEFT = 33;
    public final static int ENCODERBACKRIGHT = 34;

    public final static double WHEELRADIUS = 2; // inches
    public final static double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(WHEELRADIUS * 2 * Math.PI);
    public final static double DRIVE_MOTOR_ENCODER_RESOLUTION = 2048;
    // For converting 100 milleseconds (heretofore referred to as 'ms') to seconds
    public static final double TIME_CONSTANT_FOR_CONVERSION = 10;
    public static final double GEAR_RATIO = 6.5;
    public static final double TICKS_PER_ROTATION = DRIVE_MOTOR_ENCODER_RESOLUTION * GEAR_RATIO;

    public static final double ticksToMeter(double ticks) {
      return (ticks / TICKS_PER_ROTATION) * WHEEL_CIRCUMFERENCE;
    }

    // A simple conversion formula to turn encoder velocity (sensor units/100ms) to
    // meters per second
    public static final double VELOCITYMETERS = 1 / DRIVE_MOTOR_ENCODER_RESOLUTION * WHEEL_CIRCUMFERENCE * 1
        / GEAR_RATIO * TIME_CONSTANT_FOR_CONVERSION;

    // A simple conversion formula to turn meters per second to encoder velocity
    public static final double VELOCITYSENSOR = DRIVE_MOTOR_ENCODER_RESOLUTION * 1 / WHEEL_CIRCUMFERENCE * GEAR_RATIO
        * 1 / TIME_CONSTANT_FOR_CONVERSION;

    public static final double MAXANGULARSPEED = 1;
    public static final double MAXANGULARACCELERARTION = 1;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(Units.inchesToMeters(-13), Units.inchesToMeters(-13)), // Front Left
        new Translation2d(Units.inchesToMeters(-13), Units.inchesToMeters(13)), // Front Right
        new Translation2d(Units.inchesToMeters(13), Units.inchesToMeters(-13)), // Back Left
        new Translation2d(Units.inchesToMeters(13), Units.inchesToMeters(13))); // Back Right
  }

}
