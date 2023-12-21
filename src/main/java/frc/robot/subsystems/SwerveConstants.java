package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;

public final class SwerveConstants 
{
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
    public static final int ENCODER_COUNT = 4096;  

    public static final double MAXANGULARSPEED = 1;
    public static final double MAXANGULARACCELERARTION = 1; 

    public static final double GEAR_RATIO_WCP_BELTED = 6.5; 
    public static final double GEAR_RATIO_WCP_GEARED = 6.5; 

}
