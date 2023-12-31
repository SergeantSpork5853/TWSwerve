// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

//CTRE dependencies
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

//WPILIB dependencies
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



/* This class contains all variables and functions pertaining to a single Swerve Module. A 
 * Swerve Module is a two motor device that allows a wheel's speed and angle to be commanded 
 * separately. 
 */
public class SwerveModule extends SubsystemBase 
  {
    //A Swerve Module has a drive motor, a steering motor, and an encoder angle sensor
    private final WPI_TalonFX driveMotor; 
    private final WPI_TalonFX steeringMotor; 
    private final CANCoder steeringEncoder;

    boolean flipTarget;

     double velocityMeters;
     double ticksPerRotation; 
     double velocityToEncoder;  
    //A value to store the stop angle passed in from the Swerve Module constructor
    private double stopAngle = 0;
    private double gearRatio; 
    //Offsets for the normalize feature of the Swerve Module are stored in an array with default values of 0 
    private static double[] offsets = {0, 0, 0, 0};
    /* 
     * When applying the angle offset to a Swerve Module, it is necessary to increment through each 
     * Swerve Module. By convention, the front left module is used as the first module.  
    */
    private static final int ENCODER_BASE = SwerveConstants.ROTATIONFRONTLEFT;

  /**
   * Constructor for SwerveModule.
   * 
   * @param driveMotorID The CAN bus ID number of the drive motor of the Swerve Module
   * @param steeringMotorID The CAN bus ID number of the steering motor of the Swerve Module
   * @param steeringEncoderID The CAN bus ID number of the encoder angle sensor of the Swerve Module
   * @param stopAngle An angle specified (in degrees) to point the steering motor when the Swerve 
   * Module is not being commanded
  */
  public SwerveModule(int driveMotorID, int steeringMotorID, int steeringEncoderID, int stopAngle) 
    {
      //The drive motor is a CTRE Falcon 500 
      driveMotor = new WPI_TalonFX(driveMotorID); 
      //The steering motor is a CTRE Falcon 500
      steeringMotor = new WPI_TalonFX(steeringMotorID); 
      //The encoder angle sensor is a CTRE CANCoder 
      steeringEncoder = new CANCoder(steeringEncoderID);
      //Set the passed in stop angle's value to the subsystem stop angle 
      this.stopAngle = stopAngle;
      driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 253);//TODO: rethink if we need this speed, I don't think we do
      driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);//This is key to odometry must be around same as code loop
      driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 251);
      driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 241);
      driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 239);
      driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 233);
      driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 229);
      driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255);

      steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 240);//This packet is the motor output, limit switches, faults, we care about none of those
      steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);//This is the sensor feedback, i.e. relative encoder
      steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 251);
      steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 241);
      steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 239);
      steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 233);
      steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 229);
      steeringMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255);    

      steeringEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10);//The default on this is 10, but 20 might be better given our code loop rate
      steeringEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);

      //Initialize preference to store steering motor offsets
      String prefKey = String.format("SwerveModule/Offset_%02d", steeringMotorID);
      Preferences.initDouble(prefKey, offsets[steeringMotorID-ENCODER_BASE]);
      //Assign the stored offsets in Preferences to the swerve module
      offsets[steeringMotorID-ENCODER_BASE] =  Preferences.getDouble(prefKey, offsets[steeringMotorID-ENCODER_BASE]);
    } //End SwerveModule constructor
  
    
  //Converts sensor counts (ticks) to meters. Used for the drive motor.
  private  final double ticksToMeter(double ticks) 
    {
      return (ticks / ticksPerRotation) * SwerveConstants.WHEEL_CIRCUMFERENCE;
    }

  /** 
  * Returns the current velocity and rotation angle of the swerve module 
  * (in meters per second and radians respectively) as a SwerveModuleState
  */ 
  public SwerveModuleState getSwerveModuleState() 
    { 
      final double angle = getAngle() - offsets[steeringMotor.getDeviceID()-ENCODER_BASE];
      return new SwerveModuleState(getVelocityMetersPerSecond(), Rotation2d.fromDegrees(angle)); 
    } 

  /** 
   * Returns the current angle of the encoder angle sensor (in radians) and the total distance traveled 
   * by the drive motor (in meters) as a SwerveModulePosition data type. The date provided by this function
   * is used for wheel odometry.
  */
  public SwerveModulePosition getSwerveModulePosition()
    { 
      final double absolute = getAngle(); 
      double angle = absolute - offsets[steeringMotor.getDeviceID()-ENCODER_BASE];
      return new SwerveModulePosition(ticksToMeter(driveMotor.getSelectedSensorPosition()), Rotation2d.fromDegrees(angle)); 
    }
   
  /**
   * Allows for the Swerve Module to be commanded to any given veloctiy and angle and specifies 
   * if the wheels should use their stop angles
   * @param desiredState The state (speed and angle) to command the module to as SwerveModuleState data type
   * @param useStopAngle Boolean whether or not to command the wheels to their individual stop angles (useful 
   * for avoiding being pushed or to return the wheels to a neutral state when not commanded)
   * 
   */ 
  public void setDesiredState(SwerveModuleState desiredState, boolean useStopAngle) 
    {
      //Assign this internal SwerveModule state value the value of the desired state
      SwerveModuleState state = desiredState;      
       /*  
        * This value is needed back in sensor units/100ms to command the falcon drive motor.
        * speedMetersPerSecond is used as a percent voltage value from -1 to 1 
       */
      double driveSpeed = state.speedMetersPerSecond * velocityToEncoder;
      //The raw, unormalized value of the encoder angle sensor
      final double absolute = getAngle(); 
      /* 
       * Get the difference between the commanded angle 
       * and the reported angle (normalized by the recorded offset value)
       * as a modulo-360 degree number. The encoder angles sensor used by the robot returns a double 
       * that wraps to what is effectively +- infinity, but internal fucntionality requires that this 
       * number be with a 0-360 degree range. 
      */ 
      double delta = AngleDelta( 
      absolute - offsets[steeringMotor.getDeviceID()-ENCODER_BASE], //Subtract the raw value from the recorded offset for the given swerve module
      state.angle.getDegrees()); //This is the angle protion (in degrees) of the desiredState SwerveModuleSate 
      /*
       * If the difference between the normalized current angle and the commanded angle is greater than 
       * 90 degrees, command the steering motor to stay at its current angle and invert the drive motor. This
       * saves time and energy.
      */
      if (delta > 90.0) 
        {
          /* 
           * Change delta to a mirrored version of whatever it was before, effectively commanding the
           * steering motor to stay put.  
          */ 
          delta -= 180.0;
          //Reverse the commanded speed to the drive motor
          driveSpeed *= -1;
        } 
        /* Since delta could be negative, it is 
         * necessary to cover the case where delta is less than or equal to -90 degrees. In effect, it is 
         * the complementary case to the one above.
        */
        else if (delta < -90.0)
          {
            /* 
             * Change delta to a mirrored version of whatever it was before, effectively commanding the
             * steering motor to stay put.  
            */ 
            delta += 180.0 ;
            // Reverse the commanded speed to the drive motor
            driveSpeed *= -1;
          } 
      final double target = AngleToEncoder(absolute + delta);
       if(driveSpeed == 0.0)
        { 
          if (useStopAngle == true)
            { 
              steeringMotor.set(ControlMode.MotionMagic, AngleToEncoder(stopAngle)); 
            } 
          else 
            {
              steeringMotor.set(ControlMode.PercentOutput, 0.0);
            }

           driveMotor.set(ControlMode.PercentOutput, 0.0);

        } 
        else 
          {
            steeringMotor.set(ControlMode.MotionMagic, flipTarget ? -target : target); 
            //driveMotor.set(ControlMode.Velocity, driveSpeed);   
            driveMotor.set(ControlMode.PercentOutput, driveSpeed);
          }
       
  }

/** 
 * A getter for the velocity of the drive motor of the swerve module
 * @return the velocity of the drive motor, converted to meters per second
*/
public double getVelocityMetersPerSecond()
  { 
    return driveMotor.getSelectedSensorVelocity() * velocityMeters;
  } 
/** 
 * A getter for the angle of the steering motor of the swerve module.
 * @return the current angle of the steering motor, in degrees with no normalization.
 * NOTE: In order for this function to work correctly, CANCODERS must utilize "boot to absolute value"
 * boot strategy and be set to range 0 to 360
*/
public double getAngle()
  { 
    return steeringEncoder.getPosition();
  } 

//Convert an angle in degrees to encoder counts (ticks) 
private static int AngleToEncoder(double deg)
  {
      return (int)((double)deg / 360.0 * (double)SwerveConstants.ENCODER_COUNT);
  }

private static double AngleDelta(double current, double target)
  {
      if (current < 0)
      {
        current += 360.0;
      } 
      if (target < 0)
      {
        target += 360.0;
      } 
      double delta = target - current;
      return Math.IEEEremainder(delta, 360);
  }
/**
 * Calculate an offset for the Swerve Module and save it to flash. This offset
 * will allow any direction to be set as the forward direction since encoders
 * almost never tell the Module to point in the correct direction relative to 
 * the robot. 
 */
public void normalizeModule() 
{
    System.out.printf("Normalizing Module %d\n", steeringMotor.getDeviceID());
    offsets[steeringMotor.getDeviceID()-ENCODER_BASE] = getAngle();
    String prefKey = String.format("SwerveModule/Offset_%02d", steeringMotor.getDeviceID());
    Preferences.setDouble(prefKey, offsets[steeringMotor.getDeviceID()-ENCODER_BASE]);
}

/** 
 * Get the distance reported by the drive motor internal sensor and convert it to meters
 * @return the distance traveled by the drive wheel in meters
 */ 
public double getDistance() 
  {  
    return ticksToMeter(driveMotor.getSensorCollection().getIntegratedSensorPosition());
  } 

/**
* Set the drive motor of the Swerve Module to brake mode. "Brake Mode" means the motor will actively 
* fight against any attempts to turn the shaft while otherwise uncommanded using the back EMF 
* (Electromotive Force) generated by turning the shaft. 
*/ 
public void brakeMode()
  { 
    driveMotor.setNeutralMode(NeutralMode.Brake);  
  } 

/** 
 * Invert the steering motor by "phasing" it. Useful if the encoder and the motor are reporting  
 * opposite values relative to each other. For example, if the encoder says that the motor is going
 * counterclockwise when the motor is being commanded clockwise, then the motor and encoder are 
 * "out of phase". Employing this method should correct the issue. 
*/  
public void phaseSteeringMotor()
    { 
      steeringMotor.setSensorPhase(true);
    }

/**
 * Congfigure any settings that vary based on the type of swerve module used. 
 * These settings include gear ratios and encoder internal configurations. 
 * @param moduleType A String name representing the module being used.
 */
public void setModuleSettings(String moduleType)
  {
    switch(moduleType)
    {
      case "geared flipped":
      gearRatio = SwerveConstants.GEAR_RATIO_WCP_GEARED;
      //Because gears rotate opposite each other, the sensor ends up out of phase with the motor. Phasing corrects this. 
      steeringMotor.setSensorPhase(true);
      //The geared swerve steering motor does not need to be inverted
      steeringMotor.setInverted(false);
      //The below settings are critical to correct normalization
      steeringEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
      steeringEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
      //Counterclockwise should always be positive for the encoder
      steeringEncoder.configSensorDirection(false, 0);
      //Command a negative angle (in encoder counts) to the steering motor. Geared swerve needs a mirrored output.
      flipTarget = true;
      System.out.printf("Setting Module %d to %s Settings\n", steeringMotor.getDeviceID(), moduleType);
      break; 
      case "belted flipped":
      gearRatio = SwerveConstants.GEAR_RATIO_WCP_BELTED; 
      //Phasing the sensor allows for inversion of the motor without causing a conflict with the external sensor
      steeringMotor.setSensorPhase(true);
      //The belted swerve steering motor needs to be inverted
      steeringMotor.setInverted(true);
      //The below settings are critical for correct normalization
      steeringEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
      steeringEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
      //Counterclockwise should always be positive for the encoder
      steeringEncoder.configSensorDirection(false, 0);
      /*
       * Command a positive angle (in encoder counts) to the steering motor. For whatever reason
       * belted swerve motionmagic control generates an oscillating output when commanded negatve.
      */
      flipTarget = false; 
      System.out.printf("Setting Module %d to %s Settings\n", steeringMotor.getDeviceID(), moduleType);
      break; 
      default:
      System.out.printf("Module %d not configured properly, check for possible spelling error in moduleType argument to constructor\n", steeringMotor.getDeviceID()); 
    }
  // A simple conversion formula to turn encoder velocity (sensor units/100ms) to
  // meters per second
  velocityMeters = 1 / SwerveConstants.DRIVE_MOTOR_ENCODER_RESOLUTION * SwerveConstants.WHEEL_CIRCUMFERENCE * 1
  / gearRatio * SwerveConstants.TIME_CONSTANT_FOR_CONVERSION;

    // A simple conversion formula to turn meters per second to encoder velocity
    velocityToEncoder = SwerveConstants.DRIVE_MOTOR_ENCODER_RESOLUTION * 1 / SwerveConstants.WHEEL_CIRCUMFERENCE * gearRatio
    * 1 / SwerveConstants.TIME_CONSTANT_FOR_CONVERSION;

    //Converts wheel rotations of the drive motor to sensor counts (ticks)
    ticksPerRotation = SwerveConstants.DRIVE_MOTOR_ENCODER_RESOLUTION * gearRatio;
  }
  
}//End class Swerve Module
