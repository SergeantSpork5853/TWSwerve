// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

//CTRE deps
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

//WPILIB deps
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  private final WPI_TalonFX driveMotor; 
  private final WPI_TalonFX steeringMotor; 
  private final CANCoder steeringEncoder;

  // Map ID to offset default values
  private static double[] offsets = {0, 0, 0, 0};
  private static final int ENCODER_BASE = Constants.SwerveBase.ROTATIONFRONTLEFT;

  // The state machine has 3 states:
  // BootState  check the SwerveDrive/Normalize then Normalize or Boot
  // Normalize  normalize the wheels then move to ready
  // Ready      the motors are ready to command
  private enum NormalizeWheels {    
    // This state can only be set by restarting the robot code.
    BootState{
      public NormalizeWheels execute(SwerveModule module) {
        if (Preferences.getBoolean("SwervDrive/Normalize", false))
          return Normalize;

        return Ready;
      };
    },
    // This state is entered only after boot when SwerveDrive/Normalize is true
    Normalize{
      public NormalizeWheels execute(SwerveModule module) {
        module.NormalizeModule();
        Preferences.setBoolean("SwervDrive/Normalize", false);
        return Ready;
      };
    },
    // This state is entered after normaizing or after boot if SwerveDrive/Normalize is false
    Ready{
      public NormalizeWheels execute(SwerveModule module){
        return this;
      };
    };
    public abstract NormalizeWheels execute(SwerveModule module);

    static {
        Preferences.initBoolean("SwervDrive/Normalize", false);
    };
  };

  //Tell the wheel to stop controlling the sterring motor
  private NormalizeWheels normalizeWheels = NormalizeWheels.BootState;

  //I feel the constructor is pretty self-explanatory 
  public SwerveModule(int driveMotorID, int steeringMotorID, int steeringEncoderID, int stopAngle) {
    driveMotor = new WPI_TalonFX(driveMotorID); 
    steeringMotor = new WPI_TalonFX(steeringMotorID); 
    steeringEncoder = new CANCoder(steeringEncoderID);
    
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

    String prefKey = String.format("SwerveModule/Offset_%02d", steeringMotorID);
    Preferences.initDouble(prefKey, offsets[steeringMotorID-ENCODER_BASE]);
    offsets[steeringMotorID-ENCODER_BASE] =  Preferences.getDouble(prefKey, offsets[steeringMotorID-ENCODER_BASE]);

    getDistance();
  } 
  /** Returns the current velocity and rotation angle of the swerve module (in meters per second and 
  radians respectively) */
  public SwerveModuleState getState() { 
    final double angle = getAngle() - offsets[steeringMotor.getDeviceID()-ENCODER_BASE] ;
    return new SwerveModuleState(getVelocityMetersPerSecond(), Rotation2d.fromDegrees( angle )); 
  } 

  public SwerveModulePosition getSwerveModulePosition(){ 
    final double absolute = getAngle(); 
    double angle = absolute - offsets[steeringMotor.getDeviceID()-ENCODER_BASE];
    return new SwerveModulePosition(Constants.SwerveBase.ticksToMeter(driveMotor.getSelectedSensorPosition()), Rotation2d.fromDegrees(angle)); 
  }
  /** Allows us to command the swervemodule to any given veloctiy and angle, ultimately coming from our
  joystick inputs. */
  public void setDesiredState(SwerveModuleState desiredState, boolean parkingbrake) {
      normalizeWheels = normalizeWheels.execute(this);
      if (normalizeWheels != NormalizeWheels.Ready)
        return;

      // SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getAngleNormalized())); 
      SwerveModuleState state = desiredState;      
       /*We need this value back in sensor units/100ms to command the falcon drive motor Note: I do not know
       why the two doubles below need to have 'final' access modifiers*/
       //final double driveOutput =  state.speedMetersPerSecond / velocityMeters;
       //We are using speedMetersPerSecond as a percent voltage value from -1 to 1 
       double driveSpeed = state.speedMetersPerSecond *Constants.SwerveBase.VELOCITYSENSOR; 

      //  final double normalized = getAngleNormalized();
      final double absolute = getAngle();
      double delta = AngleDelta( absolute - offsets[steeringMotor.getDeviceID()-ENCODER_BASE], state.angle.getDegrees() );

      //This is our optimize function
      if (delta > 90.0) {
        delta -= 180.0 ;
        driveSpeed *= -1;
      } else if (delta < -90.0){
        delta += 180.0 ;
        driveSpeed *= -1;
      } 
      
      final double target = AngleToEncoder(absolute + delta);
      
      if(driveSpeed == 0.0){ 
        // if (parkingbrake == true){ 
        //   steeringMotor.set(ControlMode.MotionMagic, AngleToEncoder(-45)); 
        // } else {
        //   steeringMotor.set(ControlMode.PercentOutput, 0.0);
        // }
        driveMotor.set(ControlMode.PercentOutput, 0.0);   
      } else {
        steeringMotor.set(ControlMode.MotionMagic, -target); 
        //driveMotor.set(ControlMode.Velocity, driveSpeed);   
        driveMotor.set(ControlMode.PercentOutput, driveSpeed);
      }
       
  }

  //A getter for the velocity of the drive motor, converted to meters per second.
  public double getVelocityMetersPerSecond(){ 
    return driveMotor.getSelectedSensorVelocity() * Constants.SwerveBase.VELOCITYMETERS;
  } 

  public double getAngle(){ 
    return steeringEncoder.getPosition();
  } 

  public double getAngleNormalized(){
    return Math.IEEEremainder(steeringEncoder.getPosition(), 180.0);
  } 

  protected final static int ENCODER_COUNT = 4096;  
  public static int AngleToEncoder(double deg){
      return (int)((double)deg / 360.0 * (double)ENCODER_COUNT);
  }

  public static double EncoderToAngle(int tick){
      return tick / (double)ENCODER_COUNT * 360.0;
  }

  public static double AngleDelta(double current, double target){
    if (current < 0) current += 360.0;
    if (target < 0) target += 360.0;
    double deltaPos = current - target;
    double deltaNeg = target - current;
    if (Math.abs(deltaPos) < Math.abs(deltaNeg))
        return Math.IEEEremainder(deltaPos,360);
    else
        return Math.IEEEremainder(deltaNeg,360);
  }

  public void NormalizeModule() {
    offsets[steeringMotor.getDeviceID()-ENCODER_BASE] = getAngle();
    String prefKey = String.format("SwerveModule/Offset_%02d", steeringMotor.getDeviceID());
    Preferences.setDouble(prefKey, offsets[steeringMotor.getDeviceID()-ENCODER_BASE]);
  }

  public double distance ;
  public double totalMeters = 0;
  public double getDistance() {
    double current = driveMotor.getSelectedSensorPosition() ;
    // double current_alt = driveMotor.getSensorCollection().getIntegratedSensorPosition();
    double delta = (current - distance);
    distance = current;
    return Constants.SwerveBase.ticksToMeter(delta);
  } 

  public void brakeMode(){ 
    driveMotor.setNeutralMode(NeutralMode.Brake);
  } 
public void phaseSteeringMotor(){ 
  steeringMotor.setSensorPhase(true);
}
  
}
