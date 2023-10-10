// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;

public class SwerveDrive extends SubsystemBase implements SwerveSubsystem {
  //I think we can use these values as our speedlimit, if we make them configureable on Shuffleboard 
  public double maxVelocity; 
  public double maxAngularSpeed; 
   
  //Our swerve modules 
  private final SwerveModule frontLeft = new SwerveModule(Constants.SwerveBase.DRIVEFRONTLEFT, Constants.SwerveBase.ROTATIONFRONTLEFT, Constants.SwerveBase.ENCODERFRONTLEFT, 45); 
  private final SwerveModule frontRight = new SwerveModule(Constants.SwerveBase.DRIVEFRONTRIGHT, Constants.SwerveBase.ROTATIONFRONTRIGHT, Constants.SwerveBase.ENCODERFRONTRIGHT, -45); 
  private final SwerveModule backLeft = new SwerveModule(Constants.SwerveBase.DRIVEBACKLEFT, Constants.SwerveBase.ROTATIONBACKLEFT, Constants.SwerveBase.ENCODERBACKLEFT, -45); 
  private final SwerveModule backRight = new SwerveModule(Constants.SwerveBase.DRIVEBACKRIGHT, Constants.SwerveBase.ROTATIONBACKRIGHT, Constants.SwerveBase.ENCODERBACKRIGHT, 45); 
  //Our gyro (used to determine robot heading)
  private final AHRS gyro = new AHRS(SPI.Port.kMXP); // = new AHRS(SPI.Port.kMXP); Causing exception

  private final SwerveDriveOdometry odometry = 
          new SwerveDriveOdometry(Constants.SwerveBase.KINEMATICS, Rotation2d.fromDegrees(gyro.getYaw()), getSwerveModulePositions()); 

  private double pitchOffset = 0.0;
  public SwerveDrive() {
    //I am making the maxVelocity configurable so we can ajdust our "speedlimit"
    Preferences.initDouble("SwerveDrive/Speed Limit", 6); 
    maxVelocity = Preferences.getDouble("SwerveDrive/Speed Limit", 6) ;
    Preferences.initDouble("SwerveDrive/Rotation Speed Limit", 6.5); 
    maxAngularSpeed = Preferences.getDouble("SwerveDrive/Rotation Speed Limit", 6.5) ;
    //It may be useful to reset the gyro like this every boot-up. I believe we did this our old code
    pitchOffset = gyro.getPitch();
    resetGyro();
    enableFieldOriented(isFieldOrientedEnabled); 
    frontLeft.phaseSteeringMotor(); 
    frontRight.phaseSteeringMotor();
    backLeft.phaseSteeringMotor(); 
    backRight.phaseSteeringMotor();
  }

  public void resetGyro(){
    if (gyro != null){
      gyro.reset();
      pitchOffset = gyro.getPitch();
      gyro.setAngleAdjustment(180.0);
    }
  }

  final static double kCollisionThreshold_DeltaG = 1.0f; 
  private double last_world_linear_accel_x;
  private double last_world_linear_accel_y;
  private boolean collisionDetected = false;
  
  public boolean collisionDetected(){
    double curr_world_linear_accel_x = gyro.getWorldLinearAccelX();
    double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
    last_world_linear_accel_x = curr_world_linear_accel_x;
    double curr_world_linear_accel_y = gyro.getWorldLinearAccelY();
    double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
    last_world_linear_accel_y = curr_world_linear_accel_y;

    SmartDashboard.putNumber("SwerveDrive/Collision-X", currentJerkX);
    SmartDashboard.putNumber("SwerveDrive/Collision-Y", currentJerkY);

    return (  ( Math.abs(currentJerkX) > kCollisionThreshold_DeltaG ) ||
              ( Math.abs(currentJerkY) > kCollisionThreshold_DeltaG) );
  }

  @Override
  public void periodic() {
    updateOdometry(); 

   SmartDashboard.putNumber("front left pos", frontLeft.getAngle()); 

    SmartDashboard.putNumber("angle", gyro.getAngle()); 
    SmartDashboard.putNumber("pitch", getPitch());
    SmartDashboard.putNumber("distance x",odometry.getPoseMeters().getX()); 
    SmartDashboard.putNumber("distance y",odometry.getPoseMeters().getY());  

    SmartDashboard.putNumber("SwerveModuleAngle/frontLeft", frontLeft.getAngle()); 
    SmartDashboard.putNumber("SwerveModuleAngle/frontRight", frontRight.getAngle()); 
    SmartDashboard.putNumber("SwerveModuleAngle/backLeft", backLeft.getAngle()); 
    SmartDashboard.putNumber("SwerveModuleAngle/backRight", backRight.getAngle()); 
  }

  public void drive(double xPercent, double yPercent, double rotationPercent){ 
    driveSpeed(xPercent * maxVelocity, yPercent * maxVelocity, rotationPercent * maxAngularSpeed, false);
  } 

  public void driveSpeed(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldOriented){
     SmartDashboard.putNumber("DriveTo/Speed/x", xSpeed);
    // SmartDashboard.putNumber("DriveTo/Speed/y", ySpeed);
    // SmartDashboard.putNumber("DriveTo/Speed/z", rotationSpeed);

    SwerveModuleState[] swerveModuleStates = Constants.SwerveBase.KINEMATICS.toSwerveModuleStates(
      fieldOriented
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, 
                                              ySpeed, 
                                              rotationSpeed, 
                                              Rotation2d.fromDegrees(-getGyroAngle())) 
      : new ChassisSpeeds(xSpeed, 
                          ySpeed, 
                          rotationSpeed)); 
    //This function should limit our speed to the value we set (maxVelocity)

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxVelocity); 

     SmartDashboard.putNumber("angle", getGyroAngle()); 
    
    setModuleStates(swerveModuleStates);
  }

  public void updateOdometry(){ 
    odometry.update( Rotation2d.fromDegrees(gyro.getYaw()), getSwerveModulePositions());
    SmartDashboard.putNumber("SwerveDrive/Pose/X", Units.metersToFeet(odometry.getPoseMeters().getX()));
    SmartDashboard.putNumber("SwerveDrive/Pose/Y", Units.metersToFeet(odometry.getPoseMeters().getY()));
    SmartDashboard.putNumber("SwerveDrive/Pose/Z", odometry.getPoseMeters().getRotation().getDegrees());
  }

  public boolean fieldOriented(){ 
    return (gyro != null && isFieldOrientedEnabled) ? true : false;
  }

  protected boolean isFieldOrientedEnabled = true;
  public void enableFieldOriented(boolean value){
    isFieldOrientedEnabled = value;
    SmartDashboard.putBoolean("Drive by Field Oriented", isFieldOrientedEnabled);
  }
  // FieldOriented and Gyro control mapped to control stick button on a true/false boolean

  public void setStartLocation(Pose2d pose) {
    gyro.setAngleAdjustment(pose.getRotation().getDegrees() - getGyroAngle());
    odometry.resetPosition(Rotation2d.fromDegrees(getGyroAngle()), getSwerveModulePositions(), getPose());
  } 

  public Pose2d getPose(){ 
    Pose2d pose = odometry.getPoseMeters();
    return new Pose2d( pose.getX(), pose.getY(), Rotation2d.fromDegrees(getGyroAngle()) ); 
  } 

  public void setModuleStates(SwerveModuleState[] states){ 
    frontLeft.setDesiredState(states[0], parkingBrake);
    frontRight.setDesiredState(states[1], parkingBrake); 
    backLeft.setDesiredState(states[2], parkingBrake);
    backRight.setDesiredState(states[3], parkingBrake);
  }
  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getSwerveModulePosition(),
      frontRight.getSwerveModulePosition(),
      backLeft.getSwerveModulePosition(),
      backRight.getSwerveModulePosition()
    };
  }
  

  @Override
  public void setPose(Pose2d pose) {
    odometry.resetPosition(Rotation2d.fromDegrees(getGyroAngle()), getSwerveModulePositions(), getPose());
  }

  public SwerveDriveKinematics getKinematics() {
    return Constants.SwerveBase.KINEMATICS ;
  }

  public double getMaxSpeed() {
    return 3.0;
  }

  public double getMaxAcceleration() {
      return 1.0;
  }

  public double getMaxAngularSpeed() {
      return Math.PI*2;
  }

  public double getMaxAngularAcceleration() {
      return Math.PI;
  } 

  public void resetDistance(){
    frontLeft.getDistance();
    frontRight.getDistance();
    backLeft.getDistance();
    backRight.getDistance();
  }
  public double getDistance(){
    double distance = ( Math.abs(frontLeft.getDistance())
                       + Math.abs(frontRight.getDistance())
                       + Math.abs(backLeft.getDistance())
                       + Math.abs(backRight.getDistance())) / 4.0;
    return Constants.SwerveBase.ticksToMeter(distance);
  } 

 public double getGyroAngle(){
  if (gyro == null)
    return 0.0 ;

  return gyro.getAngle();
 } 

// private MedianFilter pitchFilter = new MedianFilter(3);
 public double getPitch(){
  double pitch = gyro.getPitch() - pitchOffset ;
  SmartDashboard.putNumber("UnfilterdPitch", pitch);
  return pitch;
//  return pitchFilter.calculate( gyro.getPitch() - pitchOffset );
 } 

 public void brakeMode(){ 
  frontLeft.brakeMode();
  backLeft.brakeMode(); 
  frontRight.brakeMode(); 
  backRight.brakeMode();
 } 
 private boolean parkingBrake = false; 
 public void parkingBrake(boolean enabled){ 
  parkingBrake = enabled; 
 }

}
