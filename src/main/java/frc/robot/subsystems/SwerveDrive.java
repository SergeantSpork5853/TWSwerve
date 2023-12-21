// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;

public class SwerveDrive extends SubsystemBase implements SwerveSubsystem { 

  double maxAngularSpeed; 
  double maxVelocity;
  double headingAdjustment = 0;  
  String moduleType;
  boolean flipped;  
  boolean debugMode; 
  double pitchOffset = 0.0;
  
  public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(Units.inchesToMeters(-13), Units.inchesToMeters(-13)), // Front Left
    new Translation2d(Units.inchesToMeters(-13), Units.inchesToMeters(13)), // Front Right
    new Translation2d(Units.inchesToMeters(13), Units.inchesToMeters(-13)), // Back Left
    new Translation2d(Units.inchesToMeters(13), Units.inchesToMeters(13))); // Back Right

  private DigitalInput normalizeSwitch = new DigitalInput(0);  
  /**
   * The constructor for the swerve drive
   * @param maxVelocity The desired max velocity of the robot in meters per second
   * @param maxAngularSpeed The desired max angular speed of the robot in radians per second 
   * @param moduleType Pass in "geared" for wcp gear swerve modules or "belted" for belted modules
   * @param flipped True for a flipped (upside down) swerve module and false for a non-flipped module
   * @param kinematics A kinematics object containing the locations of each swerve module relative to robot center 
   * @param headingAdjustment An angle offset in degrees to be applied every gryo reset (including robot start up)
   * @param debugMode If false, suppress non-essential SmartDashboard publications to save memory
   */
  public SwerveDrive(double maxVelocity, double maxAngularSpeed, String moduleType, boolean flipped, double headingAdjustment, boolean debugMode) 
  {
    this.maxAngularSpeed = maxAngularSpeed; 
    this.maxVelocity = maxVelocity; 
    this.moduleType = moduleType; 
    this.flipped = flipped; 
    this.debugMode = debugMode;
    this.headingAdjustment = headingAdjustment;  
    resetGyro();
    enableFieldOriented(isFieldOrientedEnabled);  
    frontLeft.setGearRatioDependentConstants(moduleType);
    frontRight.setGearRatioDependentConstants(moduleType);
    backLeft.setGearRatioDependentConstants(moduleType);
    backRight.setGearRatioDependentConstants(moduleType);  

    if(moduleType == "geared")
    {
    frontLeft.phaseSteeringMotor(); 
    frontRight.phaseSteeringMotor();
    backLeft.phaseSteeringMotor(); 
    backRight.phaseSteeringMotor();
    }
          
  }
  private final SwerveModule frontLeft  = new SwerveModule(SwerveConstants.DRIVEFRONTLEFT, SwerveConstants.ROTATIONFRONTLEFT, SwerveConstants.ENCODERFRONTLEFT, 45); 
  private final SwerveModule frontRight =  new SwerveModule(SwerveConstants.DRIVEFRONTRIGHT, SwerveConstants.ROTATIONFRONTRIGHT, SwerveConstants.ENCODERFRONTRIGHT, -45); 
  private final SwerveModule backLeft   =  new SwerveModule(SwerveConstants.DRIVEBACKLEFT, SwerveConstants.ROTATIONBACKLEFT, SwerveConstants.ENCODERBACKLEFT, -45); 
  private final SwerveModule backRight  = new SwerveModule(SwerveConstants.DRIVEBACKRIGHT, SwerveConstants.ROTATIONBACKRIGHT, SwerveConstants.ENCODERBACKRIGHT, 45); 
  //Our gyro (used to determine robot heading)
  private final AHRS gyro = new AHRS(SPI.Port.kMXP); // = new AHRS(SPI.Port.kMXP); Causing exception

  private final SwerveDriveOdometry odometry = 
  new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(gyro.getYaw()), getSwerveModulePositions()); 
  public void resetGyro()
  {
    if (gyro != null)
    {
      gyro.reset();
      pitchOffset = gyro.getPitch();
      gyro.setAngleAdjustment(headingAdjustment);
    }
  }

  @Override
  public void periodic() {
    updateOdometry(); 

    SmartDashboard.putNumber("gyro angle", gyro.getAngle()); 
    SmartDashboard.putNumber("pitch", getPitch());

    if(!normalizeSwitch.get())
    {
      normalizeModules();
    }

    if(debugMode)
    {
    SmartDashboard.putNumber("SwerveModuleAngle/frontLeft", frontLeft.getAngle()); 
    SmartDashboard.putNumber("SwerveModuleAngle/frontRight", frontRight.getAngle()); 
    SmartDashboard.putNumber("SwerveModuleAngle/backLeft", backLeft.getAngle()); 
    SmartDashboard.putNumber("SwerveModuleAngle/backRight", backRight.getAngle()); 

    SmartDashboard.putNumber("SwerveDrive/Pose/X", Units.metersToFeet(odometry.getPoseMeters().getX()));
    SmartDashboard.putNumber("SwerveDrive/Pose/Y", Units.metersToFeet(odometry.getPoseMeters().getY()));
    SmartDashboard.putNumber("SwerveDrive/Pose/Z", odometry.getPoseMeters().getRotation().getDegrees());
    }
  }

  public void drive(double xPercent, double yPercent, double rotationPercent)
  { 
    driveSpeed(xPercent * maxVelocity, yPercent * maxVelocity, rotationPercent * maxAngularSpeed, false);
  } 

  public void driveSpeed(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldOriented)
  {

    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
      fieldOriented
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed * maxVelocity, 
                                              ySpeed * maxVelocity, 
                                              rotationSpeed * maxAngularSpeed, 
                                              Rotation2d.fromDegrees(-getGyroAngle())) 
      : new ChassisSpeeds(xSpeed, 
                          ySpeed, 
                          rotationSpeed)); 
    //This function should limit our speed to the value we set (maxVelocity)
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxVelocity); 
    //System.out.println("xSpeed" +xSpeed);
    setModuleStates(swerveModuleStates);
  }

  public void updateOdometry(){ 
   odometry.update( Rotation2d.fromDegrees(gyro.getYaw()), getSwerveModulePositions());
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

  public void setModuleStates(SwerveModuleState[] states)
  { 
    frontLeft.setDesiredState(states[0], parkingBrake);
    frontRight.setDesiredState(states[1], parkingBrake); 
    backLeft.setDesiredState(states[2], parkingBrake);
    backRight.setDesiredState(states[3], parkingBrake);
  }
  public SwerveModulePosition[] getSwerveModulePositions() 
  {
    return new SwerveModulePosition[] 
    {
      frontLeft.getSwerveModulePosition(),
      frontRight.getSwerveModulePosition(),
      backLeft.getSwerveModulePosition(),
      backRight.getSwerveModulePosition()
    };
  }
  
  @Override
  public void setPose(Pose2d pose) 
  {
    odometry.resetPosition(Rotation2d.fromDegrees(getGyroAngle()), getSwerveModulePositions(), getPose());
  }

  public SwerveDriveKinematics getKinematics() 
  {
    return kinematics;
  }

  public double getMaxSpeed() 
  {
    return 3.0;
  }

  public double getMaxAcceleration() 
  {
      return 1.0;
  }

  public double getMaxAngularSpeed() 
  {
      return Math.PI*2;
  }

  public double getMaxAngularAcceleration() 
  {
      return Math.PI;
  } 

 public double getGyroAngle()
 {
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

 public void brakeMode()
 { 
  frontLeft.brakeMode();
  backLeft.brakeMode(); 
  frontRight.brakeMode(); 
  backRight.brakeMode();
 }

 private boolean parkingBrake = false; 
 public void parkingBrake(boolean enabled){ 
  parkingBrake = enabled; 
 }
 
 public void normalizeModules()
 {
  frontLeft.normalizeModule();
  backLeft.normalizeModule(); 
  frontRight.normalizeModule(); 
  backRight.normalizeModule();
 }
}
