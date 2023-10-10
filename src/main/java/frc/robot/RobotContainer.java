// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here... 
  private final SwerveDrive driveBase = new SwerveDrive(); 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(OperatorConstants.DriverControllerPort);
  private final CommandXboxController operator = new CommandXboxController(OperatorConstants.OperatorControllerPort);
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    ConfigShuffleboard();

    driveBase.setDefaultCommand(new SwerveDriveCommand(this::getXSpeed, 
                                                       this::getYSpeed, 
                                                       this::getRotationSpeed, driveBase));
  }

  public void disabledInit() {
      }
  
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  // private enum InputType { Cube, Cone };
  //private InputType inputType = InputType.Cone;
  // private void setInputType(InputType type) {
  //   inputType = type;
  //   SmartDashboard.putBoolean("IntakeType/Cone", inputType == InputType.Cone);
  //   SmartDashboard.putBoolean("IntakeType/Cube", inputType == InputType.Cube);
  // } 

  // private void toggleInputType() {
  //   if (inputType == InputType.Cone)
  //     setInputType(InputType.Cube);
  //   else
  //     setInputType(InputType.Cone);
  // }

  private void configureBindings() {
    
  }
  
  private void ConfigShuffleboard(){
    SmartDashboard.putNumber("Swerve/X", 0.0);
    SmartDashboard.putNumber("Swerve/Y", 0.0);
    SmartDashboard.putNumber("Swerve/Z", 0.0);

    SwerveDriveCommand swerveDrive = new SwerveDriveCommand(() -> { return SmartDashboard.getNumber("Swerve/X", 0.0); },
                                                            () -> { return SmartDashboard.getNumber("Swerve/Y", 0.0); },
                                                            () -> { return SmartDashboard.getNumber("Swerve/Z", 0.0); },
                                                            driveBase);

    SmartDashboard.putData("Swerve/Start", swerveDrive);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
  double getXSpeed(){ 
    int pov = driver.getHID().getPOV();
    double finalX;

    if ( pov == 0 )
      finalX = -0.075;
    else if(pov == 180)
      finalX = 0.075;
    else if (Math.abs(driver.getLeftY()) <= 0.1)
      finalX = 0.0;
    else
      finalX = driver.getLeftY() * 0.75 * (1.0 + driver.getLeftTriggerAxis());
    
    return -finalX;
  }

  public double getYSpeed(){ 
    int pov = driver.getHID().getPOV();

    double finalY;
    if ( pov == 270 || pov == 315 || pov == 225)
      finalY = -0.05;
    else if(pov == 90 || pov == 45 || pov == 135)
      finalY = 0.05;
    else if (Math.abs(driver.getLeftX()) <= 0.1)
      finalY = 0.0;
    else
      finalY = driver.getLeftX() * 0.75 * (1.0 + driver.getLeftTriggerAxis());
    
    return -finalY; 
  } 
  
  public double getRotationSpeed(){ 
    double finalRotation;

    // if (Math.abs(driver.getRightX()) <= 0.1)
    //   finalRotation = Math.abs(operator.getRightX()) <= 0.1 ? 0.0 : operator.getRightX() * .5 / (1.0 + operator.getRightTriggerAxis());
    // else
      finalRotation = driver.getRightX() * .5 / (1.0 + (4.0 * driver.getRightTriggerAxis()));

      if (Math.abs(finalRotation) < 0.1)
        finalRotation = 0.0;
    
    return finalRotation;
  }

}


