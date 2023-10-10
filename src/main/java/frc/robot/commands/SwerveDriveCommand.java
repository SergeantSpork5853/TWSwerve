// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class SwerveDriveCommand extends CommandBase {
  private SwerveDrive driveBase;
  private DoubleSupplier getXSpeed, getYSpeed, getRotationSpeed; 

  public SwerveDriveCommand(DoubleSupplier getXSpeed, 
                            DoubleSupplier getYSpeed, 
                            DoubleSupplier getRotationSpeed, 
                            SwerveDrive driveBase) {

    this.getXSpeed = getXSpeed; 
    this.getYSpeed = getYSpeed; 
    this.getRotationSpeed = getRotationSpeed; 
    this.driveBase = driveBase; 
    addRequirements(driveBase);
    driveBase.resetDistance();
  }

  @Override
  public void execute() {
    driveBase.driveSpeed(getXSpeed.getAsDouble(), getYSpeed.getAsDouble(), getRotationSpeed.getAsDouble()*0.001, true);

    SmartDashboard.putNumber("Drive/Distance", Units.metersToFeet( driveBase.getDistance() ));
  }

  @Override
  public void end(boolean interrupted) {
    driveBase.drive(0.0, 0.0, 0.0);
  }
}