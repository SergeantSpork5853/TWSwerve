package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SwerveSubsystem extends Subsystem{
    public void drive(double xSpeed, double ySpeed, double zSpeed);
    public void setPose(Pose2d pose);
    public Pose2d getPose();
    public double getPitch();
    public void setModuleStates(SwerveModuleState[] states);

    public SwerveDriveKinematics getKinematics();
    public double getMaxSpeed();
    public double getMaxAcceleration(); 
    public double getMaxAngularSpeed(); 
    public void brakeMode(); 
    public double getMaxAngularAcceleration();
    public void parkingBrake(boolean enabled);
}
