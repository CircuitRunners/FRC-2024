// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drive;

public class AimAtSpeaker extends Command {
  /** Creates a new AimAtSpeaker. */
  private Drive drive;  
  public AimAtSpeaker(Drive swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = swerve;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d difference = new Pose2d(AutoConstants.speakerPose.getX() -  drive.getPose().getX(), AutoConstants.speakerPose.getY() - drive.getPose().getY(), AutoConstants.speakerPose.getRotation().minus(drive.getRotation2d()));
    drive.targetAngleDrive(difference.getRotation());
  } 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
