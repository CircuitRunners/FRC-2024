// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.io.DriverControls;
import frc.robot.subsystems.Drive;

public class AimAtSpeaker extends Command {
  /** Creates a new AimAtSpeaker. */
  private double targetAngle;
  private Translation2d difference;
  private Drive drive;  
  private DriverControls controls;
  private boolean rotateAroundPose; 

  public AimAtSpeaker(Drive swerve, DriverControls controls, boolean rotateAroundPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = swerve;
    addRequirements(drive);
    this.controls = controls;
    this.rotateAroundPose = rotateAroundPose;
  }

  public AimAtSpeaker(Drive swerve, DriverControls controls) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = swerve;
    addRequirements(drive);
    this.controls = controls;
    this.rotateAroundPose = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(DriverStation.getAlliance().get().equals(Alliance.Blue)){
      difference = FieldConstants.SpeakerK.kBlueCenterOpening.toTranslation2d().minus(drive.getPose().getTranslation());
      targetAngle = Math.atan2(difference.getY(), difference.getX());
    }
    else {
      difference = FieldConstants.SpeakerK.kRedCenterOpening.toTranslation2d().minus(drive.getPose().getTranslation());
      targetAngle = Math.atan2(difference.getY(), difference.getX());
    }
    
    if (rotateAroundPose) drive.targetAngleDrive(difference, controls);
    else drive.targetAngleDrive(Rotation2d.fromRadians(targetAngle), controls);
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
