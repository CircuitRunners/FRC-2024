// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerve.Swerve;
import frc.lib.swerve.SwerveConfig;
import frc.lib.utils.FieldUtil;
import frc.lib.utils.PathPlannerUtil;
import frc.robot.Constants.SwerveConstants;
import frc.robot.io.DriverControls;

public class Drive extends SubsystemBase {
  private Swerve swerve;
  private FieldUtil fieldUtil = FieldUtil.getField();

  /** Creates a new Drive. */
  public Drive(Swerve swerve) {
    this.swerve = swerve;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Pose2d targetPose = PathPlannerUtil.getCurrentTargetPose();
    fieldUtil.setSwerveRobotPose(swerve.getPose2d(), swerve.getModuleStates(),
        SwerveConstants.modulePositions);

    fieldUtil.setObjectGlobalPose("Target Pose", targetPose);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    swerve.updateSimState(0.02, 12);
  }

  /* Drivebase Control */
  public void driveFieldCentric(ChassisSpeeds speeds) {
    swerve.setControl(SwerveConfig.drive
        .withVelocityX(speeds.vxMetersPerSecond)
        .withVelocityY(speeds.vyMetersPerSecond)
        .withRotationalRate(speeds.omegaRadiansPerSecond));
  }

  public void driveRobotCentric(ChassisSpeeds speeds) {
    swerve.setControl(SwerveConfig.robotCentric
        .withVelocityX(speeds.vxMetersPerSecond)
        .withVelocityY(speeds.vyMetersPerSecond)
        .withRotationalRate(speeds.omegaRadiansPerSecond));
  }

  public void brake() {
    swerve.setControl(SwerveConfig.brake);
  }

  public Rotation2d getRotation2d() {
    return swerve.getPose2d().getRotation();
  }

  /* Speed Control */
  public void increaseLimit() {
    SwerveConstants.limit += SwerveConstants.limit < 1 ? 0.2 : 0;
  }

  public void decreaseLimit() {
    SwerveConstants.limit -= SwerveConstants.limit > 0.2 ? 0.2 : 0;
  }

  public Command increaseLimitCommand() {
    return Commands.runOnce(this::increaseLimit);
  }

  public Command decreaseLimitCommand() {
    return Commands.runOnce(this::decreaseLimit);
  }

  public Command driveFieldCentricCommand(Supplier<ChassisSpeeds> chassisSpeeds) {
    return run(() -> driveFieldCentric(chassisSpeeds.get()));
  }

  public Command driveRobotCentricCommand(Supplier<ChassisSpeeds> chassisSpeeds) {
    return run(() -> driveRobotCentric(chassisSpeeds.get()));
  }

  public ChassisSpeeds getChassisSpeeds() {
    return swerve.getChassisSpeeds();
  }

  public Command brakeCommand() {
    return run(this::brake);
  }

  public Pose2d getPose() {
    return swerve.getPose2d();
  }

  public void resetPose(Pose2d pose) {
    swerve.resetPose(pose);
  }

  public Command resetGyroCommand(){
    return swerve.zeroGyroCommand();
  }
  
  public void targetAngleDrive(Translation2d targetAngle, DriverControls controls) {
    swerve.targetAngleDrive(targetAngle, controls.driveForward(), controls.driveStrafe());
  }
  
  public void targetAngleDrive(Rotation2d targetAngle, DriverControls controls) {
    swerve.targetAngleDrive(targetAngle, controls.driveForward(), controls.driveStrafe());
  }
}
