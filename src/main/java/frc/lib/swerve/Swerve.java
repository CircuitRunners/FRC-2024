package frc.lib.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Swerve extends SwerveDrivetrain {
  // private SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

  public Swerve(SwerveDrivetrainConstants driveTrainConstants,
      SwerveModuleConstants[] moduleConstants) {
    super(driveTrainConstants, moduleConstants);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModuleState[] getModuleStates() {
   return getState().ModuleStates != null? getState().ModuleStates : new SwerveModuleState[] {new SwerveModuleState(), new SwerveModuleState(),
    new SwerveModuleState(), new SwerveModuleState()};
  }

  public Pose2d getPose2d() {
    return getState().Pose != null ? getState().Pose : new Pose2d();
  }

  public SwerveDriveKinematics getDriveKinematics() {
    return m_kinematics;
  }

  public void resetPose(Pose2d pose) {
    try {
      m_stateLock.writeLock().lock();
      m_odometry.resetPosition(m_pigeon2.getRotation2d(), m_modulePositions, pose);
    } finally {
      m_stateLock.writeLock().unlock();
    }
  }

  public Command zeroGyroCommand() {
    return Commands.runOnce(() -> m_pigeon2.reset());
  }

  public double getPitch() {
    return m_pigeon2.getPitch().getValueAsDouble();
  }
  
  public void targetAngleDrive(Translation2d targetAngle, double forward, double strafe){
    setControl((new SwerveRequest.FieldCentricFacingAngle().withCenterOfRotation(targetAngle).withVelocityX(forward).withVelocityY(strafe)));
  }
  
  public void targetAngleDrive(Rotation2d targetAngle, double forward, double strafe){
    setControl((new SwerveRequest.FieldCentricFacingAngle().withTargetDirection(targetAngle).withVelocityX(forward).withVelocityY(strafe)));
  }
}
