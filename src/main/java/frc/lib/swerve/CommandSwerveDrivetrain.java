package frc.lib.swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CommandSwerveDrivetrain extends SwerveDrivetrain {
  // private SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

  public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants,
      SwerveModuleConstants[] moduleConstants) {
    super(driveTrainConstants, moduleConstants);
    configurePathPlanner();
  }

  private void configurePathPlanner() {

    NamedCommands.registerCommand("autoBalance", Commands.none());
  }

  public Command applyRequest(Supplier<SwerveRequest> request) {
    return Commands.run(() -> {
      this.setControl(request.get());
    });
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  public SwerveModuleState[] getModuleStates() {
    return getState().ModuleStates != null ? getState().ModuleStates
        : new SwerveModuleState[] { new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(),
            new SwerveModuleState() };
  }

  public Pose2d getPose2d() {
    return getState().Pose != null ? getState().Pose : new Pose2d();
  }

  public SwerveDriveKinematics gDriveKinematics() {
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

  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }
}
