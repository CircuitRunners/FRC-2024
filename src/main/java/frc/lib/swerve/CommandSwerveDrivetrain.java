package frc.lib.swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.SwerveConstants;

public class CommandSwerveDrivetrain extends SwerveDrivetrain{
  private SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

  public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants[] moduleConstants) {
    super(driveTrainConstants, moduleConstants);
  }
    private void configurePathPlanner(){



    NamedCommands.registerCommand("autoBalance", Commands.none());
    NamedCommands.registerCommand("brake", this.brakeCommand());
  }
  
  public Command applyRequest(Supplier<SwerveRequest> request){
    return Commands.run(() -> {this.setControl(request.get());});
  }

  public Command brakeCommand(){
    return new RunCommand(() -> this.applyRequest(() -> SwerveConfig.m_Brake));
  }
  
  public ChassisSpeeds getChassisSpeeds(){
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }
  
  public SwerveModuleState[] getModuleStates(){
    return getState().ModuleStates;
  }

  public Pose2d getPose2d(){
    return getState().Pose;
  }
  
  public SwerveDriveKinematics gDriveKinematics(){
    return m_kinematics;
  }


  public void resetPose(Pose2d pose){
    try{
      m_stateLock.writeLock().lock();
      m_odometry.resetPosition(m_pigeon2.getRotation2d(), m_modulePositions, pose); 
    } finally {
      m_stateLock.writeLock().unlock();
    }
  }

  public void increaseLimit(){
    SwerveConstants.limit += SwerveConstants.limit < 1 ? 0.2 : 0;
  }

  public void decreaseLimit(){
    SwerveConstants.limit -= SwerveConstants.limit > 0.2 ? 0.2 : 0;
  }

  public Command increaseLimitCommand(){
    return Commands.runOnce(this::increaseLimit);
  }

  public Command decreaseLimitCommand(){
    return Commands.runOnce(this::decreaseLimit);
  }

  public Command zeroGyroCommand(){
    return Commands.runOnce(() -> m_pigeon2.reset());
  }

  public double getPitch() {
    return m_pigeon2.getPitch().getValueAsDouble();
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }
}
