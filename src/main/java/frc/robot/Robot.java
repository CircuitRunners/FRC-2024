// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.swerve.SwerveConfig;
import frc.lib.utils.PathPlannerUtil;
import frc.robot.Constants.DriverConstants;
import frc.robot.io.DriverControls;
import frc.robot.subsystems.Drive;

public class Robot extends TimedRobot {
  private Drive drive;
  private DriverControls driverControls;
  private Command m_autonomousCommand;
  private final SendableChooser<Supplier<Command>> autoChooser = new SendableChooser<Supplier<Command>>();

  @Override
  public void robotInit() {
    configureSubsystems();
    configureAutos();
    configureBindings();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoChooser.getSelected().get();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  private void configureAutos() {
    PathPlannerUtil.configure(drive);
    autoChooser.setDefaultOption("Do Nothing", () -> Commands.none());
    PathPlannerUtil.getAutos().forEach(path -> {
      autoChooser.addOption(path, () -> PathPlannerUtil.getAutoCommand(path));
    });
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    driverControls = new DriverControls(DriverConstants.driverPort);
    drive.setDefaultCommand(
        drive.driveFieldCentricCommand(() -> SwerveConfig.toChassisSpeeds(driverControls, drive)));
    driverControls.increaseLimit().onTrue(drive.increaseLimitCommand());
    driverControls.decreaseLimit().onTrue(drive.decreaseLimitCommand());
    driverControls.robotRelative()
        .whileTrue(drive.driveRobotCentricCommand(() -> SwerveConfig.toChassisSpeeds(driverControls, drive)));
    driverControls.resetGyro().onTrue(drive.resetGyroCommand());
    driverControls.toAmp().whileTrue(PathPlannerUtil.getAutoCommand("Anywhere To Amp"));
    // driverControls.toAmp().whileTrue(Commands.print("Amp"));
  }

  private void configureSubsystems() {
    drive = new Drive(SwerveConfig.getConfiguredDrivetrain());
  }

}
