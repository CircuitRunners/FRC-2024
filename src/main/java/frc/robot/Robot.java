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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.swerve.SwerveConfig;
import frc.lib.utils.PathPlannerUtil;
import frc.robot.Constants.DriverConstants;
import frc.robot.io.DriverControls;
import frc.robot.io.OperatorControls;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {
  private Drive drive;
  private Elevator elevator;
  private Intake intake;
  private Shooter shooter;
  private DriverControls driverControls;
  private OperatorControls operatorControls;
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

    // ------------------------------- DRIVER CONTROLS ---------------------------------------------------------
    driverControls = new DriverControls(DriverConstants.driverPort);
    drive.setDefaultCommand(
        drive.driveFieldCentricCommand(() -> SwerveConfig.toChassisSpeeds(driverControls, drive)));
    driverControls.increaseLimit().onTrue(drive.increaseLimitCommand());
    driverControls.decreaseLimit().onTrue(drive.decreaseLimitCommand());
    driverControls.robotRelative()
        .whileTrue(drive.driveRobotCentricCommand(() -> SwerveConfig.toChassisSpeeds(driverControls, drive)));
    driverControls.resetGyro().onTrue(drive.resetGyroCommand());
    driverControls.toAmp().whileTrue(PathPlannerUtil.getAutoCommand("Anywhere To Amp"));
    driverControls.toPickup().whileTrue(PathPlannerUtil.getAutoCommand("Anywhere To Pickup"));

    // Tuning Buttons
    driverControls.y().onTrue(drive.toggleSysIDMode());
    driverControls.sysIdDynamicForward().whileTrue(drive.sysIdDynamic(Direction.kForward));
    driverControls.sysIdDynamicReverse().whileTrue(drive.sysIdDynamic(Direction.kReverse));
    driverControls.sysIdQuasistaticForward().whileTrue(drive.sysIdQuasistatic(Direction.kForward));
    driverControls.sysIdQuasistaticReverse().whileTrue(drive.sysIdQuasistatic(Direction.kReverse));

    // ------------------------------- OPERATOR CONTROLS ---------------------------------------------------------
    operatorControls = new OperatorControls(DriverConstants.operatorPort);


    operatorControls.toggleElevatorManual().onTrue(elevator.toggleManualCommand());
    operatorControls.setElevatorHigh().onTrue(elevator.setHighCommand());
    operatorControls.setElevatorMid().onTrue(elevator.setMidCommand());
    operatorControls.setElevatorLow().onTrue(elevator.setLowCommand());
    elevator.moveElevator(operatorControls.elevatorManual());

    operatorControls.toggleShooterManual().onTrue(shooter.toggleManualCommand());
    operatorControls.setShooterHigh().onTrue(shooter.setHighCommand());
    operatorControls.setShooterLow().onTrue(shooter.setLowCommand());
    shooter.spinShooter(operatorControls.shooterManual());
    operatorControls.runShooterOut().whileTrue(shooter.runShooterOutCommand());
    operatorControls.runShooterIn().whileTrue(shooter.runShooterInCommand());

    operatorControls.toggleIntakeManual().onTrue(intake.toggleManualCommand());
    operatorControls.runIntakeIn().whileTrue(intake.runIntakeInCommand());
    operatorControls.runIntakeOut().whileTrue(intake.runIntakeOutCommand());
    operatorControls.setArmHigh().onTrue(intake.setArmHighCommand());
    operatorControls.setArmLow().onTrue(intake.setArmLowCommand());
    intake.moveArm(operatorControls.intakeManual());

  }

  private void configureSubsystems() {
    drive = new Drive(SwerveConfig.getConfiguredDrivetrain());
    elevator = new Elevator();
    intake = new Intake();
    shooter = new Shooter();
  }

}
