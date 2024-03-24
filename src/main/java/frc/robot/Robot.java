// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.swerve.SwerveConfig;
import frc.lib.utils.PathPlannerUtil;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.AimAtSpeaker;
import frc.robot.generated.TunerConstants;
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
  private final SendableChooser<Supplier<Command>> autoChooser = new SendableChooser<>();
  // private Vision vision;


  @Override
  public void robotInit() {
    // DataLogManager.start("logs");
    configureSubsystems();
    shooter.arm.resetTargetAngleToEncoderAngle();
    // vision = new Vision(drive::addVisionMeasurement);
  }
  
  @Override
  public void driverStationConnected() {
    configureAutos();
    configureBindings();
    // addPeriodic(vision::run, 0.01);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    shooter.arm.resetTargetAngleToEncoderAngle();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoChooser.getSelected().get();
    shooter.arm.resetTargetAngleToEncoderAngle();
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
    shooter.arm.resetTargetAngleToEncoderAngle();
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
    PathPlannerUtil.configure(drive, intake, shooter, elevator);
    autoChooser.addOption("Do Nothing", () -> Commands.print("Doing Nothing"));
    autoChooser.setDefaultOption("Nick's Taxi Service", () ->   (drive.driveRobotCentricCommand(() -> new ChassisSpeeds(0.5, 0, 0)).withTimeout(4)));
    autoChooser.addOption("Shoot + Nick's Taxi Service", () ->   shooter.shootCommand().andThen(drive.driveRobotCentricCommand(() -> new ChassisSpeeds(1.2, 0, 0)).withTimeout(2.5)));
    PathPlannerUtil.getAutos().forEach(path -> {
      autoChooser.addOption(path, () -> PathPlannerUtil.getAutoCommand(path));
    });
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    
    // ------------------------------- DRIVER CONTROLS ---------------------------------------------------------
    driverControls = new DriverControls(DriverConstants.driverPort);
    drive.setDefaultCommand(drive.driveFieldCentricCommand(() -> SwerveConfig.toChassisSpeeds(driverControls, drive)));
      driverControls.b()
      .onTrue(Commands.runOnce(() -> Drive.limit = 1.0))
      .onFalse(Commands.runOnce(() -> Drive.limit = 0.6 ));
      driverControls.resetGyro().onTrue(drive.resetGyroCommand());
//      driverControls.toAmp().whileTrue(AutoBuilder.pathfindToPose((DriverStation.getAlliance().get() == Alliance.Blue ? FieldConstants.kBlueAmpPose2d : FieldConstants.kRedAmpPose2d), SwerveConstants.pathConstraints));
      // driverControls.toSource().whileTrue(AutoBuilder.pathfindToPose((DriverStation.getAlliance().get() == Alliance.Blue ? FieldConstants.kBlueAmpPose2d : FieldConstants.kRedAmpPose2d), SwerveConstants.pathConstraints));
      driverControls.aimAtSpeaker().whileTrue(new AimAtSpeaker(drive, driverControls));
      
      // ------------------------------ TUNING CONTROLS ---------------------------
      // driverControls.y().onTrue(drive.toggleSysIDMode());
      // driverControls.sysIdDynamicForward().whileTrue(drive.sysIdDynamic(Direction.kForward));
      // driverControls.sysIdDynamicReverse().whileTrue(drive.sysIdDynamic(Direction.kReverse));
      // driverControls.sysIdQuasistaticForward().whileTrue(drive.sysIdQuasistatic(Direction.kForward));
      // driverControls.sysIdQuasistaticReverse().whileTrue(drive.sysIdQuasistatic(Direction.kReverse));
      // ------------------------------- OPERATOR CONTROLS ---------------------------------------------------------
    operatorControls = new OperatorControls(0);

    // operatorControls.toggleElevatorManual().onTrue(elevator.toggleManualCommand());
    // operatorControls.setElevatorHigh().onTrue(elevator.setHighCommand());
    // operatorControls.setElevatorMid().onTrue(elevator.setMidCommand());
    // operatorControls.setElevatorLow().onTrue(elevator.setLowCommand());
    // elevator.moveElevator(operatorControls.elevatorManual());

    operatorControls.start().onTrue(Commands.runOnce(() -> shooter.arm.resetTargetAngleToEncoderAngle()));
    operatorControls.setArmShootPosition().onTrue(shooter.arm.setArmShootPosition());
    operatorControls.setArmIntakePosition().onTrue(shooter.arm.setArmIntakePosition());
    shooter.arm.runManual(operatorControls.armManual());
    operatorControls.runFlywheelOut().whileTrue(shooter.flywheel.setShootSpeedCommand()).onFalse(shooter.flywheel.stopFlywheelCommand());
    // operatorControls.runShooterIn().whileTrue(shooter.rollers.setRollersSpeedInCommand()).onFalse(shooter.rollers.runRollersOutCommandSlow().withTimeout(0.2).finallyDo(() -> shooter.rollers.stopRollersCommand()));
    operatorControls.autoIntakeFromSource().whileTrue(shooter.rollers.autoIntake()).onFalse(shooter.rollers.runRollersOutCommandSlow().withTimeout(0.2).finallyDo(() -> shooter.rollers.stopRollers()));
    operatorControls.runRollersOut().whileTrue(shooter.rollers.runRollersOutCommand()).onFalse(shooter.rollers.stopRollersCommand());
    operatorControls.shoot().onTrue(shooter.shootCommand());
    operatorControls.rightTrigger().onTrue(shooter.shootCommand());
    operatorControls.x().onTrue(shooter.arm.setArmAmpPosition());

    // SysID for shooter arm
    // operatorControls.armDynamicForward().whileTrue(shooter.arm.sysIdDynamicCommand(Direction.kForward));
    // operatorControls.armDynamicReverse().whileTrue(shooter.arm.sysIdDynamicCommand(Direction.kReverse));
    // operatorControls.armQuasistaticForward().whileTrue(shooter.arm.sysIdQuasistaticCommand(Direction.kForward));
    // operatorControls.armQuasistaticReverse().whileTrue(shooter.arm.sysIdQuasistaticCommand(Direction.kReverse));   

    // operatorControls.runIntakeOut().whileTrue(intake.runIntakeOutCommand());
    // operatorControls.setArmHigh().onTrue(intake.setArmHighCommand());
    // operatorControls.setArmLow().onTrue(intake.setArmLowCommand());
    // operatorControls.armManualDown().whileTrue(intake.moveArmManualDown());
    // operatorControls.armManualUp().whileTrue(intake.moveArmManualUp());


  }

  private void configureSubsystems() {
    drive = new Drive(TunerConstants.DriveTrain);
    // elevator = new Elevator();
    // intake = new Intake();
    shooter = new Shooter();
  }

}
