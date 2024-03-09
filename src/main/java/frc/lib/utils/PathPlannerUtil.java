package frc.lib.utils;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;
import java.util.stream.Stream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class PathPlannerUtil {
  private static final DoubleArraySubscriber kTargetPoseSub = NetworkTableInstance.getDefault()
      .getDoubleArrayTopic("/PathPlanner/targetPose")
      .subscribe(new double[] { 0.0, 0.0, 0.0 });

  public static void configure(Drive drive, Intake intake, Shooter shooter, Elevator elevator) {
    HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(SwerveConstants.translationalPID,
        SwerveConstants.rotationalPID,
        SwerveConstants.maxModuleVelocityMPS, SwerveConstants.driveBaseRadiusMeter, new ReplanningConfig(true, true));

    AutoBuilder.configureHolonomic(
        drive::getPose,
        drive::resetPose,
        drive::getChassisSpeeds,
        drive::driveRobotCentric,
        config,
        () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
        drive);

    NamedCommands.registerCommand("brake", drive.brakeCommand());
    NamedCommands.registerCommand("pickUpNote", intake.runIntakeInCommand());
    NamedCommands.registerCommand("intakeOut", intake.runIntakeOutCommand());
    NamedCommands.registerCommand("stopIntake", intake.stopIntakeCommand());
    NamedCommands.registerCommand("armLow", intake.setArmLowCommand());
    NamedCommands.registerCommand("armHigh", intake.setArmHighCommand());
    NamedCommands.registerCommand("shoot", shooter.runShooterOutCommand());
    NamedCommands.registerCommand("shooterIn", shooter.runShooterInCommand());
    NamedCommands.registerCommand("shooterLow", shooter.setArmIn());
    NamedCommands.registerCommand("shooterHigh", shooter.setArmOut());
    NamedCommands.registerCommand("elevatorHigh", Commands.print("elevatorHigh"));
    NamedCommands.registerCommand("elevatorMid", Commands.print("elevatorMed"));
    NamedCommands.registerCommand("elevatorLow", Commands.print("elevatorLow"));

  }

  /** Returns the command for the auto if it exists. Otherwise throws an error*/
  public static Command getAutoCommand(String name) {
    try {
      return AutoBuilder.buildAuto(name);
    } catch (Exception e) {
      DriverStation.reportError("An error occurred while loading path planner auto", e.getStackTrace());
      return Commands.none();
    }
  }

  public static List<String> getAutos() {
    var path = Path.of(Filesystem.getDeployDirectory().getAbsolutePath(), "pathplanner");
    try (Stream<Path> stream = Files.walk(path)) {
      return stream
          .filter(x -> getFileExtension(x).equals(".auto"))
          .map(x -> getFileStem(x))
          .toList();
    } catch (IOException e) {
      return Collections.emptyList();
    }
  }

  private static String getFileStem(Path path) {
    try {
      String name = path.getFileName().toString();
      return name.substring(0, name.lastIndexOf("."));
    } catch (Exception e) {
      return "";
    }
  }

  private static String getFileExtension(Path path) {
    try {
      String name = path.getFileName().toString();
      return name.substring(name.lastIndexOf("."));
    } catch (Exception e) {
      return "";
    }
  }

  public static Pose2d getCurrentTargetPose() {
    var arr = kTargetPoseSub.get();
    double x = arr[0];
    double y = arr[1];
    Rotation2d rot = Rotation2d.fromRadians(arr[2]);

    return new Pose2d(x, y, rot);
  }

}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
