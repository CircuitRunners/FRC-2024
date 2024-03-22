// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Arm;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Rollers;

public class Shooter {
  public Arm arm;
  public Flywheel flywheel;
  public Rollers rollers;
  /** Creates a new Shooter. */
  public Shooter() {
    arm = new Arm();
    flywheel = new Flywheel();
    rollers = new Rollers();
  }

  public Command shootCommand(){
    return Commands.sequence(
      rollers.runRollersOutCommand().withTimeout(2),
      Commands.parallel(
        arm.setArmShootPositionAndWait().withTimeout(2), 
        flywheel.runFlywheelOut()
        ),
      rollers.runRollersInCommand()
    );
  }
}
