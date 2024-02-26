package frc.robot.io;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveConstants;

public class DriverControls extends CommandXboxController {

  public DriverControls(int port) {
    super(port);
  }

  public double driveForward() {
    return MathUtil.applyDeadband(-getLeftY(), DriverConstants.stickDeadband) * SwerveConstants.limit
        * SwerveConstants.maxSpeedMPS;
  }

  public double driveStrafe() {
    return MathUtil.applyDeadband(-getLeftX(), DriverConstants.stickDeadband) * SwerveConstants.limit
        * SwerveConstants.maxSpeedMPS;
  }

  public double driveRotation() {
    return MathUtil.applyDeadband(-getRightX(), DriverConstants.stickDeadband) * SwerveConstants.limit
        * SwerveConstants.maxAngularVelocityRPS;
  }

  public Trigger toAmp(){
    return povLeft();
  }

  public Trigger resetGyro() {
    return start();
  }

  public Trigger focusOnSpeaker(){
    return leftTrigger();
  }

  public Trigger robotRelative() {
    return rightTrigger();
  }

  public Trigger increaseLimit() {
    return rightBumper();
  }

  public Trigger decreaseLimit() {
    return leftBumper();
  }

  public Trigger toPickup(){
    return povUp();
  }
}

