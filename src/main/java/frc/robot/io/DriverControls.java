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
    System.out
        .println("Forward" + MathUtil.applyDeadband(-getLeftY(), DriverConstants.stickDeadband) * SwerveConstants.limit
            * SwerveConstants.maxSpeedMPS);
    return MathUtil.applyDeadband(-getLeftY(), DriverConstants.stickDeadband) * SwerveConstants.limit
        * SwerveConstants.maxSpeedMPS;
  }

  public double driveStrafe() {
    System.out.println("Strafe" +MathUtil.applyDeadband(-getLeftX(), DriverConstants.stickDeadband) * SwerveConstants.limit
    * SwerveConstants.maxSpeedMPS);
    return MathUtil.applyDeadband(-getLeftX(), DriverConstants.stickDeadband) * SwerveConstants.limit
        * SwerveConstants.maxSpeedMPS;
  }

  public double driveRotation() {
    return MathUtil.applyDeadband(-getRightX(), DriverConstants.stickDeadband) * SwerveConstants.limit
        * SwerveConstants.maxAngularVelocityRPS;
  }

  public Trigger robotRelative() {
    return x();
  }

  public Trigger increaseLimit() {
    return rightBumper();
  }

  public Trigger decreaseLimit() {
    return leftBumper();
  }
}
