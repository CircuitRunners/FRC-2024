package frc.robot.io;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveConstants;

public class DriverControls extends CommandXboxController {
  private double inverted;
  public DriverControls(int port) {
    super(port);
    
  }

  public double driveForward() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      inverted = -1;
    } else {
      inverted = 1;
    }
    return MathUtil.applyDeadband(inverted * getLeftY(), DriverConstants.stickDeadband) * SwerveConstants.limit
        * SwerveConstants.maxVelocityMPS;
  }

  public double driveStrafe() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      inverted = -1;
    } else {
      inverted = 1;
    }
    return MathUtil.applyDeadband(inverted * getLeftX(), DriverConstants.stickDeadband) * SwerveConstants.limit
        * SwerveConstants.maxVelocityMPS;
  }

  public double driveRotation() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      inverted = -1;
    } else {
      inverted = 1;
    }
    return MathUtil.applyDeadband(-getRightX(), DriverConstants.stickDeadband) * SwerveConstants.limit
        * SwerveConstants.maxAngularVelocityRPS;
  }

  
  public Trigger resetGyro() {
    return start();
  }



  public Trigger robotRelative() {
    return leftTrigger();
  }
  
  public Trigger increaseLimit() {
    return rightBumper();
  }
  
  public Trigger decreaseLimit() {
    return leftBumper();
  }
  
  public Trigger toAmp(){
    return a();
  }

  public Trigger aimAtSpeaker() {
    return rightTrigger();
  }

  public Trigger sysIdDynamicForward(){
    return povUp();
  }

  public Trigger sysIdDynamicReverse(){
    return povDown();
  }

  public Trigger sysIdQuasistaticForward(){
    return povLeft();
  }

  public Trigger sysIdQuasistaticReverse(){
    return povRight();
  }

  public Trigger toggleSysIDMode(){
    return y();
  }

}

