package frc.robot.io;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Drive;

public class DriverControls extends CommandXboxController {
  public DriverControls(int port) {
    super(port);
    
  }

  public double driveForward() {
    return MathUtil.applyDeadband(-getLeftY(), DriverConstants.stickDeadband) * Drive.limit
        * SwerveConstants.maxVelocityMPS;
  }

  public double driveStrafe() {
    return MathUtil.applyDeadband(-getLeftX(), DriverConstants.stickDeadband) * Drive.limit
        * SwerveConstants.maxVelocityMPS;
  }

  public double driveRotation() {
    return MathUtil.applyDeadband(-getRightX(), DriverConstants.stickDeadband) * Drive.limit
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

