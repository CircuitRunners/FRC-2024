package frc.robot.io;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;

public class OperatorControls extends CommandXboxController {

  public OperatorControls(int port) {
    super(port);
  }

  // ---------------- Elevator ----------------
  // public Trigger setElevatorHigh() {
  //   return povUp();
  // }
  
  // public Trigger setElevatorMid() {
  //   return povLeft();
  // }
  // public Trigger setElevatorLow() {
  //   return povDown();
  // }

  // public Trigger toggleElevatorManual() {
  //   return start();
  // }

  // public double elevatorManual() {
  //   return MathUtil.applyDeadband(-getLeftY(), DriverConstants.stickDeadband);
  // }

  // ---------------- Shooter ----------------

  public Trigger setArmShootPosition() {
    return povUp();
  }


  public Trigger setArmIntakePosition() {
    return povDown();
  }

  // public Trigger toggleShooterManual() {
  //   return leftBumper();
  // }

  public double armManual() {
    return MathUtil.applyDeadband(-getRightY(), DriverConstants.stickDeadband);
  }

  public Trigger runFlywheelOut(){
    return a();
  }

  public Trigger runShooterIn(){
    return b();
  }

  public Trigger runRollersOut(){
    return leftBumper();
  }

  public Trigger shoot(){
    return leftTrigger();
  }
  public Trigger autoIntakeFromSource(){
    return rightBumper();
  }

  // ---------------- Intake ----------------


  public Trigger runIntakeOut(){
    return leftBumper();
  }


  public Trigger armManualUp(){
    return povUp();
  }

  public Trigger armManualDown(){
    return povDown();
  }

  public Trigger setArmHigh(){
    return new Trigger(() -> MathUtil.applyDeadband(-getRightY(), DriverConstants.stickDeadband) > 0);
  }
  
  public Trigger setArmLow(){
    return new Trigger(() -> MathUtil.applyDeadband(-getRightY(), DriverConstants.stickDeadband) < 0);
  }

  public Trigger armDynamicForward() {
    return povUp();
  }

  public Trigger armDynamicReverse() {
    return povDown();
  }

  public Trigger armQuasistaticForward() {
    return povLeft();
  }

  public Trigger armQuasistaticReverse() {
    return povRight();
  }
}
