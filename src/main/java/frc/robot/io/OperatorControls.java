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

  public Trigger setShooterHigh() {
    return new Trigger(() -> MathUtil.applyDeadband(-getLeftX(), DriverConstants.stickDeadband) > 0);
  }


  public Trigger setShooterLow() {
    return new Trigger(() -> MathUtil.applyDeadband(-getLeftX(), DriverConstants.stickDeadband) < 0);
  }

  public Trigger toggleShooterManual() {
    return leftBumper();
  }

  public double shooterManual() {
    return MathUtil.applyDeadband(-getRightY(), DriverConstants.stickDeadband);
  }

  public Trigger runShooterOut(){
    return a();
  }

  public Trigger runShooterIn(){
    return b();
  }

  // ---------------- Intake ----------------

  public Trigger autoIntake(){
    return rightBumper();
  }

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
    return y();
  }

  public Trigger armDynamicReverse() {
    return a();
  }

  public Trigger armQuasistaticForward() {
    return x();
  }

  public Trigger armQuasistaticReverse() {
    return b();
  }
}
