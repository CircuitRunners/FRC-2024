package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase { // i mostly updated this to match elevator.java's structure
  private TalonFX arm = new TalonFX(IntakeConstants.armId);
  private TalonFX intake = new TalonFX(IntakeConstants.intakeId);
  private PIDController armPID = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
  private double targetAngle = 0;

  public Command runIntakeInCommand(){
    return run(() -> runIntake(IntakeConstants.intakeInSpeed));
  }

  public Command runIntakeOutCommand(){
    return run(() -> runIntake(IntakeConstants.intakeOutSpeed));
  }

  public Command stopIntakeCommand(){
    return run(() -> runIntake(0));
  }

  public void runIntake(double speed){
    intake.set(speed);
  }


  //

  public Command setArmHighCommand(){
    return run(() -> moveArmToTargetPosition(IntakeConstants.high));
  }

  public Command setArmLowCommand(){
    return run(() -> moveArmToTargetPosition(IntakeConstants.low));
  }

  public Command stopArmCommand(){
    return run(() -> moveArmToTargetPosition(0));
  }

  //

  public void moveArmToTargetPosition(double target){
    this.targetAngle = target;
  }


  //

  public void periodic(){
    arm.set(armPID.calculate(arm.getPosition().getValueAsDouble(), targetAngle));
  }
}
