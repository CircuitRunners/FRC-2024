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
  private boolean manual = false;
  private double goal = 0;

  public Command runIntakeInCommand(){
    return run(this::runIntakeIn);
  }

  public Command runIntakeOutCommand(){
    return run(this::runIntakeOut);
  }

  public Command stopIntakeCommand(){
    return run(this::stopIntake);
  }

  //

  public void runIntakeIn(){
    intake.set(IntakeConstants.intakeInSpeed);
  }

  public void runIntakeOut(){
    intake.set(IntakeConstants.intakeOutSpeed);
  }

  public void stopIntake(){
    goal = 0;
  }

  //

  public Command toggleManualCommand(){
    return runOnce(this::toggleManual);
  } // (reminder for myself) :: = pass on the function ...??

  public boolean isManual(){
    return manual;
  }

  //

  public Command setArmHighCommand(){
    return run(this::setArmHigh);
  }

  public Command setArmLowCommand(){
    return run(this::setArmLow);
  }

  public Command stopArmCommand(){
    return run(this::stopArm);
  }

  //

  public void setArmHigh(){
    goal = IntakeConstants.high;
  }

  public void setArmLow(){
    goal = IntakeConstants.low;
  }

  public void stopArm(){
    goal = 0;
  }

  public void toggleManual(){
    manual = !manual;
  }

  //

  public void moveArm(double speed){
    if (arm.getPosition().getValueAsDouble() < IntakeConstants.high || arm.getPosition().getValueAsDouble() > IntakeConstants.low){
      arm.set(speed);
    } else {
      arm.set(0);
    }
  }

  //

  public void periodic(){
    if (!manual){
      arm.set(armPID.calculate(arm.getPosition().getValueAsDouble(), goal));
    }
  }
}
