package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase { // i mostly updated this to match elevator.java's structure
  private TalonFX intake = new TalonFX(IntakeConstants.intakeId);
  private double goal = 0;

  //

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
    goal = IntakeConstants.intakeInSpeed;
  }

  public void runIntakeOut(){
    goal = IntakeConstants.intakeOutSpeed;
  }

  public void stopIntake(){
    goal = 0;
  }

  //

  public void periodic(){
    if (!manual){
      intake.set(goal);
    }
  }
}
