package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Intake extends SubsystemBase {
  TalonFX intake = new TalonFX(ArmConstants.intakeId);

  public Command runIntakeInCommand(){
    return run(this::runIntakeIn);
  }
  public void runIntakeIn(){
    intake.set(1.5);
  }
  public Command runIntakeOutCommand(){
    return run(this::runIntakeOut);
  }
  public void runIntakeOut(){
    intake.set(-0.5);
  }
  public void periodic(){}
}
