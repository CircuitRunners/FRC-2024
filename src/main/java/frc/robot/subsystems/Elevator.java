
// Most of this code is from before 
package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private TalonFX elevatorl = new TalonFX(ElevatorConstants.lMotor);
  private TalonFX elevatorr = new TalonFX(ElevatorConstants.rMotor);
  PIDController elevatorPID = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
  private boolean manual = false;
  private double goal = 0;
  private ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.ks, ElevatorConstants.kg, ElevatorConstants.kv);

  /** Creates a new Elevator. */
  public Elevator() {
  }


  public double getPosition(){
    return elevatorl.getPosition().getValueAsDouble();
  }

  public Command toggleManualCommand(){
    return runOnce(this::toggleManual);
  }
  public void toggleManual(){
    manual = !manual;
  }

  public boolean isManual(){
    return manual;
  }

  public Command setHighCommand(){
    return runOnce(this::setHigh);
  }
  public void setHigh(){
    goal = ElevatorConstants.high;
  }

  public Command setMidCommand(){
    return runOnce(this::setMid);
  }

  public void setMid(){
    goal = ElevatorConstants.mid;
  }

  public Command setLowCommand(){
    return runOnce(this::setLow);
  }

  public void setLow(){
    goal = ElevatorConstants.low;
  }

  public void moveElevator(double speed){
    if(elevatorl.getPosition().getValueAsDouble() < ElevatorConstants.high || elevatorl.getPosition().getValueAsDouble() > ElevatorConstants.low){
      elevatorl.set(0);
    }
    else{
      elevatorl.set(feedforward.calculate(speed));
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(!manual){
      elevatorl.set(elevatorPID.calculate(elevatorl.getPosition().getValueAsDouble(), goal));
      elevatorr.set(elevatorPID.calculate(elevatorr.getPosition().getValueAsDouble(), goal));
    }
  }
}
