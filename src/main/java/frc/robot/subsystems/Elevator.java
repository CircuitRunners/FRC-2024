
// Most of this code is from before 
package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private TalonFX elevator = new TalonFX(ElevatorConstants.lMotor);
  private boolean manual = false;
 
  //Replace this with whatever controller we're using
  ElevatorConstants.kD, new TrapezoidProfile.Constraints(ElevatorConstants.maxVel, ElevatorConstants.maxAcc));
  private double goal = 0;
  private ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.ks, ElevatorConstants.kg, ElevatorConstants.kv);

  /** Creates a new Elevator. */
  public Elevator() {
    // elevatorR.setIdleMode(IdleMode.kBrake);

  }


  public double getPosition(){
    return elevator.getSelectedSensorPosition();
  }

  public CommandBase toggleManualCommand(){
    return runOnce(this::toggleManual);
  }
  public void toggleManual(){
    manual = !manual;
  }

  public boolean isManual(){
    return manual;
  }

  public CommandBase setHighCommand(){
    return runOnce(this::setHigh);
  }
  public void setHigh(){
    goal = ElevatorConstants.high;
  }

  public CommandBase setMidCommand(){
    return runOnce(this::setMid);
  }

  public void setMid(){
    goal = ElevatorConstants.mid;
  }

  public CommandBase setLowCommand(){
    return runOnce(this::setLow);
  }

  public void setLow(){
    goal = ElevatorConstants.low;
  }

  public void moveElevator(double speed){
    speed = MathUtil.applyDeadband(speed, Constants.stickDeadband);
    if(elevator.getSelectedSensorPosition() < ElevatorConstants.high || elevator.getSelectedSensorPosition() > ElevatorConstants.low){
      elevator.set(ControlMode.Velocity, 0);
    }
    else{
      elevator.set(ControlMode.Velocity, feedforward.calculate(speed));
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(!manual){
      elevator.set(ControlMode.Position, elevatorPID.calculate(elevator.getSelectedSensorPosition(), goal));
    }
  }
}
