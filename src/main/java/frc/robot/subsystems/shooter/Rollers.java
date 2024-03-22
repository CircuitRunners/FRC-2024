// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants.RollerConstants;

public class Rollers extends SubsystemBase {
  /** Creates a new Rollers. */
  private VictorSP rollers = new VictorSP(RollerConstants.rollerID);
  private TimeOfFlight tof = new TimeOfFlight(RollerConstants.tofId);


  public Rollers() {

  }

  public Command runRollersInCommand(){
    return run(() -> runRollers(RollerConstants.rollerInSpeed));
  }

  public Command autoIntake(){
    return runRollersInCommand().until(this::hasNote).finallyDo(this::stopIntake);
  }
  public void stopIntake(){
    runRollers(0);
  }
  public boolean hasNote(){
    return tof.getRange() < RollerConstants.tofThreshold;
  }

  public void runRollers(double speed){
    rollers.set(speed);
  }

  public Command runRollersOut(){
    return run(() -> runRollers(RollerConstants.rollerOutSpeed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}