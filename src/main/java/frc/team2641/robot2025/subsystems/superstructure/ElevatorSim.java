// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2641.robot2025.subsystems.superstructure;


import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants;

public class ElevatorSim extends SubsystemBase implements ElevatorIO {

private static ElevatorSim instance;


  
  /** Creates a new ElevatorSim. 
   * @param m_elevMotorGearBox */
  // public ElevatorSim(DCMotor m_elevMotorGearBox, 
  //   boolean simulateGravity, 
  //   double startingHeight, 
  //   double measurementStandardDeviation) {

  public ElevatorSim()
  {

  }

    

    public static ElevatorSim getInstance(){
      if (instance == null)
        instance = new ElevatorSim();
      return instance;
    } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void setPosition(double pos) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
  }

  @Override
  public double getPosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
  }

  
}
