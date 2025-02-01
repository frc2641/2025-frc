// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2641.robot2025.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.helpers.ArmPos;

public class Arm extends SubsystemBase {

  private Wrist wrist;
  private Elevator elev;
  private ArmPos pos;
  private static Arm instance;


  public static Arm getInstance() {
    if (instance == null)
      instance = new Arm();
    return instance;
  }
  /** Creates a new Arm. */
  private Arm() {
    wrist = Wrist.getInstance();
    elev = Elevator.getInstance();
    pos = new ArmPos(0, 0);
  }

  public void setTargetPosition(ArmPos pos){
    this.pos = pos;
  }

  public void setPosition(){
    wrist.setPos(pos.getWrist());
    elev.setPos(pos.getElev());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
