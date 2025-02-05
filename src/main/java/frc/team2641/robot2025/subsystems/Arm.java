// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2641.robot2025.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.helpers.ArmPos;

public class Arm extends SubsystemBase {

  private Wrist wrist;
  private Elevator elev;
  private ArmPos pos;
  public boolean isAuto;
  public static enum switcher {
    L1,
    L2,
    L3,
    L4,
    HUMAN_PLAYER,
    PROCESSOR,
    ALGAE_REMOVAL
  };
  private static Arm instance;


  public static Arm getInstance() {
    if (instance == null)
      instance = new Arm();
    return instance;
  }
  /** Creates a new Arm. */
  private Arm() {
    isAuto = false;
    wrist = Wrist.getInstance();
    elev = Elevator.getInstance();
    pos = new ArmPos(0, 0);

    
  }

  public void setTargetPosition(ArmPos pos){
    this.pos = pos;
  }

  public void move(){
    
    wrist.setPos(pos.getWrist());
    elev.setPos(pos.getElev());
  }

  public void changeTarget(ArmPos pos){
    this.pos = pos;
  }

  public ArmPos getPosition(){
    return pos;
  }

  public boolean atPos(){
    return (Math.abs(pos.getWrist()-wrist.getPos())<0.5)&&(Math.abs(pos.getElev()-elev.getPos())<0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  


}
