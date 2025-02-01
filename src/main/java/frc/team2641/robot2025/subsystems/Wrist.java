package frc.team2641.robot2025.subsystems;

// Copyright (c) 2023 FRC Team 2641
// Use of this source code is governed by the MIT license


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants;

public class Wrist extends SubsystemBase {
  private static Wrist instance = null;

  public static Wrist getInstance() {
    if (instance == null)
      instance = new Wrist();
    return instance;
  }

  private TalonFX wrist = new TalonFX(Constants.CAN.wrist);

  public Wrist() {

    wrist.configAllowableClosedloopError(0, Constants.IntakeGains.wristGains.kAllowableError, 30);

    wrist.config_kP(0, Constants.IntakeGains.wristGains.kP, 30);
    wrist.config_kI(0, Constants.IntakeGains.wristGains.kI, 30);
    wrist.config_kD(0, Constants.IntakeGains.wristGains.kD, 30);
    wrist.config_kF(0, Constants.IntakeGains.wristGains.kF, 30);

    wrist.setNeutralMode(NeutralMode.Brake);
  }

  public void set(double value) {
    wrist.set(Constants.IntakeGains.wristRateLimiter.calculate(value * Constants.IntakeGains.wristGains.kPeakOutput));
  }

  public void setPos(double pos) {
    wrist.set(ControlMode.Position, pos);
  }

  public int getEncoder() {
    return (int) wrist.getSelectedSensorPosition();
  }

  public void setEncoder(double value) {
    wrist.setSelectedSensorPosition(value);
  }

  @Override
  public void periodic() {
  }
}