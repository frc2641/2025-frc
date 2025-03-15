package frc.team2641.robot2025.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants.CANConstants;
import frc.team2641.robot2025.Constants.ClimberConstants;

public class ClimberReal extends SubsystemBase implements ClimberIO {
  private SparkMax motor;

  private static ClimberReal instance;
  public static ClimberReal getInstance() {
    if (instance == null) instance = new ClimberReal();
    return instance;
  }

  private ClimberReal() {
    configMotor();
        motor.getEncoder().setPosition(0);
  }

  public void stop() {
    motor.stopMotor();
  }


  public void extend() {
    
  }

  public void retract() {
    motor.set(-ClimberConstants.climberSpeed);
  }

  public double getPosition() {
    return motor.getAbsoluteEncoder().getPosition();
  }

  private void configMotor() {
    motor = new SparkMax(CANConstants.climber, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(20)
        .idleMode(IdleMode.kBrake);

    // Persist parameters to retain configuration in the event of a power cycle
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  
  @Override
  public void periodic() {
    if ((Math.abs(motor.getAbsoluteEncoder().getVelocity()) < ClimberConstants.stallV) && (motor.getOutputCurrent() > ClimberConstants.stallI)){
      System.out.println("\n\n *** STALL DETECTED - RIGHT CLIMBER *** \n\n");
      stop();
    }
  }

  public SparkMax getMotor() {
    return motor;
  }
}