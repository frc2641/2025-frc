package frc.team2641.robot2025.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants.CANConstants;
import frc.team2641.robot2025.Constants.ClimberConstants;

public class ClimberNeo extends SubsystemBase implements ClimberIO {
  private SparkMax motor;

  private static ClimberNeo instance;
  public static ClimberNeo getInstance() {
    if (instance == null) instance = new ClimberNeo();
    return instance;
  }

  private ClimberNeo() {
    configMotor();
  }

  public void stop() {
    double val = ClimberConstants.SRL.calculate(0);
    motor.set(val);
  }

  public void extend() {
    double val = ClimberConstants.SRL.calculate(ClimberConstants.climberSpeed);
    motor.set(val);
  }

  public void retract() {
    double val = ClimberConstants.SRL.calculate(ClimberConstants.climberSpeed);
    motor.set(-val);
  }
  
  public double getPosition() {
    return motor.getAbsoluteEncoder().getPosition();
  }

  private void configMotor() {
    motor = new SparkMax(CANConstants.climberNeo, MotorType.kBrushless);

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
    }
  }

  public SparkMax getMotor() {
    return motor;
  }
}