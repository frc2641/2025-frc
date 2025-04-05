package frc.team2641.robot2025.subsystems;

import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants.CANConstants;
import frc.team2641.robot2025.Constants.ClimberConstants;

public class Climber extends SubsystemBase  {
  private TalonFX motor;

  private static Climber instance;
  public static Climber getInstance() {
    if (instance == null) instance = new Climber();
    return instance;
  }

  private Climber() {
    configMotor();
  }

  public void stop() {
    motor.stopMotor();
  }

  public void extend() {
    // if(motor.getPosition().getValueAsDouble() > ClimberConstants.highSoftStop)
    motor.set(ClimberConstants.climberSpeed);
    // else  
    // stop();
  }
  
  public void retract() {
    // if (!(motor.getPosition().getValueAsDouble() > 0))
    motor.set(-ClimberConstants.climberSpeed);
    // else  
      // stop();
  }

  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  private void configMotor() {
    motor = new TalonFX(CANConstants.climber);

    TalonFXConfigurator configurator = motor.getConfigurator();
    
    TalonFXConfiguration defaultConfig = new TalonFXConfiguration();
    configurator.apply(defaultConfig);
    
    OpenLoopRampsConfigs config = new OpenLoopRampsConfigs();
    config.withDutyCycleOpenLoopRampPeriod(0.25);
    configurator.apply(config);

    motor.setNeutralMode(NeutralModeValue.Coast);
    motor.setPosition(0);
  }

  @Override
  public void periodic() {
    if ((Math.abs(motor.getVelocity().getValueAsDouble()) < ClimberConstants.stallV) && (motor.getStatorCurrent().getValueAsDouble() > ClimberConstants.stallI)){
      System.out.println("\n\n *** STALL DETECTED - CLIMBER *** \n\n");
      // stop();
    }

    SmartDashboard.putNumber("Climber Position", getPosition());
  }

  public TalonFX getMotor() {
    return motor;
  }
}