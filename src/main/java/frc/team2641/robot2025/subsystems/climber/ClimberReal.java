package frc.team2641.robot2025.subsystems.climber;

import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants.CANConstants;
import frc.team2641.robot2025.Constants.ClimberConstants;

public class ClimberReal extends SubsystemBase implements ClimberIO {
  private TalonFX leftMotor;
  private TalonFX rightMotor;

  private static ClimberReal instance;
  public static ClimberReal getInstance() {
    if (instance == null) instance = new ClimberReal();
    return instance;
  }

  private ClimberReal() {
    configMotor();
  }

  public void stop() {
    double val = ClimberConstants.SRL.calculate(0);
    rightMotor.set(val);
    leftMotor.set(val);
  }

  public void extend() {
    double val = ClimberConstants.SRL.calculate(ClimberConstants.climberSpeed);
    rightMotor.set(val);
    leftMotor.set(val);
  }

  public void retract() {
    double val = ClimberConstants.SRL.calculate(ClimberConstants.climberSpeed);
    rightMotor.set(-val);
    leftMotor.set(-val);
  }
  
  public double getPosition() {
    return rightMotor.getPosition().getValueAsDouble();
  }

  private void configMotor() {
    leftMotor = new TalonFX(CANConstants.leftClimber);
    rightMotor = new TalonFX(CANConstants.rightClimber);

    TalonFXConfigurator leftConfiger = leftMotor.getConfigurator();
    TalonFXConfigurator rightConfiger = leftMotor.getConfigurator();

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    config.withOpenLoopRamps(new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0.5));

    // TODO numbers are untested
    // config.CurrentLimits.SupplyCurrentLimit = 40;
    // config.CurrentLimits.SupplyCurrentLowerLimit = 45;
    // config.CurrentLimits.SupplyCurrentLowerTime = 0.1;
    // config.CurrentLimits.StatorCurrentLimit = 60;

    leftConfiger.apply(config);
    rightConfiger.apply(config);
  }
  
  @Override
  public void periodic() {
    if ((Math.abs(leftMotor.getVelocity().getValue().baseUnitMagnitude()) < ClimberConstants.stallV) && (leftMotor.getTorqueCurrent().getValue().baseUnitMagnitude() > ClimberConstants.stallI)){
      System.out.println("\n\n *** STALL DETECTED - LEFT CLIMBER *** \n\n");
    }

    if ((Math.abs(rightMotor.getVelocity().getValue().baseUnitMagnitude()) < ClimberConstants.stallV) && (rightMotor.getTorqueCurrent().getValue().baseUnitMagnitude() > ClimberConstants.stallI)){
      System.out.println("\n\n *** STALL DETECTED - RIGHT CLIMBER *** \n\n");
    }
  }

  public TalonFX getMotor() {
    return rightMotor;
  }
}