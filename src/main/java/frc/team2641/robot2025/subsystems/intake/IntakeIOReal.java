package frc.team2641.robot2025.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.Constants.CANConstants;
import frc.team2641.robot2025.Constants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {
  private TalonFX intakeMotor;
  private TalonFX outtakeMotor;
  
  private static IntakeIOReal instance;
  
  public static IntakeIO getInstance() {
    if (instance == null) instance = new IntakeIOReal();
    return instance;
  }

  private IntakeIOReal() {
    configMotor();

  }

  @Override
  public void stop() {
    intakeMotor.stopMotor();
    outtakeMotor.stopMotor();
  }

  private void configMotor() {
    intakeMotor = new TalonFX(CANConstants.intake);
    outtakeMotor = new TalonFX(CANConstants.outtake);

    
    TalonFXConfigurator configer1 = intakeMotor.getConfigurator();
    TalonFXConfigurator configer2 = outtakeMotor.getConfigurator();

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        // TODO numbers are untested
    // config.CurrentLimits.SupplyCurrentLimit = 40;
    // config.CurrentLimits.SupplyCurrentLowerLimit = 45;
    // config.CurrentLimits.SupplyCurrentLowerTime = 0.1;
    // config.CurrentLimits.StatorCurrentLimit = 60;

    configer1.apply(config);
    configer2.apply(config);

  }

  @Override
  public void periodic() {
    if (Math.abs(intakeMotor.getVelocity().getValue().baseUnitMagnitude()) < IntakeConstants.stallV) {
      System.out.println("\n\n *** STALL DETECTED - INTAKE *** \n\n");
    }
  }

  public TalonFX getMotor() {
    return intakeMotor;
  }

  @Override
  public void firstSpin() {
  intakeMotor.set(Constants.IntakeConstants.speedIn);
  }

  @Override
  public void secondSpin() {
  outtakeMotor.set(Constants.IntakeConstants.speedOut);
  }
}