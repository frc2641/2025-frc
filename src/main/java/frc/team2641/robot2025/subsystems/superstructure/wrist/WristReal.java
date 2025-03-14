package frc.team2641.robot2025.subsystems.superstructure.wrist;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants.CANConstants;
import frc.team2641.robot2025.Constants.WristConstants;

public class WristReal extends SubsystemBase implements WristIO {
  private TalonFX motor;

  private double setpoint = WristConstants.initPos;
  private boolean stalled;
  private static WristReal instance;
  public static WristReal getInstance() {
    if (instance == null) instance = new WristReal();
    return instance;
  }

  private WristReal() {
    configMotor();
  }

  public void stop() {
    motor.stopMotor();
  }

  public void set(double speed) {
    motor.set(speed);
  }
  
  public void goTo(double pos) {
    setpoint = pos;
  }

  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  public double getSetpoint() {
    return setpoint;
  }

  private void configMotor() {
    motor = new TalonFX(CANConstants.wrist);

    TalonFXConfigurator configer = motor.getConfigurator();

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        // TODO numbers are untested
    // config.CurrentLimits.SupplyCurrentLimit = 40;
    // config.CurrentLimits.SupplyCurrentLowerLimit = 45;
    // config.CurrentLimits.SupplyCurrentLowerTime = 0.1;
    // config.CurrentLimits.StatorCurrentLimit = 60;

    configer.apply(config);

    motor.setPosition(0);

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = WristConstants.wristPID.kP;
    slot0Configs.kI = WristConstants.wristPID.kI;
    slot0Configs.kD = WristConstants.wristPID.kD;
    configer.apply(slot0Configs);
  }
  
  @Override
  public void periodic() {
    if (setpoint < WristConstants.minPos) setpoint = WristConstants.minPos;
    if (setpoint > WristConstants.maxPos) setpoint = WristConstants.maxPos;

    // motor.setControl(posRequest.withPosition(setpoint));
    // System.out.println("Wrist setpoint: " + setpoint);

    if ((Math.abs(motor.getVelocity().getValue().baseUnitMagnitude()) < WristConstants.stallV) && (motor.getTorqueCurrent().getValue().baseUnitMagnitude() > WristConstants.stallI)){
      // stop();
      stalled = true;
    }
    else

  stalled = false;    
    // TODO: Move setpoint retargeting to an else statement above

    SmartDashboard.putNumber("Arm Wrist Real Pos ", motor.getPosition().getValueAsDouble()); 
    SmartDashboard.putBoolean("Wrist Stall", stalled);
   
  }

  public TalonFX getMotor() {
    return motor;
  }
}