package frc.team2641.robot2025;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {
  public static final double ROBOT_MASS = 47; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms spark max velocity lag
  public static final double SNIPER_MODE = 0.2; // increase... around 25%

  public static final class AutoConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(1.25, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(6, 0, 0);
  }

  public static final class DriveConstants {
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final double MAX_SPEED = 6; // m/s, will be around 6 or 7
  }

  public static class OperatorConstants {
    public static final double LEFT_X_DEADBAND = 0.1;
    
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class CANConstants {
    public static final int elevator = 13;
    public static final int intake = 14;
    public static final int outtake = 15;
    public static final int winch = 16;
    public static final int climber = 17;

    public static final int pdh = 20;
  }

  public static final class IntakeConstants {
    public static final double speedIn = 0.05;
    public static final double speedOut = 0.1;
    
    public static final double stallV = 0.1;
    public static final double stallI = 30;
  }

  public static final class ClimberConstants {
    public static final double winchSpeed = 0.75;
    public static final double climberSpeed = 0.8;

    public static final double stallV = 0.1;
    public static final double stallI = 30;
  }

  public static final class ElevatorConstants {
    public static final double elevatorSpeed = 0.01;

    // public static final PIDConstants PID = new PIDConstants(8, 0.15, 0.2); // better values?
    public static final PIDConstants PID = new PIDConstants(4, 0.0, 0);
    public static final SlewRateLimiter SRL = new SlewRateLimiter(1); // m/s

    // TODO: Find elevator range

    public static final double elevConvert = (2 * 1.0 / 12 * Units.inchesToMeters(2) * Math.PI) / 0.584; // gearbox * sprocket diameter 
    public static final double initPos = 0;

    public static final double stallV = 0.1;
    public static final double stallI = 30;

// from https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=80&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A6.58691%2C%22u%22%3A%22kg%22%7D&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22Kraken%20X60%20%28FOC%29%2A%22%7D&ratio=%7B%22magnitude%22%3A12%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A0.5%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A2.35%2C%22u%22%3A%22m%22%7D
    public static final double kG = 0.5; 
    public static final double kV = 9.33; 
    public static final double kA = 0.03; 

    public static final int kMotorPort = 0;
    public static final int kEncoderAChannel = 0;
    public static final int kEncoderBChannel = 1;
    public static final int kJoystickPort = 0;

  
    public static final double kS = 0; // volts (V)
    // public static final double kElevatorkG = 0.762; // volts (V)
    // public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
    // public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))
  
    public static final double kElevatorGearing = 12;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(1);// or 1/2??? 
    public static final double kCarriageMass = 6.58691; // kg

    public static final double kSetpointMeters = 0.75;

    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double kMinElevatorHeightMeters = 0.0;
    public static final double kMaxElevatorHeightMeters = 2.35;
  
    // distance per pulse = (distance per revolution) / (pulses per revolution)
    //  = (Pi * D) / ppr
    public static final double kElevatorEncoderDistPerPulse = 2.0 * Math.PI * kElevatorDrumRadius / 4096;
  }

  public static final class ElevatorPositions {
    public static final double L1 = 0.6898161006834332;
    public static final double L2 = 0.8502850411837511;
    public static final double L3 = 1.4037806205120347;
    public static final double L4 = 2.1647419080138217; 
    public static final double HP = 0.3372576874968421;

  }
  public static enum ELEVNUM {
    L1,
    L2,
    L3,
    L4,
    HP
  }
}