package frc.team2641.robot2025;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.team2641.robot2025.helpers.ArmPosition;
import swervelib.math.Matter;

public final class Constants {
  public static final double ROBOT_MASS = 47; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms spark max velocity lag

  public static final class AutoConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(1.25, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(6, 0, 0);
  }

  public static final class DriveConstants {
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final double MAX_SPEED = 10; // m/s
  }

  public static class OperatorConstants {
    public static final double LEFT_X_DEADBAND = 0.1;
    
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class CANConstants {
    // TODO make wrist the last CAN id in case it doesn't exist
    public static final int elevator = 13;
    public static final int leftIntake = 14;
    public static final int rightIntake = 15;
    public static final int winch = 16;
    public static final int wrist = 17;
    public static final int climber = 18;

    public static final int pdh = 20;
  }

  public static final class IntakeConstants {
    public static final double rightIntake = 0.3;
    public static final double leftIntake = 0.2;

    

    public static final double stallV = 0.1;
    public static final double stallI = 30;
  }

  public static final class WristConstants {
    public static final double speed = 0.25;

    public static final PIDConstants wristPID = new PIDConstants(8, 0.01, 0.1);

    public static final SlewRateLimiter wristRateLimiter = new SlewRateLimiter(3);

    public static final double initPos = 0;
    // TODO find wrist max/win pos
    public static final double maxPos = 0;
    public static final double minPos = -8.49;

    public static final double stallV = 0.1;
    public static final double stallI = 30;
  }

  public static final class ClimberConstants {
    public static final double winchSpeed = 0.25;
    public static final double climberSpeed = 0.20;


    public static final double stallV = 0.1;
    public static final double stallI = 30;
  }

  public static final class ElevatorConstants {
    public static final double elevatorSpeed = 0.05;

    public static final PIDConstants PID = new PIDConstants(3.5, 0, 0);
    public static final SlewRateLimiter SRL = new SlewRateLimiter(1); // m/s

    public static final double elevConvert = (2 * 1.0 / 12 * Units.inchesToMeters(2) * Math.PI) / 0.584; // gearbox * sprocket diameter 
    public static final double initPos = 0;

    public static final double stallV = 0.1;
    public static final double stallI = 30;

    // from https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=100&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A30%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22Kraken%20X60%20%28FOC%29%2A%22%7D&ratio=%7B%22magnitude%22%3A12%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A0.5%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A60%2C%22u%22%3A%22in%22%7D
    public static final double kElevatorkG = 0.31; 
    public static final double kElevatorkV = 9.33; 
    public static final double kElevatorkA = 0.04; 

    public static final int kMotorPort = 0;
    public static final int kEncoderAChannel = 0;
    public static final int kEncoderBChannel = 1;
    public static final int kJoystickPort = 0;
  
    public static final double kElevatorKp = 50;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0;
  
    public static final double kElevatorkS = 0.1; // volts (V)
    // public static final double kElevatorkG = 0.762; // volts (V)
    // public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
    // public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))
  
    public static final double kElevatorGearing = 12;
    
    public static final double kElevatorDrumRadius = Units.inchesToMeters(1);// or 1/2??? 
    public static final double kCarriageMass = 15.8757; // kg

    public static final double kSetpointMeters = 0.75;

    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double kMinElevatorHeightMeters = 0.0;
    public static final double kMaxElevatorHeightMeters = 2.35;
  
    // distance per pulse = (distance per revolution) / (pulses per revolution)
    //  = (Pi * D) / ppr
    public static final double kElevatorEncoderDistPerPulse = 2.0 * Math.PI * kElevatorDrumRadius / 4096;
  }

  public static final class ArmPositions {
    public static final ArmPosition L1 = new ArmPosition(0, 0.6898161006834332);
    public static final ArmPosition L2 = new ArmPosition(0, 0.9260855653059362);
    public static final ArmPosition L3 = new ArmPosition(0, 1.45);
    public static final ArmPosition L4 = new ArmPosition(0, 2.2350001091175025);
    public static final ArmPosition humanPlayer = new ArmPosition(0, 0.3290514870627403);
    public static final ArmPosition processor = new ArmPosition(0, -1);
    public static final ArmPosition algaeRemoval = new ArmPosition(0, -1);
  } 

}