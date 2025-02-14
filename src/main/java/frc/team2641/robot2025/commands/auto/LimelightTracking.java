package frc.team2641.robot2025.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.Limelight;
import frc.team2641.robot2025.subsystems.swerve.Drivetrain;

public class LimelightTracking extends Command {
  private final Drivetrain drivetrain;

  private final PIDController controllerX;
  private final PIDController controllerAngle;

  public LimelightTracking() {
    this.drivetrain = Drivetrain.getInstance();

    controllerX = new PIDController(3, 1, 0);
    controllerX.setTolerance(0.025);
    controllerX.setSetpoint(0.0);

    controllerAngle = new PIDController(2.9, 0, 0);
    controllerAngle.setTolerance(0.01);
    controllerAngle.setSetpoint(0.0);

    addRequirements(this.drivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double tx = Limelight.getBotPose3d_TargetSpace("").getX();
    double tangle = Limelight.getBotPose3d_TargetSpace("").getRotation().getY();

    double translationX = MathUtil.clamp(controllerX.calculate(tx, 0.0), -1, 1);
    double rotation = MathUtil.clamp(controllerAngle.calculate(tangle, 0.0), -1, 1);

    drivetrain.drive(new Translation2d(0, translationX), -rotation, false);
    SmartDashboard.putNumber("translationX", translationX);
    SmartDashboard.putNumber("rotation", rotation);
  }

  @Override
  public boolean isFinished() {
    return Limelight.getFiducialID("") != -1 && controllerX.atSetpoint() && controllerAngle.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.lock();
  }
}
