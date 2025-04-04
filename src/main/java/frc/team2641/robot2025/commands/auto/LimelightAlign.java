package frc.team2641.robot2025.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.Limelight;
import frc.team2641.robot2025.Constants.PIPELINE;
import frc.team2641.robot2025.subsystems.swerve.Drivetrain;

public class LimelightAlign extends Command {
  private final Drivetrain drivetrain = Drivetrain.getInstance();

  private final PIDController controllerX;
  private final PIDController controllerAngle;
  private int p;

  public LimelightAlign(PIPELINE p, double xSetpoint) {
    this.p = p.get();

    controllerX = new PIDController(3, 0, 0);
    controllerX.setTolerance(0.01);
    controllerX.setSetpoint(xSetpoint);

    controllerAngle = new PIDController(2.25, 0, 0);
    controllerAngle.setTolerance(3 * Math.PI / 180);
    controllerAngle.setSetpoint(0);

    addRequirements(this.drivetrain);
  }

  @Override
  public void initialize() {
    Limelight.setPipelineIndex("limelight", p);
  }

  @Override
  public void execute() {
    double tx = Limelight.getBotPose3d_TargetSpace("limelight").getX();
    double tangle = Limelight.getBotPose3d_TargetSpace("limelight").getRotation().getY() + Math.PI / 2 ;

    SmartDashboard.putNumber("limelight_tx", tx);
    SmartDashboard.putNumber("limelight_tangle", tangle);

    double translationX = controllerX.calculate(tx);
    double rotation = controllerAngle.calculate(tangle);

    drivetrain.drive(new Translation2d(-translationX, 0), -rotation, false);
    SmartDashboard.putNumber("translationX", -translationX);
    SmartDashboard.putNumber("rotation", -rotation);
  }

  @Override
  public boolean isFinished() {
    return Limelight.getFiducialID("") != -1 && controllerX.atSetpoint() && controllerAngle.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
  }
}
