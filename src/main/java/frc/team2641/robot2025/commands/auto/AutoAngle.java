package frc.team2641.robot2025.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.Constants.AutoAngleNum;
import frc.team2641.robot2025.subsystems.swerve.Drivetrain;

public class AutoAngle extends Command {
	private Drivetrain drivetrain = Drivetrain.getInstance();

	private boolean isAutonomous;

	private BooleanPublisher alignmentPub;
	private DoublePublisher angularVelocityPub;
	private AutoAngleNum goal;

	public AutoAngle(AutoAngleNum x, boolean isAutonomous) {
		
		this.isAutonomous = isAutonomous;
		// if ( x == null ) 
		// 	end(false);
		// else	
			goal = x;
		

			
			NetworkTable table = NetworkTableInstance.getDefault().getTable("state");
			
			alignmentPub = table.getBooleanTopic("autoAlign").publish();
			alignmentPub.set(false);
			

		angularVelocityPub = table.getDoubleTopic("angularVelocity").publish();
		angularVelocityPub.set(0);

		addRequirements(drivetrain);
	}

	public void initialize() {
		alignmentPub.set(true);
		
		// TODO see if necesary
		// if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
			
		// }
	}
	

	public void execute() {
		if (goal.getTarget() < 0) {
			if (drivetrain.getHeading().getDegrees() > goal.getTarget() && drivetrain.getHeading().getDegrees() < goal.getOpposite()) {
				if (drivetrain.getHeading().getDegrees() < goal.getTarget()+20 && drivetrain.getHeading().getDegrees() > 0) {
					if (isAutonomous)
						drivetrain.drive(new Translation2d(0, 0), -0.4, true);
					else
						angularVelocityPub.set(-0.4);
				} else {
					if (isAutonomous)
						drivetrain.drive(new Translation2d(0, 0), -0.65, true);
					else
						angularVelocityPub.set(-0.65);
				}
			} else {
				if (drivetrain.getHeading().getDegrees() > goal.getTarget()-20 && drivetrain.getHeading().getDegrees() < 0) {
					if (isAutonomous)
						drivetrain.drive(new Translation2d(0, 0), 0.4, true);
					else
						angularVelocityPub.set(0.4);
				} else {
					if (isAutonomous)
						drivetrain.drive(new Translation2d(0, 0), 0.65, true);
					else
						angularVelocityPub.set(0.65);
				}
			}
		} else {
			if (drivetrain.getHeading().getDegrees() < goal.getTarget() && drivetrain.getHeading().getDegrees() > goal.getOpposite()) {
				if (drivetrain.getHeading().getDegrees() > goal.getTarget()-20 && drivetrain.getHeading().getDegrees() < 0) {
					if (isAutonomous)
						drivetrain.drive(new Translation2d(0, 0), 0.4, true);
					else
						angularVelocityPub.set(0.4);
				} else {
					if (isAutonomous)
						drivetrain.drive(new Translation2d(0, 0), 0.65, true);
					else
						angularVelocityPub.set(0.65);
				}
			} else {
				if (drivetrain.getHeading().getDegrees() < goal.getTarget()+20 && drivetrain.getHeading().getDegrees() > 0) {
					if (isAutonomous)
						drivetrain.drive(new Translation2d(0, 0), -0.4, true);
					else
						angularVelocityPub.set(-0.4);
				} else {
					if (isAutonomous)
						drivetrain.drive(new Translation2d(0, 0), -0.65, true);
					else
						angularVelocityPub.set(-0.65);
				}
			}
		}
	}

	public void end(boolean interrupted) {
		alignmentPub.set(false);
		if (isAutonomous)
			drivetrain.drive(new Translation2d(0, 0), 0, false);
		else
			angularVelocityPub.set(0);
	}

	public boolean isFinished() {
		return (drivetrain.getHeading().getDegrees()<(goal.getTarget()+1) && drivetrain.getHeading().getDegrees()>(goal.getTarget()-1));
	}
}