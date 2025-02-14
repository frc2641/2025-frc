package frc.team2641.robot2025.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IntakeIO extends Subsystem {
	public void intake();
	public void shoot();
	public void stop();
	public void spin();
}