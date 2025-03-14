package frc.team2641.robot2025.helpers;

import frc.team2641.robot2025.Constants;

public class ElevatorContrain {
	private double setpoint;

	public ElevatorContrain(double setpoint) {
		this.setpoint = setpoint;
		constrain();
	}

	public double get() {
		return setpoint;
	}

	public String toString() {
		return "Setpoint: " + setpoint;
	}

	private void constrain() {
		if (setpoint > Constants.ElevatorConstants.kMaxElevatorHeightMeters) setpoint = Constants.ElevatorConstants.kMaxElevatorHeightMeters;
		if (setpoint < Constants.ElevatorConstants.kMinElevatorHeightMeters) setpoint = Constants.ElevatorConstants.kMinElevatorHeightMeters;
	}
}