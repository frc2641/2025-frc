package frc.team2641.robot2025.helpers;

import frc.team2641.robot2025.Constants;

public class ElevatorConstrain {
	private double setpoint;

	public ElevatorConstrain(double setpoint) {
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

	public static double constrain(double x){
		double z = x;
		if (z > Constants.ElevatorConstants.kMaxElevatorHeightMeters) z = Constants.ElevatorConstants.kMaxElevatorHeightMeters;
		if (z < Constants.ElevatorConstants.kMinElevatorHeightMeters) z = Constants.ElevatorConstants.kMinElevatorHeightMeters;
		return z;
	}
}