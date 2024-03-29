package frc.robot.constants;

import frc.robot.subsystems.Arm;

// @formatter:off
public enum ScoringPosition {
	// @fieldcal
	Ground(0, .525),
	GroundFar(0, .82),
	MidCube(.702, .777),
	MidCone(.99, .951, true),
	HighCube(1.04, 1.213),
	HighCone(ElevatorConstants.maxHeight_m, 1.39, true),
	DoubleSubstation(1.075, .37),
	SingleSubstationCube(.58, 0),
	AllIn(.23, Arm.nutDistToArmDist(ArmConstants.minNutDist_m)),
	AllInHigher(.25, Arm.nutDistToArmDist(ArmConstants.minNutDist_m)),
	AllInBumper(.18, Arm.nutDistToArmDist(ArmConstants.minNutDist_m));

// @formatter:on
	public final double elevatorHeight_m;
	public final double armExtension_m;
	public final boolean isCone;

	ScoringPosition(double elevatorHeight_m, double armExtension_m) {
		this(elevatorHeight_m, armExtension_m, false);
	}

	ScoringPosition(double elevatorHeight_m, double armExtension_m, boolean isCone) {
		this.elevatorHeight_m = elevatorHeight_m;
		this.armExtension_m = armExtension_m;
		this.isCone = isCone;
	}
}
