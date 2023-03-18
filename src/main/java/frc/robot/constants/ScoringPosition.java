package frc.robot.constants;

// @formatter:off
public enum ScoringPosition {
	// @fieldcal
	Ground(0, .48),
	GroundFar(0, .82),
	MidCube(.702, .777),
	MidCone(.9652, .8, true),
	HighCube(1, 1.213),
	HighCone(ElevatorConstants.maxHeight_m, 1.39, true),
	DoubleSubstation(1.05, .382),
	SingleSubstationCube(.58, 0),
	AllIn(.19, .25);

// @formatter:on
	public final double elevatorHeight_m;
	public final double armExtension_m;
	public final boolean moveDown;

	ScoringPosition(double elevatorHeight_m, double armExtension_m) {
		this(elevatorHeight_m, armExtension_m, false);
	}

	ScoringPosition(double elevatorHeight_m, double armExtension_m, boolean moveDown) {
		this.elevatorHeight_m = elevatorHeight_m;
		this.armExtension_m = armExtension_m;
		this.moveDown = moveDown;
	}
}
