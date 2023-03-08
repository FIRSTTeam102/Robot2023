package frc.robot.constants;

// @formatter:off
public enum ScoringPosition {
	Ground(0, .413),
	MidCube(.702, .777),
	MidCone(.9652, .82, true),
	HighCube(.973, 1.213),
	HighCone(ElevatorConstants.maxHeight_m, 1.308, true),
	DoubleSubstation(.952, .382),
	AllIn(.19, 0);

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
