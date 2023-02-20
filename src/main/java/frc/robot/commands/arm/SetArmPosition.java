package frc.robot.commands.arm;

import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetArmPosition extends InstantCommand {
	private Arm arm;
	private double armLength_m;

	public SetArmPosition(Arm arm, double armLength_m) {
		this.armLength_m = armLength_m;
		this.arm = arm;
		addRequirements(arm);
	}

	/** sets arm position to 0, esentiallty a reset */
	public SetArmPosition(Arm arm) {
		this.armLength_m = 0;
		this.arm = arm;
		addRequirements(arm);
	}

	@Override
	public void initialize() {
		arm.setPosition(armLength_m);
	}
}