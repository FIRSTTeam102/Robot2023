package frc.robot.commands.arm;

import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetArmPosition extends InstantCommand {
	private Arm arm;
	private double position;

	/** 
	 * @param position position in percentage of max extension on the range [0, 1] (0 is fully in, 1 is max extension)
	 */
	public SetArmPosition(Arm arm, double position) {
		this.position = position;
		this.arm = arm;
		addRequirements(arm);
	}

	/**
	 * sets arm position to 0, esentiallty a reset
	 */
	public SetArmPosition(Arm arm) {
		this.position = 0;
		this.arm = arm;
		addRequirements(arm);
	}

	@Override
	public void initialize() {
		arm.setPosition(position);
	}
}
