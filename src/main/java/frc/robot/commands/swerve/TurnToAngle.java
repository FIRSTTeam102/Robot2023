package frc.robot.commands.swerve;

import frc.robot.subsystems.Swerve;

import frc.robot.commands.Autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class TurnToAngle extends PIDCommand {
	private static Translation2d zeroTranslation = new Translation2d(0, 0);

	/** turns to a field angle, 0 is away from driver station */
	public TurnToAngle(double angle_rad, Swerve swerve) {
		super(
			Autos.ppRotationController,
			() -> MathUtil.angleModulus(swerve.getYaw().getRadians()),
			() -> MathUtil.angleModulus(angle_rad),
			(double output) -> swerve.drive(zeroTranslation, output, false),
			swerve);
		/** controller already configured in {@link frc.robot.Autos} */
	}

	@Override
	public boolean isFinished() {
		return m_controller.atSetpoint();
	}
}
