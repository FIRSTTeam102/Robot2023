package frc.robot.commands.swerve;

import frc.robot.Autos;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class TurnToAngle extends PIDCommand {
	private static Translation2d zeroTranslation = new Translation2d(0, 0);

	TurnToAngle(double angle_rad, Swerve swerve) {
		super(
			Autos.ppRotationController,
			() -> swerve.getYaw().getRadians(),
			() -> angle_rad,
			(double output) -> swerve.drive(zeroTranslation, output),
			swerve);
		m_controller.setTolerance(0.01, 0.01);
	}

	@Override
	public boolean isFinished() {
		return m_controller.atSetpoint();
	}
}
