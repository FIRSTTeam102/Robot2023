import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

import frc.robot.commands.swerve.TeleopSwerve;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class SwerveTest {
	private static final double measurementDelta = 0.3;

	private Swerve swerve;

	@BeforeEach
	public void setup() {
		assert HAL.initialize(500, 0);
		swerve = RobotContainer.getInstance().swerve;
		var zeroHeading = new Rotation2d(0.0);
		swerve.poseEstimator.resetPosition(zeroHeading, swerve.getPositions(), new Pose2d(0.0, 0.0, zeroHeading));
	}

	@AfterEach
	public void shutdown() throws Exception {
		// swerve.close();
	}

	@Test
	public void driverCanTurn() {
		var driverController = spy(new XboxController(2));
		when(driverController.getRightX()).thenReturn(0.5);

		var teleopSwerve = spy(new TeleopSwerve(swerve, driverController));
		when(teleopSwerve.runsWhenDisabled()).thenReturn(true);
		teleopSwerve.schedule();
		assertTrue(teleopSwerve.isScheduled());

		// just run it a bunch of times so the modules get there
		for (int i = 0; i <= 20; i++) {
			CommandScheduler.getInstance().run();
			// Timer.delay(Constants.loopPeriod_s);
		}

		final var states = swerve.getStates();
		final var angles = new double[] {7 * Math.PI / 4, Math.PI / 4, 7 * Math.PI / 4, Math.PI / 4};
		for (int i = 0; i < states.length; i++)
			assertEquals(angles[i], states[i].angle.getRadians(), measurementDelta);

		teleopSwerve.cancel();
	}
}
