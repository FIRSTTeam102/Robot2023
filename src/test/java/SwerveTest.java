import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.*;

import frc.robot.RobotContainer;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.XboxController;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class SwerveTest {
	private Swerve swerve;
	private XboxController driverController = mock(XboxController.class);
	private TeleopSwerve teleopSwerve;

	@BeforeEach
	public void setup() {
		assert HAL.initialize(500, 0);
		swerve = RobotContainer.getInstance().swerve;
		teleopSwerve = new TeleopSwerve(swerve, driverController);
		// DriverStationSim.setDsAttached(true);
		// DriverStationSim.setEnabled(true);
		// assertTrue(DriverStationSim.getEnabled());
	}

	@AfterEach
	public void shutdown() {
		teleopSwerve.cancel();
		// swerve.close();
	}

	@Test
	public void driverCanTurn() {
		doAnswer(invocation -> 0.5).when(driverController).getRightX();
		// doAnswer(invocation -> 0.5).when(driverController).getLeftY();
		teleopSwerve.schedule();
		// assertTrue(teleopSwerve.isScheduled());

		for (int i = 0; i <= 10; i++) {
			// idk why the command scheduler wont schedule
			// CommandScheduler.getInstance().run();
			teleopSwerve.execute();
			swerve.periodic();
			// Timer.delay(0.2);
		}

		System.out.println(swerve.getPose());
		assertEquals(-Math.PI / 4, swerve.getStates()[0].angle.getRadians(), 0.1);
	}
}
