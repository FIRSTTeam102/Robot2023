import static org.mockito.Mockito.spy;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Grabber;

import edu.wpi.first.hal.HAL;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

public class GrabberTest {
	private Grabber grabber;

	@BeforeEach
	public void setup() {
		assert HAL.initialize(500, 0);
		grabber = spy(RobotContainer.getInstance().grabber);
	}

	@AfterEach
	public void shutdown() throws Exception {
		// grabber.close();
	}

	// @Test
	// public void autoGrabs() {
	// CommandScheduler.getInstance().run();
	// assertFalse(command.isScheduled());

	// var inputs = new GrabberIOInputsAutoLogged();
	// inputs.objectDetected = true;
	// inputs.closed = false;
	// when(updateInputs.invoke(grabber, inputs)).thenReturn(inputs);

	// CommandScheduler.getInstance().run();
	// assertTrue(command.isScheduled());
	// }
}
