package frc.robot;

import frc.robot.constants.Constants;
import frc.robot.constants.Constants.CameraConstants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.io.GyroIO;
import frc.robot.io.GyroIOPigeon2;
import frc.robot.io.GyroIOSim;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.swerve.ChargeStationBalance;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.swerve.XStance;
import frc.robot.commands.vision.AprilTagVision;
import frc.robot.commands.vision.ObjectDetectionVision;
import frc.robot.commands.vision.RetroreflectiveVision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	private static RobotContainer instance = null;

	public static RobotContainer getInstance() {
		if (instance == null)
			instance = new RobotContainer();
		return instance;
	}

	public final CommandXboxController driverController = new CommandXboxController(
		OperatorConstants.driverControllerPort);
	public final CommandXboxController operatorController = new CommandXboxController(
		OperatorConstants.operatorControllerPort);

	public final GyroIO gyro = Robot.isReal()
		? new GyroIOPigeon2(Constants.pigeonId)
		: new GyroIOSim();

	/**
	 * Subsystems
	 */
	public final Swerve swerve = new Swerve(gyro);
	public final Vision vision = new Vision();

	private final TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, driverController.getHID());

	private LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto mode");

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		DriverStation.silenceJoystickConnectionWarning(true);

		// will be automatically scheduled when no other scheduled commands require swerve
		swerve.setDefaultCommand(teleopSwerve);

		configureBindings();

		// setup autos
		autoChooser.addDefaultOption("nothing", new InstantCommand());
		autoChooser.addOption("PP test", Autos.simpleWall(swerve));

		// for testing
		autoChooser.addOption("drive characterization",
			new FeedForwardCharacterization(swerve, true, swerve::runCharacterization, swerve::getCharacterizationVelocity));

		SmartDashboard.putData("do balancing", new ChargeStationBalance(swerve));

		// camera
		try {
			var camera = CameraServer.startAutomaticCapture("arm", 0);
			// camera.setConnectVerbose(0);
			camera.setFPS(CameraConstants.fps);
			camera.setResolution(CameraConstants.width, CameraConstants.height);

			// camera server is evil

			// var cameraServer = CameraServer.addSwitchedCamera("camera");
			// var cameraServer = CameraServer.startAutomaticCapture(camera);
			var cameraServer = (MjpegServer) CameraServer.getServer();
			cameraServer.setFPS(CameraConstants.fps);
			cameraServer.setResolution(CameraConstants.width, CameraConstants.height);
			cameraServer.setCompression(CameraConstants.compression);
			cameraServer.setDefaultCompression(CameraConstants.compression);

			Shuffleboard.getTab("Drive")
				.add("camera", SendableCameraWrapper.wrap(camera))
				// .addCamera("camera", "arm", cameraServ
				.withWidget(BuiltInWidgets.kCameraStream)
				.withSize(5, 4);
		} catch (edu.wpi.first.cscore.VideoException e) {
			DriverStation.reportError("Failed to get camera: " + e.toString(), e.getStackTrace());
		}
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
	 * predicate, or via the named factories in
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link CommandXboxController Xbox} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
	 */
	private void configureBindings() {
		/* driver */
		driverController.a().onTrue(teleopSwerve.toggleFieldRelativeCommand());
		driverController.b().onTrue(new InstantCommand(() -> swerve.zeroYaw()));
		driverController.x().whileTrue(new XStance(swerve));

		/* operator */
		operatorController.povLeft().and(operatorController.x())
			.whileTrue(new AprilTagVision(AprilTagVision.Routine.Left, vision, swerve));
		operatorController.povLeft().and(operatorController.a())
			.whileTrue(new AprilTagVision(AprilTagVision.Routine.Middle, vision, swerve));
		operatorController.povLeft().and(operatorController.b())
			.whileTrue(new AprilTagVision(AprilTagVision.Routine.Right, vision, swerve));

		operatorController.povDown().and(operatorController.a())
			.whileTrue(new RetroreflectiveVision(RetroreflectiveVision.Routine.Middle, vision, swerve));
		operatorController.povDown().and(operatorController.y())
			.whileTrue(new RetroreflectiveVision(RetroreflectiveVision.Routine.Top, vision, swerve));

		operatorController.povRight().and(operatorController.a())
			.whileTrue(new ObjectDetectionVision(ObjectDetectionVision.Routine.Ground, vision, swerve));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */

	public Command getAutonomousCommand() {
		return autoChooser.get();
	}
}
