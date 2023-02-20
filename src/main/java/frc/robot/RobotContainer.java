package frc.robot;

import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.CameraConstants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.GrabberConstants;
import frc.robot.io.GyroIO;
import frc.robot.io.GyroIOPigeon2;
import frc.robot.io.GyroIOSim;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.arm.ManualArmControl;
import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.elevator.ManualElevatorControl;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.grabber.CloseGrabber;
import frc.robot.commands.grabber.OpenGrabber;
import frc.robot.commands.swerve.ChargeStationBalance;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.swerve.XStance;
import frc.robot.commands.vision.AprilTagVision;
import frc.robot.commands.vision.GamePieceVision;
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

	/* subsystems */
	public final Swerve swerve = new Swerve(gyro);
	public final Vision vision = new Vision();
	public final Arm arm = new Arm();
	public final Elevator elevator = new Elevator();
	public final Grabber grabber = new Grabber();

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
		/* driver controller */

		// drive modes
		driverController.start().onTrue(teleopSwerve.toggleFieldRelativeCommand());
		driverController.back().onTrue(new InstantCommand(() -> swerve.zeroYaw()));
		driverController.leftStick().whileTrue(new XStance(swerve));

		// vision modes
		driverController.povLeft().and(driverController.x())
			.whileTrue(new AprilTagVision(AprilTagVision.Routine.BlueRedGridDoublesubstationLeft, vision, swerve));
		driverController.povLeft().and(driverController.a())
			.whileTrue(new AprilTagVision(AprilTagVision.Routine.BlueRedGridMiddle, vision, swerve));
		driverController.povLeft().and(driverController.b())
			.whileTrue(new AprilTagVision(AprilTagVision.Routine.BlueRedGridDoublesubstationRight, vision, swerve));

		driverController.povDown().and(driverController.a())
			.whileTrue(new RetroreflectiveVision(RetroreflectiveVision.Routine.BlueRedGridMiddle, vision, elevator, swerve));
		driverController.povDown().and(driverController.y())
			.whileTrue(new RetroreflectiveVision(RetroreflectiveVision.Routine.BlueRedGridTop, vision, elevator, swerve));

		driverController.povRight().and(driverController.a())
			.whileTrue(
				new GamePieceVision(GamePieceVision.Routine.Gamepiece, vision, elevator, arm, grabber, swerve));

		// arm modes
		driverController.rightTrigger(0.3).whileTrue(new ManualArmControl(arm, operatorController));
		driverController.rightTrigger().and(driverController.x())
			.onTrue(new SetArmPosition(arm, ArmConstants.resetLength_m));

		// elevator modes
		driverController.leftTrigger(.3).whileTrue(new ManualElevatorControl(elevator, operatorController));
		driverController.rightTrigger().and(driverController.x())
			.onTrue(new SetElevatorPosition(elevator, ElevatorConstants.resetHeight_m));
		driverController.rightTrigger().and(driverController.a())
			.onTrue(new SetElevatorPosition(elevator, ElevatorConstants.middleNodeHeight_m));
		driverController.rightTrigger().and(driverController.b())
			.onTrue(new SetElevatorPosition(elevator, ElevatorConstants.doubleSubstationHeight_m));
		driverController.rightTrigger().and(driverController.y())
			.onTrue(new SetElevatorPosition(elevator, ElevatorConstants.topNodeHeight_m));

		// grabber modes
		driverController.a().toggleOnTrue(new CloseGrabber(grabber, .5, GrabberConstants.closingTime_s));
		driverController.b().toggleOnTrue(new OpenGrabber(grabber, .4, GrabberConstants.openingTime_s));
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