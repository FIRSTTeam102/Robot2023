package frc.robot;

import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.CameraConstants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.Constants.ShuffleboardConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.io.GyroIO;
import frc.robot.io.GyroIOPigeon2;
import frc.robot.io.GyroIOSim;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.arm.ManualArmControl;
import frc.robot.commands.elevator.ManualElevatorControl;
import frc.robot.commands.elevator.MoveElevatorBy;
import frc.robot.commands.grabber.GrabGrabber;
import frc.robot.commands.grabber.GrabGrabberUntilGrabbed;
import frc.robot.commands.grabber.ReleaseGrabber;
import frc.robot.commands.grabber.StopGrabber;
import frc.robot.commands.scoring.SetElevatorArmPosition;
import frc.robot.commands.swerve.ChargeStationBalance;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.swerve.XStance;
import frc.robot.commands.vision.AprilTagVision;
import frc.robot.commands.vision.ObjectDetectionVision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.Map;

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
	public final CommandGenericHID operatorConsole = new CommandGenericHID(
		OperatorConstants.operatorConsolePort);
	public final CommandJoystick operatorJoystick = new CommandJoystick(
		OperatorConstants.operatorJoystickPort);

	public final GyroIO gyro = Robot.isReal()
		? new GyroIOPigeon2(Constants.pigeonId)
		: new GyroIOSim();

	/* subsystems */
	public final Swerve swerve = new Swerve(gyro);
	public final Vision vision = new Vision();
	public final Arm arm = new Arm();
	public final Elevator elevator = new Elevator();
	public final Grabber grabber = new Grabber();

	private LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto mode");

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		DriverStation.silenceJoystickConnectionWarning(true);

		configureBindings();

		// setup autos
		autoChooser.addDefaultOption("nothing", new InstantCommand());
		autoChooser.addOption("2 piece", Autos.runAutoPath("2 piece", this));

		// for testing
		autoChooser.addOption("drive characterization",
			new FeedForwardCharacterization(swerve, true, swerve::runCharacterization, swerve::getCharacterizationVelocity));

		configureCameras();
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
		var teleopSwerve = new TeleopSwerve(
			() -> -driverController.getLeftY(),
			() -> -driverController.getLeftX(),
			() -> -driverController.getRightX(),
			driverController.getHID()::getLeftBumper, // override speed
			() -> driverController.getLeftTriggerAxis() > OperatorConstants.boolTriggerThreshold, // preceise mode
			swerve, arm, elevator);
		swerve.setDefaultCommand(teleopSwerve);

		driverController.rightTrigger(OperatorConstants.boolTriggerThreshold)
			.whileTrue(teleopSwerve.holdToggleFieldRelative());

		driverController.a().onTrue(teleopSwerve.toggleFieldRelative());
		driverController.y().onTrue(teleopSwerve.zeroYaw());
		driverController.x().whileTrue(new XStance(swerve));

		driverController.povUp().toggleOnTrue(new ChargeStationBalance(swerve));

		/*
		 * operator console
		 * G1 G2 G3 B4
		 * G5 Y6 Y7 B8
		 * R9 Y10 Y11 B12
		 * R13 R14 R15 B16
		 */

		// go to grid (green)
		operatorConsole.button(1)
			.whileTrue(new AprilTagVision(AprilTagVision.Routine.Left, vision, swerve));
		operatorConsole.button(2)
			.whileTrue(new AprilTagVision(AprilTagVision.Routine.Middle, vision, swerve));
		operatorConsole.button(3)
			.whileTrue(new AprilTagVision(AprilTagVision.Routine.Right, vision, swerve));

		// move arm/elevator/score
		operatorConsole.button(4) // double substation
			.onTrue(new SetElevatorArmPosition(elevator, arm,
				ElevatorConstants.doubleSubstationHeight_m, ArmConstants.doubleSubstationExtension_m));
		operatorConsole.button(7) // high cone
			.onTrue(new SetElevatorArmPosition(elevator, arm,
				ElevatorConstants.highConeHeight_m, ArmConstants.highConeExtension_m));
		// .whileTrue(new RetroreflectiveVision(RetroreflectiveVision.Routine.Top, vision, swerve));
		operatorConsole.button(8) // high cube
			.onTrue(new SetElevatorArmPosition(elevator, arm,
				ElevatorConstants.highCubeHeight_m, ArmConstants.highCubeExtension_m));
		operatorConsole.button(11) // mid cone
			.onTrue(new SetElevatorArmPosition(elevator, arm,
				ElevatorConstants.midConeHeight_m, ArmConstants.midConeExtension_m));
		// .whileTrue(new RetroreflectiveVision(RetroreflectiveVision.Routine.Middle, vision, swerve));
		operatorConsole.button(12) // mid cube
			.onTrue(new SetElevatorArmPosition(elevator, arm,
				ElevatorConstants.midCubeHeight_m, ArmConstants.midCubeExtension_m));
		operatorConsole.button(15) // all in
			.onTrue(new SetElevatorArmPosition(elevator, arm,
				ElevatorConstants.inHeight_m, ArmConstants.inExtension_m));
		operatorConsole.button(16) // ground
			.onTrue(new SetElevatorArmPosition(elevator, arm,
				ElevatorConstants.groundHeight_m, ArmConstants.groundExtension_m));

		operatorConsole.button(5) // aim at game piece
			.whileTrue(new ObjectDetectionVision(ObjectDetectionVision.Routine.Ground, vision, swerve));

		/*
		 * operator flight stick
		 */
		arm.setDefaultCommand(new ManualArmControl(arm, () -> -operatorJoystick.getX()));
		elevator.setDefaultCommand(new ManualElevatorControl(elevator, () -> operatorJoystick.getY()));
		// todo: what happens when both pressed?
		operatorJoystick.trigger()
			.whileTrue(new GrabGrabber(grabber));
		operatorJoystick.button(2)
			.whileTrue(new ReleaseGrabber(grabber));
		operatorJoystick.button(3)
			.onTrue(new MoveElevatorBy(elevator, ElevatorConstants.coneMoveDownHeight_m)
				.andThen(new ReleaseGrabber(grabber)));
		operatorJoystick.button(4)
			.onTrue(new StopGrabber(grabber));
		operatorJoystick.button(5)
			.onTrue(new GrabGrabberUntilGrabbed(grabber));
	}

	@SuppressWarnings("unused")
	private void configureCameras() {
		// camera
		try {
			var camera = CameraServer.startAutomaticCapture("arm", 0);
			camera.setConnectVerbose(0);
			camera.setFPS(CameraConstants.fps);
			camera.setResolution(CameraConstants.width, CameraConstants.height);
			if (CameraConstants.exposure >= 0)
				camera.setExposureManual(CameraConstants.exposure);
			else
				camera.setExposureAuto();
			// camera.setBrightness(0);

			// camera server is evil
			// var cameraServer = CameraServer.addSwitchedCamera("camera");
			// var cameraServer = CameraServer.startAutomaticCapture(camera);
			var cameraServer = (MjpegServer) CameraServer.getServer();
			cameraServer.setFPS(CameraConstants.fps);
			cameraServer.setResolution(CameraConstants.width, CameraConstants.height);
			cameraServer.setCompression(CameraConstants.compression);
			cameraServer.setDefaultCompression(CameraConstants.compression);

			Shuffleboard.getTab(ShuffleboardConstants.driveTab)
				.add("camera", SendableCameraWrapper.wrap(camera))
				// .addCamera("camera", "arm", cameraServ
				.withWidget(BuiltInWidgets.kCameraStream)
				.withSize(8, 6);
		} catch (edu.wpi.first.cscore.VideoException e) {
			DriverStation.reportError("Failed to get camera: " + e.toString(), e.getStackTrace());
		}

		if (Robot.isReal()) {
			var limelightCamera = new HttpCamera("limelightStream", "http://limelight.local:5800/stream.mjpg",
				HttpCameraKind.kMJPGStreamer);
			Shuffleboard.getTab(ShuffleboardConstants.driveTab)
				.add("limelight", limelightCamera)
				.withProperties(Map.of("show crosshair", false, "show controls", false))
				.withWidget(BuiltInWidgets.kCameraStream)
				.withSize(7, 6);
		}
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser.get();
	}

	Alert controllerAlert = new Alert("OI not connected/in wrong spots (changed in teleopInit)", AlertType.Error);

	public void updateOIAlert() {
		if (!driverController.getHID().isConnected()
			|| driverController.getHID().getName().indexOf("Xbox") < 0
			|| !operatorConsole.getHID().isConnected()
			|| operatorConsole.getHID().getName().indexOf("CyController") < 0
			|| !operatorJoystick.getHID().isConnected()
			|| operatorJoystick.getHID().getName().indexOf("Logitech") < 0) {
			controllerAlert.set(true);
		} else {
			controllerAlert.set(false);
		}
	}
}