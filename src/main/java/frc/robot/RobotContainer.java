package frc.robot;

import frc.robot.constants.Constants;
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
import frc.robot.commands.grabber.OpenGrabber;
import frc.robot.commands.swerve.ChargeStationBalance;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.swerve.XStance;
import frc.robot.commands.vision.AprilTagVision;
import frc.robot.commands.vision.ObjectDetectionVision;
import frc.robot.commands.vision.RetroreflectiveVision;

import edu.wpi.first.wpilibj.DriverStation;
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

	public final GyroIO gyro = Robot.isReal()
		? new GyroIOPigeon2(Constants.pigeonId)
		: new GyroIOSim();

	/* subsystems */
	public final Swerve swerve = new Swerve(gyro);
	public final Vision vision = new Vision();
	public final Arm arm = new Arm();
	public final Elevator elevator = new Elevator();
	public final Grabber grabber = new Grabber();

	public final CommandXboxController driverController = new CommandXboxController(
		OperatorConstants.driverControllerPort);
	public final CommandXboxController operatorController = new CommandXboxController(
		OperatorConstants.operatorControllerPort);

	private LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto mode");

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		DriverStation.silenceJoystickConnectionWarning(true);

		// will be automatically scheduled when no other scheduled commands require swerve
		swerve.setDefaultCommand(new TeleopSwerve(swerve, driverController.getHID()));

		configureBindings();

		// setup autos
		autoChooser.addDefaultOption("nothing", new InstantCommand());
		autoChooser.addOption("PP test", Autos.simpleWall(swerve));

		// for testing
		autoChooser.addOption("drive characterization",
			new FeedForwardCharacterization(swerve, true, swerve::runCharacterization, swerve::getCharacterizationVelocity));

		SmartDashboard.putData("do balancing", new ChargeStationBalance(swerve));
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
		driverController.a().onTrue(new InstantCommand(() -> swerve.toggleFieldRelative()));
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

		// todo: will using normal buttons conflict with the pov stuff? is there an exclusive bind?

		operatorController.rightTrigger(0.3).whileTrue(new ManualArmControl(arm, operatorController));
		operatorController.a().onTrue(new SetArmPosition(arm)); // reset arm

		operatorController.leftTrigger(.3).whileTrue(new ManualElevatorControl(elevator, operatorController));
		operatorController.a().onTrue(new SetElevatorPosition(elevator, ElevatorConstants.lowHeight_m)); // low
		operatorController.b().onTrue(new SetElevatorPosition(elevator, ElevatorConstants.midHeight_m)); // mid
		operatorController.y().onTrue(new SetElevatorPosition(elevator, ElevatorConstants.highHeight_m)); // high

		operatorController.leftBumper().onTrue(new OpenGrabber(grabber, GrabberConstants.openingTime_s));
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