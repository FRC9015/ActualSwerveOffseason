// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SwerveModuleConfiguration;
import frc.robot.Constants.Constants.OperatorConstants;
import frc.robot.commands.DefaultDrive;
import frc.robot.subsystems.IMU;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.SelfDriving.AmpSelfDrive;
import frc.robot.subsystems.SelfDriving.SpeakerSelfDrive;
import frc.robot.subsystems.Swerve.SwerveModule;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	
	
	// The robot's subsystems and commands are defined here...
	public static final SwerveSubsystem swerve = new SwerveSubsystem();
	
	private RobotSelf robotSelf = new RobotSelf();
	


	  public static final IMU imu = new IMU();

	// Replace with CommandPS4Controller or CommandJoystick if needed
	public static final CommandXboxController driveController =
			new CommandXboxController(OperatorConstants.kDriverControllerPort);

			private final LimelightInterface limelightInterface = new LimelightInterface();
			private final AmpSelfDrive AmpSelfDrive = new AmpSelfDrive(driveController, limelightInterface, swerve);
			private final SpeakerSelfDrive SpeakerSelfDrive = new SpeakerSelfDrive(driveController, limelightInterface, swerve);
	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
		// CommandScheduler.getInstance().registerSubsystem(limelightInterface);
		// CommandScheduler.getInstance().registerSubsystem(AmpSelfDrive);
		// CommandScheduler.getInstance().registerSubsystem(SpeakerSelfDrive);
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
	 * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {
		swerve.setDefaultCommand(new DefaultDrive());

		driveController.a().onTrue(swerve.printOffsets());
		driveController.x().onTrue(new InstantCommand(() -> imu.zeroYaw()));
	}

    
}
