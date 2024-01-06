// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Constants.SwerveConstants.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DefaultDrive extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public DefaultDrive() {
		addRequirements(swerve);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double vx = driveController.getLeftX(); // X - Direction Velocity
		double vy = -driveController.getLeftY(); // Y - Direction Velocity
		double w = -driveController.getRightX(); // Rotational Velocity 
		double mag = Math.hypot(vx, vy); // Find fastest path between desired x and y velocities
		double ma2 = MathUtil.applyDeadband(mag, 0.1); // Apply deadband for x and y velocities to avoid joystick jitter
		w = MathUtil.applyDeadband(w, 0.1); // Apply rotational velocity deadband to avoid joystick jitter
		
		// Change range of values to a circle rather than square map to avoid conflict between motors
		double theta = Math.atan2(vy, vx); 
		vx = cos(theta) * ma2 * maxSpeed;
		vy = sin(theta) * ma2 * maxSpeed; // The maxSpeed coefficient prevents overload on the motor and excessive motor speed
		w = w * maxSpeed;

		ChassisSpeeds speeds =
				ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, w, imu.yaw()); // Finalize and set speeds
		swerve.drive(speeds);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
