package frc.robot.subsystems.Swerve;

import static frc.robot.Constants.Constants.*;

import java.util.Optional;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotSelf;
import frc.robot.Constants.SwerveModuleConfiguration;
import frc.robot.subsystems.IMU;



public class SwerveSubsystem extends SubsystemBase {
	
	private RobotSelf robotSelf = new RobotSelf();
	private IMU imu = new IMU();

	public SwerveDrivePoseEstimator pose_est;
	
	private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			new Translation2d(robotLength / 2, robotWidth / 2), // NW
			new Translation2d(robotLength / 2, -robotWidth / 2), // NE
			new Translation2d(-robotLength / 2, -robotWidth / 2), // SE
			new Translation2d(-robotLength / 2, robotWidth / 2) // SW
			);

	private SwerveModule[] modules = new SwerveModule[] {
		new SwerveModule(SwerveModuleConfiguration.NW, "NW"),
		new SwerveModule(SwerveModuleConfiguration.NE, "NE"),
		new SwerveModule(SwerveModuleConfiguration.SE, "SE"),
		new SwerveModule(SwerveModuleConfiguration.SW, "SW"),
	};

	public void drive(ChassisSpeeds speeds) {
		
		
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
		for (int i = 0; i < modules.length; i++) {
			modules[i].setState(states[i]);
		}
	}

	@Override
	public void periodic() {
		//if statment is so that the telop wont run if selfdrive is on.
		if (!robotSelf.getAmpSelf() && !robotSelf.getSpeakerSelf()){
			for (SwerveModule module : modules) {
				module.telop();
			}
		}
		//odametry
		pose_est.update(new Rotation2d(imu.yaw()), getPositions());
		Optional<EstimatedRobotPose> vis_pos = photon.getEstimatedGlobalPose(pose_est.getEstimatedPosition());
		if (vis_pos.isPresent()) {
			EstimatedRobotPose new_pose = vis_pos.get();
			pose_est.addVisionMeasurement(new_pose.estimatedPose.toPose2d(), new_pose.timestampSeconds);
		}
		}
	
	
	public void getOffsets() {
		for (SwerveModule module : modules) module.fixOffset();
	}

	public Command printOffsets() {
		return new InstantCommand(this::getOffsets, this);
	}
}
