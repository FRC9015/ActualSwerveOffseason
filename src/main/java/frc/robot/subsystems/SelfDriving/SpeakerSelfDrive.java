package frc.robot.subsystems.SelfDriving;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotSelf;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.Swerve.SwerveModule;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SpeakerSelfDrive extends SubsystemBase{

    //brings in other subsystems to be used
    private SwerveSubsystem drive;
    private LimelightInterface limelight;
    private CommandXboxController controller;
    private RobotSelf robotSelf;
    
    //makes the subsystems exist and usable
    public SpeakerSelfDrive(CommandXboxController controller, RobotSelf robotSelf, LimelightInterface limelightInterface, SwerveSubsystem swerve) {
        this.controller = controller;
        this.robotSelf = robotSelf;
        this.limelight = limelightInterface;
        this.drive = swerve;
        
    }
    @Override
    public void periodic(){
        // if(controller.getHID().getYButtonPressed() && !robotSelf.getAmpSelf()){//need to find out what the y button is in the command controller
        //     //toggles the speaker boolean for doing speaker self drive
        //     robotSelf.toggleSpeakerSelf();
        //     //puts speakerSelf onto smartdashboard
        //     SmartDashboard.putBoolean("SpeakerSelf",robotSelf.getSpeakerSelf());
        // }

        //makes variables for the X Y Area and Distance of the limelight in SpeakerSelfDrive
        double x = limelight.getX();
        double y = limelight.getY();
        double area = limelight.getArea();
        double distance = limelight.getDistance();

        if(robotSelf.getAmpSelf()){
            //updates variables
            x = limelight.getX();
            y = limelight.getY();
            area = limelight.getArea();
            distance = limelight.getDistance();
        }
    }
}
