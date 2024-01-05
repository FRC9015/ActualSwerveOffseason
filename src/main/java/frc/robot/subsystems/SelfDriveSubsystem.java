package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotSelf;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SelfDriveSubsystem {
    //private DriveSubsystem drive;
    private LimelightInterface limelight;
    private XboxController controller;
    private RobotSelf robotSelf;
    
    public SelfDriveSubsystem(XboxController controller, RobotSelf robotSelf, LimelightInterface limelight) {//, DriveSubsystem drive
        this.controller = controller;
        this.robotSelf = robotSelf;
        this.limelight = limelight;
        //this.drive = drive;
        
    }
    @Override
    public void periodic(){
        //checks to see if the X button has been pressed
        if(controller.getXButtonReleased()){
            //toggles selfdrive boolean
            robotSelf.toggleselfdrive();
            //puts selfdrive onto smartdashboard
            SmartDashboard.putBoolean("Self Drive", robotSelf.getselfdrive());
        }
    }
}
