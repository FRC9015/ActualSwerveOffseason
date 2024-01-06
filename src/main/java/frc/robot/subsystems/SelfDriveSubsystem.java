package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotSelf;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SelfDriveSubsystem extends SubsystemBase{
    
    //brings in other subsystems to be used
    private SwerveModule drive;
    private LimelightInterface limelight;
    private XboxController controller;
    private RobotSelf robotSelf;
    
    //makes the subsystems exist and usable
    public SelfDriveSubsystem(XboxController controller, RobotSelf robotSelf, LimelightInterface limelight,SwerveModule drive) {
        this.controller = controller;
        this.robotSelf = robotSelf;
        this.limelight = limelight;
        this.drive = drive;
        
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
        //makes seperate X, Y, Area values outside of the limelightinterface
        double x = limelight.getX();
        double y = limelight.getY();
        double area = limelight.getArea();
        
        //makes sure that selfdrive is true
        if(robotSelf.getselfdrive()){
            //checks for a tag using the limelightinterface commands
            if(limelight.TagCheck()){
                //updates values
                x = limelight.getX();
                y = limelight.getY();
                area = limelight.getArea();
                // uses drive system to drive
                drive.limelightcontrol(x, y, area);
                
            }
        }
    }
}
