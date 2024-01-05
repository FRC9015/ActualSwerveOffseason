package frc.robot;

//used so that selfdrive can be used in any file
public class RobotSelf {
    private boolean selfdrive = false;
    public boolean getselfdrive(){
        return selfdrive;
    }
    public void toggleselfdrive(){
        selfdrive = !selfdrive;
    }

     

}

