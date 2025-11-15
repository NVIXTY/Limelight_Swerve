package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;
    
    public LimelightSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }
 
    public boolean hasValidTarget(){ //tv is 1.0 if there is a target, 0.0 if there is no target. Returns true if it sees a AprilTag
        return limelightTable.getEntry("tv").getDouble(0) == 1;
    }

    public double getTx(){ // Horizontal offset / error (-27 degrees to 27 degrees) I think
        return limelightTable.getEntry("tx").getDouble(0);
    }

    public double getTy(){ // Vertical offset / error from target (-20.5 degrees to 20.5 degrees) I think
        return limelightTable.getEntry("ty").getDouble(0);
    }

    public double getTa(){  //estimates distance 
        return limelightTable.getEntry("ta").getDouble(0);
    }

    /** Returns the detected AprilTag ID (or -1 if no tag is found) */
    public int getTagID() {
        double rawTagID = limelightTable.getEntry("tid").getDouble(-1);
        int tagID = (int) Math.round(rawTagID); // Ensures correct integer conversion
        System.out.println("📸 Limelight Detected Tag ID: " + tagID); // Debugging
        return tagID;
    }
    

    /* botpose gives the robot’s X, Y, Rotation based on AprilTags.
        botpose[0] = X Position (meters) WE USE THIS
        botpose[1] = Y Position (meters) WE USE THIS 
        botpose[2] = Z Position (height)
        botpose[3] = Roll (tilt sideways)
        botpose[4] = Pitch (tilt forward/backward)
        botpose[5] = Yaw (Rotation in degrees) WE USE THIS
     */
    public double[] getBotPose() {
        String poseKey = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
            ? "botpose_wpiblue"
            : "botpose_wpired";

        return limelightTable.getEntry(poseKey).getDoubleArray(new double[6]);
    }
    @Override
    public void periodic() { //prints bots pose 
        // System.out.println("Limelight X: " + getBotPose()[0] + " Y: " + getBotPose()[1] + " Rotation: " + getBotPose()[5]);
    }
}