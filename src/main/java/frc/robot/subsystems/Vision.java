package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.core.util.LimelightHelpers;
import frc.robot.core.util.LimelightHelpers.*;

public class Vision extends SubsystemBase {
    private static Vision instance;
    public static Vision getInstance() {
        if(instance != null) instance = new Vision();
        return instance;
    }

    private NetworkTable networkTable;
    private NetworkTableEntry tv, tx, ty, ta;
    private LimelightResults llresults;
    private int numAprilTags;
    private boolean hasTarget = false;
    private double currentX, currentY, currentA;

    private Vision() {
        networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        tv = networkTable.getEntry("tv"); // Whether limelight detects any valid targets 0, 1
        tx = networkTable.getEntry("tx"); // Horizontal offset from crosshair to target (-27, 27)
        ty = networkTable.getEntry("ty"); // Vertical offset from crosshair to target (-20.5, 20.5)
        ta = networkTable.getEntry("ta"); // Target area (Between 0% and 100%)
    }

    public boolean targetInSight() {
        return hasTarget;
    }

    public double getHorizontalAngle() {
        return currentX;
    }

    public double getVerticalAngle() {
        return currentY;
    }

    public double getCurrentArea() {
        return currentA;
    }

    public LimelightTarget_Fiducial getClosestTarget(LimelightTarget_Fiducial[] target_Fiducials) {
        if (numAprilTags > 1) {
            LimelightTarget_Fiducial closestFiducial = null;

            for (LimelightTarget_Fiducial tF : target_Fiducials) {
                if (closestFiducial == null)
                    closestFiducial = tF;
                else {
                    if (tF.getTargetPose_CameraSpace().getTranslation().getDistance(null) <
                            closestFiducial.getTargetPose_CameraSpace().getTranslation().getDistance(null))
                        closestFiducial = tF;
                }
            }
            return closestFiducial;
        }
        return target_Fiducials[0];
    }

    @Override
    public void periodic() {
        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        //post to shuffleboard periodically
        var llTab = Shuffleboard.getTab("limelight");

        llTab.addDouble("x", () -> x);
        llTab.addDouble("y", () -> y);
        llTab.addDouble("area", () -> area);

        llresults = LimelightHelpers.getLatestResults("");
        numAprilTags = llresults.targetingResults.targets_Fiducials.length;

        hasTarget = numAprilTags > 0; // (tv.getDouble(0.0) < 1.0) ? false : true;
        currentX = tx.getDouble(0.0);
        currentY = ty.getDouble(0.0);
        currentA = ta.getDouble(0.0);
    }
}

