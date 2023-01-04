package frc.robot.subsystems;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Autonomous.*;
import frc.robot.subsystems.IntakeSubsystem.IntakeStates;
import frc.robot.subsystems.SensorSubsystem.SensorStates;
import frc.robot.subsystems.ShootSubsystem.ShootState;
import frc.robot.subsystems.TransitionSubsystem.TransitionState;
import frc.robot.subsystems.TurretSubsystem;

public class LimelightSubsystem {
    /*
    TODOS/PROBLEMS

    1) robot bounces?

    2) commented out shooting & everything to do with turret

    3) make tx_2 for driving after centering

    */
    public final double MOUNTINGANGLE = 2.0; //degrees
    public final double HUBHEIGHT = 54.0; //inches : 104.0
    public final double ROBOTHEIGHT = 20.0; //inches 
    public final double IDEALDISTANCE = 40.0; //inches
    private static final AutonomousBasePID autonomousBasePID = new AutonomousBasePID();
    private static final TransitionSubsystem transitionSubsystem = new TransitionSubsystem();
    private static final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private static final ShootSubsystem shootSubsystem = new ShootSubsystem();
    private static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private static final SensorSubsystem sensorSubsystem = new SensorSubsystem();
    private static final TurretSubsystem turretSubsystem = new TurretSubsystem();
    public static double initialPosition;
    public static double tx_0;
    public static double tx_1;
    public static double tx_2;
    // this variable determines whether the Limelight has a valid target
    private static double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);

    // this variable determines the horizonal (x direction) error from the crosshair to the target
    private static double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);

    // this variable determines the vertical (y direction) error from the crosshair to the target
    private static double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);

    // target Area (0% of image to 100% of image)
    private static double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);

    public static void limelightData(){
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);
        SmartDashboard.putNumber("tv: ", tv);
        SmartDashboard.putNumber("tx: ", tx);
        SmartDashboard.putNumber("ty: ", ty);
        SmartDashboard.putNumber("ta: ", ta);
    }
    /*
`   pipeline declaring in c++
    std::shared_ptr<NetworkTable> networkTable = nt::NetworkTableInstance::GetDefault()
            .GetTable("limelight");
    networkTable->PutNumber("pipeline", pipeline);
    */

    public NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    

    public void setPipeline(double pipeline){
        //System.out.println("0 network table: " + networkTable.getEntry("pipeline").getDouble(0.0));
        networkTable.getEntry("pipeline").setNumber(pipeline);
        System.out.println("pipeline: "+ networkTable.getEntry("pipeline").getDouble(0.0));
        System.out.println("tv: " + networkTable.getEntry("tv").getDouble(0.0));
        //System.out.println("multi network table: " + multiNetworkTable.getEntry("pipeline").getDouble(0.0));
        //System.out.println("1 network table: " + networkTable.getEntry("pipeline").getDouble(0.0));
    }


    public static enum LimelightStates{
        CENTER, //make the turret and the robot line up 
        SCANZERO,
        SCANONE,
        TURNLOOSE,
        TURNFINE,
        MOVE,
        SHOOT,
        OFF;
    }

    public static LimelightStates state = LimelightStates.OFF;

    public void init(){
        setState(LimelightStates.SCANZERO);
        initialPosition = driveSubsystem.getTicks();
        tx_0 = 0.0;
        tx_1 = 0.0;
    }

    public void periodic(){
        if(state == LimelightStates.CENTER){
            align();
            System.out.println("aligning!");
        } else if (state == LimelightStates.SCANZERO){ //turret
            System.out.println("turret is scanning on pipeline zero");
            scanPipelineZero();
        } else if (state == LimelightStates.SCANONE) { //turret
            System.out.println("turret is scanning on pipeline one");
            scanPipelineOne();
        } else if (state == LimelightStates.TURNLOOSE){ //robot but only if we are wrong distance
            turnLoose();
        } else if (state == LimelightStates.TURNFINE){ //robot but only if we are wrong distance
            turnFine();
        } else if (state == LimelightStates.MOVE){
            moveRoboWithLimeLight();
            System.out.println("robot is moving");
        } else if (state == LimelightStates.SHOOT){
            shootSubsystem.setState(ShootState.SHOOT1);
        } else {
            shootSubsystem.setState(ShootState.OFF);
            intakeSubsystem.setState(IntakeStates.OFF);
            transitionSubsystem.setState(TransitionState.OFF);
            sensorSubsystem.setState(SensorStates.OFF);  
        }
    }

    public static void setState(LimelightStates newState){
        state = newState;
        if (newState == LimelightStates.MOVE || newState == LimelightStates.CENTER){
            initialPosition = driveSubsystem.getTicks();
        }
    }

    // turning tx and ty into distance (in)
    public double yDistanceFromIdeal(){
        double distanceFromTarget = (HUBHEIGHT - ROBOTHEIGHT)/Math.tan(Math.toRadians(MOUNTINGANGLE + ty));
        System.out.println("distance ftom target: " + distanceFromTarget);
        double distanceFromIdeal = distanceFromTarget - IDEALDISTANCE;
        System.out.println("distance from ideal: " + distanceFromIdeal);
        return distanceFromIdeal;
    }

    public void moveRoboWithLimeLight(){
        double finalDistance = yDistanceFromIdeal()/Math.cos(Math.toRadians(tx_2)); ///Math.cos(tx);
        System.out.println("final distance: " + finalDistance);
        if (Math.abs(finalDistance) >= 10){
            autonomousBasePID.driveForwardVision(-finalDistance);
        } else {
            setState(LimelightStates.SHOOT);
        }
        // if(Math.abs(initialPosition - driveSubsystem.getTicks()) >= 100 && driveSubsystem.getDriveVelocity() < 0.1 && driveSubsystem.getDriveVelocity() > -0.1){
        //     setState(LimelightStates.SHOOT);
        // }
    }

    public void align(){
        autonomousBasePID.turnToAnglePDVision(tx_0);//+tx_1);//turretSubsystem.getTicks()*9/256); 
        // turretSubsystem.turnToAnglePD(-tx_0-tx_1); //-turretSubsystem.getTareEncoder());
        //if(Math.abs(initialPosition - driveSubsystem.getTicks()) >= 100 && driveSubsystem.getDriveVelocity() < 0.1 && driveSubsystem.getDriveVelocity() > -0.1){
            //setState(LimelightStates.MOVE);
        //}
    }

    public void turnLoose(){
        // turretSubsystem.turnToAnglePD(tx);
        // if(turretSubsystem.getRate() < 0.1 && turretSubsystem.getRate() > -0.1){
        //     setState(LimelightStates.SCANONE);
        // }
        System.out.println("turret");
        setState(LimelightStates.SCANONE);
    }

    public void turnFine(){
        // turretSubsystem.turnToAnglePD(tx);
        // if(turretSubsystem.getRate() < 0.1 && turretSubsystem.getRate() > -0.1){
        //     if(yDistanceFromIdeal()<6.0 && yDistanceFromIdeal()>-6.0){
        //         setState(LimelightStates.SHOOT);
        //     }else{
        //         setState(LimelightStates.CENTER);
        //     }
        // }
        System.out.println("turret to center");
        setState(LimelightStates.CENTER);
    }

    public boolean isThereTarget(){
        if(tv != 0){
            return true;
        }
        return false; 
    }

    public double getTv(){
        return networkTable.getEntry("tv").getDouble(0.0);
    }

    public double getTx(){
        return tx;
    }

    public void scanPipelineZero(){
        setPipeline(0.0);
        if(isThereTarget()){
            tx_0 = tx;
            System.out.println("tx_0: " + tx_0);
            setState(LimelightStates.TURNLOOSE);
        } else {
            System.out.println("limelight screwed up and/or driver do better");
        }
    }

    public void scanPipelineOne(){
        setPipeline(1.0);
        if(isThereTarget()){
            tx_1 = tx;
            System.out.println("tx_0: " + tx_0);
            System.out.println("tx_1: "+ tx_1);
            setState(LimelightStates.TURNFINE); 
        } else {
            System.out.println("not working :(");
        }
    }

//a bit worried about the infinate loop possibilities between scanzero and scanone 
}
