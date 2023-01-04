package frc.robot.Autonomous;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.LimelightStates;

public class AutonomousBasePID extends AutonomousBase{
    public static final double KP_DRIVE = 0.008333333; //0.025; -- divide this by 3 //0.03333333;//previously 0.05--multiplied by 2/3
    public static final double KD_DRIVE = 0.012666667; //0.038 -- divide by 3; //0.04666667; //previously 0.07
    public static final double KP_ANGLE = 0.0035; // 0.004; //0.0046667; // multiplied 0.007 by two thirds //0.00042 --divided 0.007/(5/3); //0.007; // proportional - 0.01+ was less accurate 
    public static final double KD_ANGLE = 0.005; //0.01; // derivitive
    public static final double MAXVOLT = 0.35; // 30% max output
    public static final double MINVOLT = 0.15; //previously .12 on 3/24 4:44pm// 10%
    public static LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

    public AutonomousBasePID(){ 
    }

    @Override
    public void driveForward(double distance){//drivePD method 
        double velocity = driveSubsystem.countsToInches(driveSubsystem.getDriveVelocity());
        double error = distance - driveSubsystem.countsToInches(driveSubsystem.getTicks()); 
        //System.out.println("Original error: "+ error);
        //System.out.println("Original velocity: "+ velocity);
        double voltage = (error*KP_DRIVE) + (velocity*KD_DRIVE);
        //System.out.println("Original Voltage: " + voltage);
        if(Math.abs(voltage) > MAXVOLT){
            voltage = Math.signum(voltage) * MAXVOLT;
        }
        if(Math.abs(voltage) < MINVOLT){
            voltage = Math.signum(voltage) * MINVOLT;
        }
        driveSubsystem.driveTank(voltage, voltage);
        //System.out.println("voltage: " + voltage);
        //System.out.println("error: " + error);
    }

    @Override
    public void driveForwardVision(double distance){//drivePD method 
        double velocity = driveSubsystem.countsToInches(driveSubsystem.getDriveVelocity());
        double error = distance - driveSubsystem.countsToInches(driveSubsystem.getTicks()); 
        //System.out.println("Original error: "+ error);
        //System.out.println("Original velocity: "+ velocity);
        double voltage = (error*KP_DRIVE) + (velocity*KD_DRIVE);
        //System.out.println("Original Voltage: " + voltage);
        if(Math.abs(voltage) > MAXVOLT){
            voltage = Math.signum(voltage) * MAXVOLT;
        }
        if(Math.abs(voltage) < MINVOLT){
            voltage = Math.signum(voltage) * MINVOLT;
        }
        if(error >= -10 && error <= 10){
            driveSubsystem.driveTank(0,0);
            //LimelightSubsystem.setState(LimelightStates.SHOOT);
        } else {
            driveSubsystem.driveTank(voltage, voltage);
        }
        //System.out.println("voltage: " + voltage);
        //System.out.println("error: " + error);
    }

    @Override
    public void turnToAnglePD(double degrees){
        double velocity = driveSubsystem.getRate(); // get rate of change of gyro
        double error = degrees - driveSubsystem.getDegrees(); // where we want to be - where we currently are (in degrees)
        //System.out.println("Our Degrees: " + driveSubsystem.getDegrees()); 
        double voltage = (error*KP_ANGLE) + (velocity*KD_ANGLE);
        //System.out.println("Original voltage: " + voltage);
        if(Math.abs(voltage) > MAXVOLT){
            voltage = Math.signum(voltage) * MAXVOLT;
        }
        if(Math.abs(voltage) < MINVOLT){
            voltage = Math.signum(voltage) * MINVOLT;
        }
        System.out.println("Our Voltage: " + voltage);
        
        driveSubsystem.driveTank(voltage, -voltage); //left, right
    
    }

    @Override
    public void turnToAnglePDVision(double degrees){
        double velocity = driveSubsystem.getRate(); // get rate of change of gyro
        double error = degrees - driveSubsystem.getDegrees(); // where we want to be - where we currently are (in degrees)
        //System.out.println("Our Degrees: " + driveSubsystem.getDegrees()); 
        double voltage = (error*KP_ANGLE*0.1) + (velocity*KD_ANGLE*0.1);
        //System.out.println("Original voltage: " + voltage);
        if(Math.abs(voltage) > MAXVOLT){
            voltage = Math.signum(voltage) * MAXVOLT;
        }
        if(Math.abs(voltage) < MINVOLT){
            voltage = Math.signum(voltage) * MINVOLT;
        }
        System.out.println("Our Voltage: " + voltage);

        if(Math.abs(error) <= 10){
            driveSubsystem.driveTank(0,0);
            limelightSubsystem.tx_2 = limelightSubsystem.getTx();
            System.out.println("done centering; moving on to move");
            LimelightSubsystem.setState(LimelightStates.MOVE);
        } else {
            driveSubsystem.driveTank(voltage, -voltage); //left, right
        }

        System.out.println("error: " + error);
    
    }

}
