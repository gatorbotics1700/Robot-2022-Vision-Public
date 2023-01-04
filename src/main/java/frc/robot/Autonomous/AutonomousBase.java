package frc.robot.Autonomous;

import frc.robot.subsystems.*;
import frc.robot.subsystems.IntakeSubsystem.IntakeStates;
import frc.robot.subsystems.SensorSubsystem.SensorStates;
import frc.robot.subsystems.ShootSubsystem.ShootState;
import frc.robot.subsystems.TransitionSubsystem.TransitionState;

public class AutonomousBase{
    public AutonomousBase(){}

    public static enum AutonomousStates{
        DRIVING1,
        DRIVING2,
        DRIVING3,
        DRIVING4,
        DRIVING5,
        SHOOTING, 
        STOPPING, 
        TURNING1,
        TURNING2,
        TURNING3,
        OUTTAKE,
        INTAKE;
    }

    public static enum Paths{
        BOOP1,
        BOOP3,
        SHOOTANDCROSS,
        SHOOTTWICE,
        SMI,
        SM,
        ALTSHOOTTWICE,
        SMIO,
        ONLYSHOOT, 
        BARELYCROSS, 
        DSHOOTTWICE, 
        DALTSHOOTTWICE; 
    }

    public DriveSubsystem driveSubsystem = new DriveSubsystem();
    public ShootSubsystem shootSubsystem = new ShootSubsystem();
    public IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public TransitionSubsystem transitionSubsystem = new TransitionSubsystem();
    public SensorSubsystem sensorSubsystem = new SensorSubsystem();
    
    public static AutonomousStates state; 
    public static Paths path;
    
    //every non constant, reset at approp time in init then robot.java 
    public static final double MSFORINTAKE = 1.5 * 1000;
    public static final double MSFORDRIVE = 1 * 1000; //driving backward for shootandcross
    public static double shootingStartTime;
    public double intakeStartTime;
    public double drive1StartTime;
    public static final double DISTANCETOLERANCE = 0.5; //change based on testing
    public static final double ANGTOL = 2;
    public static final double ANG3TOL = 6;
    public Boolean isOnSecondShot;
    public static boolean isSwap;

    public void init(){
        intakeStartTime = 0;
        drive1StartTime = 0;
        isSwap = false;
        isOnSecondShot = false;
        driveSubsystem.resetEncoders();
        System.out.println("encoders reset");
        driveSubsystem.resetGyro();
        state = AutonomousStates.SHOOTING;
        ShootSubsystem.hasShot = false;
        ShootSubsystem.preShootSavedMillis = System.currentTimeMillis();
        shootingStartTime = System.currentTimeMillis(); //starting shooting start time after we hit the idealRPM
    }

    public void driveForward(double distance){}

    public void driveForwardVision(double distance){}

    public void turnToAnglePD(double degrees){}

    public void turnToAnglePDVision(double degrees){}

    public void periodic(double firstDistance, double angle1, double secondDistance, double angle2, double thirdDistance, double angle3, double fourthDistance){
        double error;
        double turnError;

        System.out.println("STATE: " + state);
    
        shootSubsystem.periodic(true); //SKETCH AF >:)
        if(state == AutonomousStates.SHOOTING){
            System.out.println("hasShot: " + ShootSubsystem.hasShot);
            sensorSubsystem.setState(SensorStates.OFF);
            if(Math.abs(shootSubsystem.getRPM()) < ShootSubsystem.SHOOTRPM){ // not at ideal RPM - warm up shooter
                shootSubsystem.setState(ShootState.START);
                ShootSubsystem.preShootSavedMillis = System.currentTimeMillis();
                System.out.println("preshootsaved millis #1 - last value is the last value before moving to actually shooting the ball (aka when warming up)");
                System.out.println("has shot" + ShootSubsystem.hasShot);
            } else if(ShootSubsystem.hasShot){ // have finished a shot
                if (!isOnSecondShot){
                    driveSubsystem.resetEncoders(); 
                    driveSubsystem.resetGyro(); 
                    drive1StartTime = System.currentTimeMillis(); 
                    state = AutonomousStates.DRIVING1;
                    if (path == Paths.ALTSHOOTTWICE || path == Paths.SHOOTTWICE) {
                        isSwap = true;
                    }
                } else { // done with second shot (aka has shot)
                    state = AutonomousStates.STOPPING;
                } 
            } else { // at ideal RPM -- shoot the ball
                System.out.println("subtraction: " + (System.currentTimeMillis() - shootingStartTime) + "vs. 3s");
                ShootSubsystem.isTwice = false;
                System.out.println("SHOOTING FIRST BALL");
                shootSubsystem.setState(ShootState.SHOOT1);
                System.out.println("WE HAVE TURNED ON THE TRANSITION MOTORS FOR THE FINAL SHOT (suppsoedly)");
                transitionSubsystem.setState(TransitionState.ON); //added bc shoot1 isnt doing it
            }
        } else if(state == AutonomousStates.DRIVING1){
            if (path == Paths.SHOOTANDCROSS) { //TIMER VERSION
                if (System.currentTimeMillis() < drive1StartTime + MSFORDRIVE){
                    driveSubsystem.driveTank(-0.3, -0.3);
                } else {
                    setState(AutonomousStates.STOPPING);
                }
            } else { //PD VERSION
                error = firstDistance - driveSubsystem.countsToInches(driveSubsystem.getTicks());
                System.out.println("Error: " + error);
                shootSubsystem.shooterOff();
                System.out.println("ticks: " + driveSubsystem.getTicks());
                System.out.println("driving 1");
                System.out.println("First distance: " + firstDistance);
                driveForward(firstDistance);
                if(Math.abs(error) <= DISTANCETOLERANCE){
                    System.out.println("final error: " + error);
                    driveSubsystem.driveTank(0, 0);
                    driveSubsystem.resetEncoders();
                    driveSubsystem.resetGyro();
                    state = AutonomousStates.TURNING1;
                    if (path == Paths.SM){
                        setState(AutonomousStates.STOPPING);
                    }
                }
            }
        } else if(state == AutonomousStates.TURNING1){
            System.out.println("turning 1: " + angle1);
            //System.out.println("Stopping turning");
            turnError = angle1 - driveSubsystem.getDegrees();
            //System.out.println("DriveSubsystem.getDegrees: " + driveSubsystem.getDegrees());
            System.out.println("Turn Error: " + turnError);
            turnToAnglePD(angle1);

            // if error of angle is less than the angleTolerance, do the following line:
            if(Math.abs(turnError)<=ANGTOL){
                System.out.println("Final turn error: " + turnError);
                driveSubsystem.driveTank(0, 0);
                driveSubsystem.resetEncoders();
                driveSubsystem.resetGyro();
                setState(AutonomousStates.DRIVING2);
                transitionSubsystem.setState(TransitionState.ON);
                sensorSubsystem.setState(SensorStates.ON);
                intakeSubsystem.setState(IntakeStates.INTAKING);
            } 
        } else if(state == AutonomousStates.DRIVING2){
            error = secondDistance - driveSubsystem.countsToInches(driveSubsystem.getTicks());
            System.out.println("Error: " + error);
            driveForward(secondDistance);
            if(Math.abs(error) <= DISTANCETOLERANCE){
                driveSubsystem.driveTank(0, 0);
                driveSubsystem.resetEncoders();
                driveSubsystem.resetGyro();
                //intakeStartTime = System.currentTimeMillis(); for time based alternative
                transitionSubsystem.setState(TransitionState.ON);
                sensorSubsystem.setState(SensorStates.ON);
                intakeSubsystem.setState(IntakeStates.INTAKING);
                state = AutonomousStates.INTAKE;
            }
        }else if(state == AutonomousStates.INTAKE){  
            if(TransitionSubsystem.state == TransitionState.STOP_PD){
                System.out.println("done");
                intakeSubsystem.setState(IntakeStates.OFF);
                if(path == Paths.DSHOOTTWICE || path == Paths.DALTSHOOTTWICE || path == Paths.SMI){
                    state = AutonomousStates.STOPPING;
                }else{
                    state = AutonomousStates.TURNING2;
                }
                driveSubsystem.resetEncoders();
                driveSubsystem.resetGyro();
            }
            //time based alternative
            //if(System.currentTimeMillis() - intakeStartTime > millisecondsForIntaking){
            //     intakeSubsystem.setState(IntakeStates.OFF);
            //     state = AutonomousStates.STOPPING;
            //     //state = AutonomousStates.TURNING2;
            //     driveSubsystem.resetEncoders();
            //     driveSubsystem.resetGyro();
            // }
        
        }else if(state == AutonomousStates.TURNING2){
            System.out.println("turning 2");
            turnError = angle2 - driveSubsystem.getDegrees();
            turnToAnglePD(angle2);

            // if error of angle is less than the angleTolerance, do the following line:
            if(Math.abs(turnError)<=ANGTOL){
                System.out.println("Final turn error: " + turnError);
                driveSubsystem.driveTank(0, 0);
                driveSubsystem.resetEncoders();
                driveSubsystem.resetGyro();
                setState(AutonomousStates.DRIVING3);
            } 
        }else if(state == AutonomousStates.DRIVING3){
            System.out.println("driving 3");
            error = thirdDistance - driveSubsystem.countsToInches(driveSubsystem.getTicks());
            driveForward(thirdDistance);
            
            if(Math.abs(error) <= DISTANCETOLERANCE){
                //  System.out.println("final error: " + error);
                driveSubsystem.driveTank(0, 0);
                driveSubsystem.resetEncoders();
                driveSubsystem.resetGyro();
                state = AutonomousStates.TURNING3;
            }
        }else if(state == AutonomousStates.TURNING3){
            System.out.println("turning 3");
            turnError = angle3 - driveSubsystem.getDegrees();
            turnToAnglePD(angle3);
            
            if(Math.abs(turnError)<=ANG3TOL){
                System.out.println("turn error: " + turnError);
                System.out.println("angle tolerance: " + ANGTOL);
                driveSubsystem.driveTank(0, 0);
                driveSubsystem.resetEncoders();
                driveSubsystem.resetGyro();
                setState(AutonomousStates.DRIVING4);

            }
        }else if(state == AutonomousStates.DRIVING4){
            System.out.println("driving 4");
            error = fourthDistance - driveSubsystem.countsToInches(driveSubsystem.getTicks());
            driveForward(fourthDistance);
            
            if(Math.abs(error) <= DISTANCETOLERANCE){
                driveSubsystem.driveTank(0, 0);
                driveSubsystem.resetEncoders();
                driveSubsystem.resetGyro();
                intakeStartTime = System.currentTimeMillis();
                if(path == Paths.BOOP1 || path == Paths.BOOP3){
                    state = AutonomousStates.OUTTAKE;
                } else if (path == Paths.SHOOTTWICE || path == Paths.ALTSHOOTTWICE){ //Shoot twice path 
                    System.out.println("shooting the second time");
                    isOnSecondShot = true;
                    ShootSubsystem.hasShot = false;
                    ShootSubsystem.preShootSavedMillis = System.currentTimeMillis();
                    System.out.println("preshoot saved millis - if already at ideal RPM -- in driving 4 last part of path");
                    shootingStartTime =  System.currentTimeMillis(); 
                    state = AutonomousStates.SHOOTING;
                } else if (path == Paths.SMIO){
                    state = AutonomousStates.OUTTAKE;
                }
            }
        } else if(state == AutonomousStates.OUTTAKE){
            System.out.println("Outtaking");
            sensorSubsystem.setState(SensorStates.OFF);
            if(System.currentTimeMillis() - intakeStartTime < MSFORINTAKE){
                transitionSubsystem.setState(TransitionState.REVERSE);
                intakeSubsystem.setState(IntakeStates.OUTTAKING); // call to the setMotorsOn method, parameter isPositive = false so that motors runs opposite direction so we can outtake
            } else{
                setState(AutonomousStates.STOPPING);
            }
        }else if(state == AutonomousStates.STOPPING){
            System.out.println("stopping");
            driveSubsystem.driveTank(0,0);
            intakeSubsystem.setState(IntakeStates.OFF);;
            transitionSubsystem.setState(TransitionState.OFF);
            shootSubsystem.setState(ShootState.OFF);
            isSwap = false;
        }
        System.out.println(path);
    }


    public void setState(AutonomousStates newState){
        state = newState;
    }

    public void setPath(Paths newPath){
        path = newPath;
    }

}