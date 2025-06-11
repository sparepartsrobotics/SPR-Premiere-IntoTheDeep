package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
public class SpecimenMethods {
    public Servo clawArm;
    public Servo bucket;
    public Servo clawRotate;
    public Servo clawTilt;
    public Servo claw;
    public Servo tiltStop;

    public Servo teleArm, specimenClaw, specimenRotate, specimenTilt, specPush;

    public TouchSensor specimenReset;

    public double frMotorPower = 1.0;
    public double flMotorPower = 1.0;
    public double brMotorPower = 1.0;
    public double blMotorPower = 1.0;
    public boolean hasBlock = false;

    //public double incTemp= 0.0;
    //public double decTemp= 1.0;
    public double inc = 0.005;
    public double telePosition = 1.0;

    public double clawArmUp = 0.38;//0.7;
    public double clawTiltUp = 0.28;//.3
    public double clawTiltMiddle = .675;
    public double clawArmDownHigh = .8;
    public double clawTiltDown = 1;//.87
    public double clawArmDownLow = .96;
    public double clawArmOut = 0.3;

    public double clawOpen = 1.0;

    public double clawClose = 0;

    public DcMotorEx linearSlideTilt, leftSlide, rightSlide,  specimenArm;

    SpecimenMethods(HardwareMap hardwareMap){
        claw = hardwareMap.servo.get("claw");
        clawRotate = hardwareMap.servo.get("clawRotate");
        clawArm = hardwareMap.servo.get("clawArm");
        clawTilt = hardwareMap.servo.get("clawTilt");
        bucket = hardwareMap.servo.get("bucket");
        tiltStop = hardwareMap.servo.get("tiltStop");
        teleArm = hardwareMap.servo.get("teleArm");
        claw.setDirection(Servo.Direction.FORWARD);
        clawArm.setDirection(Servo.Direction.FORWARD);
        clawTilt.setDirection(Servo.Direction.FORWARD);
        clawRotate.setDirection(Servo.Direction.FORWARD);
        bucket.setDirection(Servo.Direction.FORWARD);
        tiltStop.setDirection(Servo.Direction.FORWARD);
        linearSlideTilt = hardwareMap.get(DcMotorEx.class, "linearSlideTilt");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        specimenArm = hardwareMap.get(DcMotorEx.class, "specimenArm");
        specimenTilt = hardwareMap.servo.get("specimenTilt");
        specimenRotate = hardwareMap.servo.get("specimenRotate");
        specimenClaw = hardwareMap.servo.get("specimenClaw");
        specPush = hardwareMap.servo.get("specPush");
        specimenReset = hardwareMap.touchSensor.get("specimenReset");
    }
    public class liftSpec implements Action{
        @Override

        public boolean run(@NonNull TelemetryPacket packet){
            specimenClaw.setPosition(clawClose);
            specimenRotate.setPosition(0.9);
            specimenTilt.setPosition(0.5);
            specimenArm.setTargetPosition(255);
            specimenArm.setMode(RUN_TO_POSITION);
            specimenArm.setPower(1);
            return false;
        }
    }
    public class pickUpSpec implements Action{
        @Override

        public boolean run(@NonNull TelemetryPacket packet){
            specimenTilt.setPosition(1.0);
            specimenRotate.setPosition(0.0);
            specimenClaw.setPosition(0.2);
            specimenArm.setTargetPosition(1022);
            specimenArm.setMode(RUN_TO_POSITION);
            specimenArm.setPower(.6);
            return false;
        }
    }
    public class closeCl implements Action{
        @Override

        public boolean run(@NonNull TelemetryPacket packet){
            specimenClaw.setPosition(clawClose);
            return false;
        }
    }
    public class openCl implements Action{
        @Override

        public boolean run(@NonNull TelemetryPacket packet){
            specimenClaw.setPosition(clawOpen);
            return false;
        }
    }
    public class spec implements Action{
        @Override

        public boolean run(@NonNull TelemetryPacket packet){
            specimenArm.setTargetPosition(500);
            specimenArm.setMode(RUN_TO_POSITION);
            specimenArm.setPower(1);
            return false;
        }
    }
    public class specPu implements Action{
        @Override

        public boolean run(@NonNull TelemetryPacket packet){
            specPush.setPosition(.28);
            return false;
        }
    }
    public class specPuCl implements Action{
        @Override

        public boolean run(@NonNull TelemetryPacket packet){
            specPush.setPosition(0);
            return false;
        }
    }
    public Action liftSpecimenArm(){
        return new liftSpec();
    }
    public Action pickupSpecimen(){
        return new pickUpSpec();
    }
    public Action closeClaw(){
        return new closeCl();
    }
    public Action openClaw(){
        return new openCl();
    }
    public Action specimen(){
        return new spec();
    }
    public Action specPush(){ return new specPu();}
    public Action specPuClose(){ return new specPuCl();}

}
