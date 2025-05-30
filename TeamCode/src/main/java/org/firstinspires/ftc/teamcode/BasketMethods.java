package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
public class BasketMethods {
    public Servo clawArm;
    public Servo bucket;
    public Servo clawRotate;
    public Servo clawTilt;
    public Servo claw;
    public Servo tiltStop;

    public Servo teleArm, specimenClaw, specimenRotate, specimenTilt;

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

    BasketMethods(HardwareMap hardwareMap){
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

    }
    public class releaseSampleEnd implements Action{
        @Override

        public boolean run(@NonNull TelemetryPacket packet){
            clawArm.setPosition(0.2);
            bucket.setPosition(.8);
            return false;
        }
    }
    public class linSlideHigh implements Action{
        @Override

        public boolean run(@NonNull TelemetryPacket packet){
            claw.setPosition(clawClose);
            tiltStop.setPosition(.5);
            clawTilt.setPosition(clawTiltUp);
            clawArm.setPosition(clawArmUp);
            clawTilt.setDirection(Servo.Direction.REVERSE);
            clawTilt.setPosition(0.675);
            bucket.setPosition(.25);
            linearSlideTilt.setDirection(DcMotorSimple.Direction.REVERSE);
            linearSlideTilt.setTargetPosition(1500);
            linearSlideTilt.setMode(RUN_TO_POSITION);
            linearSlideTilt.setPower(1);
            leftSlide.setTargetPosition(2225);
            leftSlide.setMode(RUN_TO_POSITION);
            rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
            rightSlide.setTargetPosition(2225);
            rightSlide.setMode(RUN_TO_POSITION);
            leftSlide.setPower(1);
            rightSlide.setPower(1);
            teleArm.setPosition(.7);
            return false;
        }
    }
    public class bringClawDown implements Action{
        @Override

        public boolean run(@NonNull TelemetryPacket packet){
            claw.setPosition(clawClose);
            clawTilt.setDirection(Servo.Direction.REVERSE);
            clawTilt.setPosition(clawTiltDown);
            clawArm.setPosition(clawArmDownLow);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            claw.setPosition(clawOpen);
            try {
                Thread.sleep(700);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            clawTilt.setPosition(clawTiltUp);
            clawArm.setPosition(clawArmUp);
            return false;
        }
    }
    public class bringClawDown2 implements Action{
        @Override

        public boolean run(@NonNull TelemetryPacket packet){
            clawRotate.setPosition(.9);
            claw.setPosition(clawClose);
            clawTilt.setDirection(Servo.Direction.REVERSE);
            clawTilt.setPosition(clawTiltDown);
            clawArm.setPosition(clawArmDownLow);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            claw.setPosition(clawOpen);
            try {
                Thread.sleep(700);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            clawRotate.setPosition(0.47);
            clawArm.setPosition(clawArmUp);
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            clawTilt.setPosition(clawTiltUp);
            return false;
        }
    }
    public class resetLin implements Action{
        @Override

        public boolean run(@NonNull TelemetryPacket packet){
            clawTilt.setDirection(Servo.Direction.REVERSE);
            clawTilt.setPosition(0.675);
            bucket.setPosition(.25);
            linearSlideTilt.setDirection(DcMotorSimple.Direction.REVERSE);
            linearSlideTilt.setTargetPosition(0);
            linearSlideTilt.setMode(RUN_TO_POSITION);
            linearSlideTilt.setPower(1);
            leftSlide.setTargetPosition(0);
            leftSlide.setMode(RUN_TO_POSITION);
            rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
            rightSlide.setTargetPosition(0);
            rightSlide.setMode(RUN_TO_POSITION);
            leftSlide.setPower(0.5);
            rightSlide.setPower(0.5);
            teleArm.setPosition(.7);
            return false;
        }
    }
    public class initRob implements Action{
        @Override

        public boolean run(@NonNull TelemetryPacket packet){
            linearSlideTilt.setDirection(DcMotorSimple.Direction.REVERSE);
            linearSlideTilt.setTargetPosition(0);
            linearSlideTilt.setMode(RUN_TO_POSITION);
            linearSlideTilt.setPower(1);
            return false;
        }
    }
    public class parkRob implements Action{
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

    public Action releaseSample(){
        return new releaseSampleEnd();
    }
    public Action linSlideHighBasket(){
        return new linSlideHigh();
    }
    public Action resetLinSlide(){
        return new resetLin();
    }
    public Action bringClawArmDown(){
        return new bringClawDown();
    }
    public Action bringClawArmDown2(){
        return new bringClawDown2();
    }

    public Action initLinSlide(){
        return new initRob();
    }
    public Action park(){
        return new parkRob();
    }
}
