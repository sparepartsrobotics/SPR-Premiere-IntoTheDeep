package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MecanumDrive;
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
@TeleOp (name = "SPR TeleOp")
public class SprTeleOp extends LinearOpMode{
    MecanumDrive srobot;
    public double frMotorPower = 1.0;
    public double flMotorPower = 1.0;
    public double brMotorPower = 1.0;
    public double blMotorPower = 1.0;
    public boolean hasBlock = false;

    //public double incTemp= 0.0;
    //public double decTemp= 1.0;
    public double inc = .005;
    public double incArm = .01;
    public double incArmUp = 1;
    public double telePosition = 1.0;

    public double clawArmUp = 0.38;//0.7;
    public double clawTiltUp = 0.28;//.3
    public double clawTiltMiddle = .675;
    public double clawArmDownHigh = .8;
    public double clawTiltDown = 1;//.87
    public double clawArmDownLow = .93;
    public double clawArmOut = 0.3;
    public int targetPos = 0; //2018;

    public int distanceToTarget = 0;
    public double clawOpen = 1.0;

    public double clawClose = 0;
    public boolean specimenIsMoved = false;
    public boolean isArmReleased = false;
    public boolean yoWhatsGood = false;
    public boolean specimenMoving = false;
    double joystick1LeftX,joystick1RightX,joystick1LeftY;

    public double angle1;
    public double angle2;

    @Override public void runOpMode(){
        srobot = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        waitForStart();
        initRobot();
        //While the opmode is running, the method controls is running and the telemetry is updating
        while (opModeIsActive())
        {
            robotMovement();
            teleArm();
            openCloseClaw();
            pushClawDown();
            rotateClaw();
            outtake();
            specimenClaw();
            specimen();
            linearSlide();
            increaseArm();
            decreaseArm();
            //testInverse();
            lights();
            /* if ((targetPos == 1045 && !specimenIsMoved) || (targetPos == 0 && specimenIsMoved)) {
                distanceToTarget = Math.abs(targetPos - srobot.specimenArm.getCurrentPosition());
                if (distanceToTarget > 0) {
                    double power = (double) distanceToTarget / targetPos;
                    srobot.specimenArm.setPower(power);
                } else if (distanceToTarget == 0) {
                    srobot.specimenArm.setPower(0.2);
                }
            } */
            telemetry.addData("time", time);
            //telemetry.addData("clawRotate", srobot.clawRotate.getPosition());
            //telemetry.addData("leftSlide", srobot.leftSlide.getCurrentPosition());
            //telemetry.addData("rightSlide", srobot.leftSlide.getCurrentPosition());
            telemetry.addData("slideTilt", srobot.linearSlideTilt.getCurrentPosition());
            //telemetry.addData("clawArm", srobot.clawArm.getPosition());
            //telemetry.addData("clawTilt", srobot.clawTilt.getPosition());
            //telemetry.addData("clawAngle", angle1);
            //telemetry.addData("armAngle", angle2);
            telemetry.addData("Specimen Arm", srobot.specimenArm.getCurrentPosition());
            telemetry.addData("Specimen Reset", srobot.specimenReset.isPressed());
            //telemetry.addData("targetPos", targetPos);
            //telemetry.addData("resetSlide", srobot.slideReset.isPressed());
            //telemetry.addData("resetTilt", srobot.tiltReset.isPressed());
            //telemetry.addData("hi", yoWhatsGood);
            telemetry.update();
        }

    }
    public void initRobot(){
        srobot.tiltStop.setPosition(1);
        srobot.specimenArm.setMode(STOP_AND_RESET_ENCODER);
        srobot.specimenArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        srobot.linearSlideTilt.setMode(STOP_AND_RESET_ENCODER);
        srobot.linearSlideTilt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        srobot.leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        srobot.rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        srobot.leftSlide.setMode(STOP_AND_RESET_ENCODER);
        srobot.rightSlide.setMode(STOP_AND_RESET_ENCODER);
        srobot.specimenRotate.setPosition(0);
        srobot.clawArm.setPosition(clawArmUp);
        srobot.bucket.setPosition(.25);
        srobot.clawTilt.setDirection(Servo.Direction.REVERSE);
        srobot.clawTilt.setPosition(clawTiltUp);
        srobot.specimenTilt.setPosition(0.2);
        srobot.teleArm.setPosition(1.0);
        srobot.specPush.setPosition(0);
    }

    public void lights() {
        if (time > 65 && time < 70) {
            srobot.LEDdriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        }
        else if (time > 70 && time < 105) {
            srobot.LEDdriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else if (time >= 105) {
            srobot.LEDdriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        } else {
            srobot.LEDdriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
        }
    }
    public void teleArm(){
        if (gamepad1.right_trigger > 0.0) {
            srobot.clawArm.setPosition(clawArmDownHigh);
            srobot.clawTilt.setDirection(Servo.Direction.REVERSE);
            srobot.clawTilt.setPosition(clawTiltDown);
            telePosition -= inc;
        }
        if (gamepad1.left_trigger > 0.0) {
            telePosition += inc;
        }

        if (telePosition > 1.0) {
            telePosition = 1.0;
        } else if (telePosition > 0.5) {
            telePosition = 0.5;
        }

        srobot.teleArm.setPosition(telePosition);

        if (gamepad1.a) {
            if (srobot.clawTilt.getPosition() != clawTiltUp) {
                //srobot.clawArm.setPosition(0.4);
                srobot.clawArm.setPosition(clawArmUp);
                srobot.bucket.setPosition(.25);
                //sleep(500);
                double waitTime = time+0.5;
                while (waitTime > time) {
                    robotMovement();
                }
                if (srobot.claw.getPosition() < 0.5) {
                    srobot.claw.setPosition(clawClose);
                    srobot.clawRotate.setPosition(.9);

                } else if (srobot.claw.getPosition() >= 0.5) {
                    srobot.claw.setPosition(clawOpen);
                    srobot.clawRotate.setPosition(0.47);
                }
                double waitTime2 = time+0.25;
                while (waitTime2 > time) {
                    robotMovement();
                }
                srobot.clawTilt.setDirection(Servo.Direction.REVERSE);
                srobot.clawTilt.setPosition(clawTiltUp);
                srobot.teleArm.setPosition(0.9);
            }
            telePosition = 1.0;
            //srobot.teleArm.setPosition(telePosition);
        }
    }
    public void pushClawDown() {
        if (gamepad1.dpad_down) {
            if (srobot.clawRotate.getPosition() == 0.9) {
                srobot.clawArm.setPosition(clawArmDownLow);
                srobot.clawTilt.setDirection(Servo.Direction.REVERSE);
                srobot.clawTilt.setPosition(.97);
            }
            else {
                srobot.clawArm.setPosition(clawArmDownLow);
                srobot.clawTilt.setDirection(Servo.Direction.REVERSE);
                srobot.clawTilt.setPosition(.95);
            }
        }
    }
    public void specimenClaw(){
        //open
        if(gamepad2.left_bumper){
            srobot.specimenClaw.setPosition(0.2);
        }
        //close
        else if(gamepad2.right_bumper){
            srobot.specimenClaw.setPosition(0.0);
        }
    }
    //Function is just used for inverse kinematics testing
    //Remove after it is working on intake arm
    public void testInverse() {
        if (gamepad2.dpad_up) {
            moveIntakeArm(180, 110);
        }
        if (gamepad2.dpad_left) {
            srobot.clawArm.setPosition(0.53);
            srobot.clawTilt.setPosition(1);
        }
        if (gamepad2.dpad_right) {
            srobot.clawArm.setPosition(1.0);
            srobot.clawTilt.setPosition(0.5);
        }
    }
    public void specimen(){
        //Grab off wall
        if(gamepad2.x) {
            targetPos = 0;
            if (!specimenIsMoved) {
                targetPos = 1045;
                specimenIsMoved = true;
            }
            srobot.specimenTilt.setPosition(1);
            srobot.specimenRotate.setPosition(0.0);
            srobot.specimenClaw.setPosition(0.2);
            srobot.specimenArm.setTargetPosition(targetPos);
            srobot.specimenArm.setMode(RUN_TO_POSITION);
            srobot.specimenArm.setPower(0.8);
            //specimenReset is true when not pressed
            while (opModeIsActive() && srobot.specimenReset.isPressed()) {
                distanceToTarget = Math.abs(targetPos - srobot.specimenArm.getCurrentPosition());
                if (distanceToTarget > 0) {
                    double power = (double) distanceToTarget / (targetPos + Math.abs(targetPos - 1045));
                    srobot.specimenArm.setPower(power);
                } else if (distanceToTarget == 0) {
                    srobot.specimenArm.setPower(0.2);
                }
                robotMovement();
                teleArm();
                pushClawDown();
                rotateClaw();
                openCloseClaw();
            }
            srobot.specimenArm.setPower(0);
            srobot.specimenArm.setMode(STOP_AND_RESET_ENCODER);
            srobot.specimenArm.setMode(RUN_USING_ENCODER);
        }
        //put on high chamber
        else if(gamepad2.y){
            /* New Position */
            targetPos = -765; //255
            srobot.specimenTilt.setPosition(0.5);
            srobot.specimenArm.setTargetPosition(targetPos);
            srobot.specimenArm.setMode(RUN_TO_POSITION);
            srobot.specimenArm.setPower(0.8);
            double waitTime5 = time + 0.3;
            while (waitTime5 > time) {
                robotMovement();
            }
            srobot.specimenRotate.setPosition(.9);
        }
        //lower arm off high chamber
        if(gamepad2.b){
            /* New Position */
            targetPos = -855; //190
            srobot.specimenClaw.setPosition(0.2);
            srobot.specimenTilt.setPosition(.4);
            srobot.specimenArm.setTargetPosition(targetPos);
            srobot.specimenArm.setMode(RUN_TO_POSITION);
            srobot.specimenArm.setPower(0.8);
        }
        if(gamepad2.a) {
            /* New position */
            targetPos = -1045;
            srobot.specimenClaw.setPosition(0.0);
            srobot.specimenTilt.setPosition(0.2);
            srobot.specimenArm.setTargetPosition(targetPos);
            srobot.specimenArm.setMode(RUN_TO_POSITION);
            srobot.specimenArm.setPower(0.6);
            while (srobot.specimenArm.isBusy() && opModeIsActive()) {
                robotMovement();
            }
            srobot.specimenArm.setPower(0);
            srobot.specimenArm.setMode(RUN_USING_ENCODER);
        }
    }
    public void linearSlide(){
        if(gamepad1.x){
            srobot.teleArm.setPosition(.7);
            srobot.tiltStop.setPosition(.5);
            srobot.clawTilt.setDirection(Servo.Direction.REVERSE);
            srobot.clawTilt.setPosition(0.675);
            srobot.linearSlideTilt.setDirection(DcMotorSimple.Direction.REVERSE);
            srobot.linearSlideTilt.setTargetPosition(1500);
            srobot.linearSlideTilt.setMode(RUN_TO_POSITION);
            srobot.linearSlideTilt.setPower(1);
            //srobot.leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
            srobot.leftSlide.setTargetPosition(2225);
            srobot.leftSlide.setMode(RUN_TO_POSITION);
            srobot.rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
            srobot.rightSlide.setTargetPosition(2225);
            srobot.rightSlide.setMode(RUN_TO_POSITION);
            srobot.leftSlide.setPower(1);
            srobot.rightSlide.setPower(1);
            double waitTime4 = time + 0.4;
            while (time <= waitTime4) {
                robotMovement();
            }
            srobot.bucket.setPosition(.45);
        }
        else if(gamepad1.y) {
            srobot.bucket.setPosition(.25);
            srobot.clawTilt.setDirection(Servo.Direction.REVERSE);
            srobot.clawTilt.setPosition(0.675);
            srobot.linearSlideTilt.setDirection(DcMotorSimple.Direction.REVERSE);
            srobot.linearSlideTilt.setTargetPosition(0);
            srobot.linearSlideTilt.setMode(RUN_TO_POSITION);
            srobot.linearSlideTilt.setPower(1);
            //srobot.leftSlide.setTargetPosition(0);
            //srobot.rightSlide.setTargetPosition(0);
            srobot.leftSlide.setMode(RUN_USING_ENCODER);
            srobot.rightSlide.setMode(RUN_USING_ENCODER);
            srobot.leftSlide.setPower(-0.5);
            srobot.rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
            srobot.rightSlide.setPower(-0.5);
            srobot.teleArm.setPosition(0.7);
            //slideReset & tiltReset are true when not pressed
            while (opModeIsActive() && (srobot.slideReset.isPressed() || srobot.tiltReset.isPressed())) {
                robotMovement();
                teleArm();
                pushClawDown();
                rotateClaw();
                openCloseClaw();
                telemetry.addData("resetSlide", srobot.slideReset.isPressed());
                telemetry.addData("resetTilt", srobot.tiltReset.isPressed());
                telemetry.addData("slideTilt", srobot.linearSlideTilt.getCurrentPosition());
                telemetry.update();
            }
            srobot.leftSlide.setPower(0);
            srobot.rightSlide.setPower(0);
            srobot.linearSlideTilt.setPower(0);
            srobot.leftSlide.setMode(STOP_AND_RESET_ENCODER);
            srobot.rightSlide.setMode(STOP_AND_RESET_ENCODER);
            srobot.leftSlide.setMode(RUN_USING_ENCODER);
            srobot.rightSlide.setMode(RUN_USING_ENCODER);
            srobot.linearSlideTilt.setMode(RUN_USING_ENCODER);
        }
    }
    public void robotMovement(){
        joystick1LeftX = gamepad1.left_stick_x;
        joystick1LeftY = -gamepad1.left_stick_y;
        joystick1RightX = gamepad1.right_stick_x;

        flMotorPower = -joystick1LeftY - joystick1LeftX - joystick1RightX;
        blMotorPower  = -joystick1LeftY + joystick1LeftX - joystick1RightX;
        frMotorPower = -joystick1LeftY + joystick1LeftX + joystick1RightX;
        brMotorPower = -joystick1LeftY - joystick1LeftX + joystick1RightX;

        //runs when the right stick buttons is pressed down
        if (gamepad1.right_stick_button)
        {
            //boosts the robot's speed by 35%
            srobot.leftFront.setPower(flMotorPower * -1);
            srobot.rightFront.setPower(frMotorPower * -1);
            srobot.leftBack.setPower(blMotorPower * -1);
            srobot.rightBack.setPower(brMotorPower * -1);
        }
        //runs when the right stick is not pressed
        else
        {
            //speed of robot is normal
            srobot.leftFront.setPower(flMotorPower * -0.3);
            srobot.rightFront.setPower(frMotorPower * -0.3);
            srobot.leftBack.setPower(blMotorPower * -0.3);
            srobot.rightBack.setPower(brMotorPower * -0.3);
        }

    }


    public void rotateClaw(){

        if(gamepad1.dpad_right){
            srobot.clawRotate.setPosition(0.47);
        }
        //Rotated
        else if(gamepad1.dpad_left){
            srobot.clawRotate.setPosition(0.9);
        }
    }
    public void openCloseClaw(){
        if(gamepad1.left_bumper){
            srobot.claw.setPosition(clawClose);
            if (srobot.teleArm.getPosition() == clawArmUp) {
                srobot.teleArm.setPosition(1.0);
            }
        }
        else if(gamepad1.right_bumper){
            srobot.claw.setPosition(clawOpen);
            if (srobot.teleArm.getPosition() == clawArmUp) {
                srobot.teleArm.setPosition(1.0);
            }
        }
    }
    public void outtake() {
        if (gamepad1.b) {
            srobot.clawTilt.setDirection(Servo.Direction.REVERSE);
            srobot.clawTilt.setPosition(0.7);
            double waitTime3 = time+0.5;
            while (waitTime3 > time) {
                robotMovement();
            }
            srobot.clawArm.setPosition(2*clawArmOut);
            srobot.bucket.setPosition(0.8);
        }

    }
    public void increaseArm(){
        if (gamepad2.right_trigger > 0.0) {
            targetPos += incArmUp;
            srobot.specimenArm.setTargetPosition(targetPos);
            srobot.specimenArm.setMode(RUN_TO_POSITION);
            srobot.specimenArm.setPower(1);
        }


    }
    public void decreaseArm(){
        if (gamepad2.left_trigger > 0.0) {
            targetPos -= incArm;
            srobot.specimenArm.setTargetPosition(targetPos);
            srobot.specimenArm.setMode(RUN_TO_POSITION);
            srobot.specimenArm.setPower(1);
        }
    }

    public void moveIntakeArm(int x, int y) {
        //desired x and y pos (mm)
        double posX = x;
        double posY = y;
        //arm and claw length (mm)
        double armLength = 135.82;
        double clawLength = 160.624;
        //angles of servos based of x & y
        double armAngle = 0;
        double clawAngle = 0;
        //check for extraneous values
        if (-25 < x && x < 25) {
            //x cannot be between -25 and 25
            posX = 25;
        }
        if (armLength + clawLength < Math.sqrt(posX * posX + posY * posY)) {
            //the longest it could reach is the length of the arm
            posY = posY - ((Math.sqrt(posX * posX + posY * posY)) - (armLength + clawLength));
        }
        if (posX >= 25) {
            clawAngle = Math.acos(((posX * posX) + (posY * posY) - (armLength * armLength) - (clawLength * clawLength)) / (2 * armLength * clawLength));
            armAngle = Math.atan(posY/posX) - Math.atan((clawLength * Math.sin(clawAngle)) / (armLength + clawLength * Math.cos(clawAngle)));
        } else if (posX <= -25) {
            clawAngle = -1 * Math.acos(((posX * posX) + (posY * posY) - (armLength * armLength) - (clawLength * clawLength)) / (2 * armLength * clawLength));
            armAngle = Math.atan(posY/posX) + Math.atan((clawLength * Math.sin(clawAngle)) / (armLength + clawLength * Math.cos(clawAngle)));
        }
        //If angle is negative, find the co-terminal angle between 0 & 2 Pi radians
        /**if (clawAngle <= 0) {
         clawAngle += 2 * Math.PI;
         }*/
        //if (armAngle <= 0) {
        //     armAngle += 2 * Math.PI;
        // }
        clawAngle += (3 * Math.PI) / 4;
        armAngle += (3 * Math.PI) / 2;
        //Servos have 270 degrees of rotation, so divide by 3 Pi/2 to have correct ratio
        //Also had to use new variables to set servo position
        double clawTheta = clawAngle / ((3 * Math.PI) / 2);
        double armTheta = armAngle / ((3 * Math.PI) / 2);
        //Used for debugging
        angle1 = clawAngle / Math.PI;
        angle2 = armAngle / Math.PI;
        //Set the positions for the servos
        srobot.clawArm.setPosition(armTheta);
        srobot.clawTilt.setPosition(clawTheta);
    }

    public void checkSpecimenMovement() {
        if (srobot.specimenArm.isBusy()) {
            specimenMoving = true;
        } else if (!srobot.specimenArm.isBusy()) {
            specimenMoving = false;
        }
    }
}