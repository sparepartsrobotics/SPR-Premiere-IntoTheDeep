package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;


// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Config
@Autonomous(name = "Blue Basket", group = "Autonomous")

public class BlueBasketAuto extends LinearOpMode{

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

    @Override public void runOpMode(){
        Pose2d initialPose =  new Pose2d(16,63, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap,initialPose);
        BasketMethods bmethods = new BasketMethods(hardwareMap);
        drive.clawRotate.setPosition(0.47);
        drive.linearSlideTilt.setMode(STOP_AND_RESET_ENCODER);
        drive.linearSlideTilt.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.linearSlideTilt.setTargetPosition(2600);
        drive.linearSlideTilt.setMode(RUN_TO_POSITION);
        drive.linearSlideTilt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        drive.linearSlideTilt.setPower(1);

        drive.claw.setPosition(clawClose);
        drive.specimenArm.setMode(STOP_AND_RESET_ENCODER);
        drive.specimenArm.setTargetPosition(1);
        drive.specimenArm.setMode(RUN_TO_POSITION);
        drive.specimenArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        drive.specimenArm.setPower(1);

        drive.leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.leftSlide.setMode(STOP_AND_RESET_ENCODER);
        drive.rightSlide.setMode(STOP_AND_RESET_ENCODER);
        drive.specimenClaw.setPosition(clawClose);
        drive.specimenRotate.setPosition(0);
        drive.clawArm.setPosition(clawArmUp);
        drive.bucket.setPosition(.25);
        drive.clawTilt.setDirection(Servo.Direction.REVERSE);
        drive.clawTilt.setPosition(clawTiltUp);
        drive.specimenTilt.setPosition(0.2);
        drive.teleArm.setPosition(1.0);

        waitForStart();

            if(isStopRequested()) return;

            Actions.runBlocking(
                    drive.actionBuilder(initialPose)
                            .stopAndAdd(bmethods.initLinSlide())
                            .strafeToLinearHeading(new Vector2d(54.5, 54.5), Math.toRadians(-140))
                            .stopAndAdd(bmethods.linSlideHighBasket())
                            .waitSeconds(1.3)
                            .stopAndAdd(bmethods.releaseSample())
                            .waitSeconds(.4)
                            .stopAndAdd(bmethods.resetLinSlide())
                            .strafeToLinearHeading(new Vector2d(48.5,39.5), Math.toRadians(-90))
                            .stopAndAdd(bmethods.bringClawArmDown())
                            .waitSeconds(.8)
                            .strafeToLinearHeading(new Vector2d(54,54), Math.toRadians(-140))
                            .stopAndAdd(bmethods.linSlideHighBasket())
                            .waitSeconds(1.3)
                            .stopAndAdd(bmethods.releaseSample())
                            .waitSeconds(.4)
                            .stopAndAdd(bmethods.resetLinSlide())
                            .strafeToLinearHeading(new Vector2d(59.5,39.5), Math.toRadians(-90))
                            .stopAndAdd(bmethods.bringClawArmDown())
                            .waitSeconds(.8)
                            .strafeToLinearHeading(new Vector2d(55, 55), Math.toRadians(-140))
                            .stopAndAdd(bmethods.linSlideHighBasket())
                            .waitSeconds(1.3)
                            .stopAndAdd(bmethods.releaseSample())
                            .waitSeconds(.4)
                            .stopAndAdd(bmethods.resetLinSlide())
                            .strafeToLinearHeading(new Vector2d(56,27), Math.toRadians(0))
                            .stopAndAdd(bmethods.bringClawArmDown2())
                            .waitSeconds(.8)
                            .strafeToLinearHeading(new Vector2d(54.5, 54.5), Math.toRadians(-140))
                            .stopAndAdd(bmethods.linSlideHighBasket())
                            .waitSeconds(1.3)
                            .stopAndAdd(bmethods.releaseSample())
                            .waitSeconds(.4)
                            .stopAndAdd(bmethods.resetLinSlide())
                            .stopAndAdd(bmethods.park())
                            .strafeTo(new Vector2d(40,15))
                            .strafeToLinearHeading(new Vector2d(23, 10), Math.toRadians(180), new TranslationalVelConstraint(75))
                            .build()
            );
    }
}
