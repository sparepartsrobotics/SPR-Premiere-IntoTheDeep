package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
@Config
@Autonomous(name = "SpecimenBlue", group = "Autonomous")
public class SpecimenBlue extends LinearOpMode{
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
    public void runOpMode(){
        Pose2d initialPose = new Pose2d(-16, 63, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        BasketMethods bmethods = new BasketMethods(hardwareMap);
        SpecimenMethods smethods = new SpecimenMethods(hardwareMap);
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
        drive.specPush.setPosition(0);
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .afterTime(3.5, smethods.specPush())
                .stopAndAdd(bmethods.initLinSlide())
                .stopAndAdd(smethods.liftSpecimenArm())
                .strafeTo(new Vector2d(0,29))
                .stopAndAdd(smethods.openClaw())
                .lineToY(50)
                .strafeToLinearHeading(new Vector2d(-34,30), Math.toRadians(250))
                .strafeToSplineHeading(new Vector2d(-40,49), Math.toRadians(150))
                .stopAndAdd(smethods.specPuClose())
                .strafeToLinearHeading(new Vector2d(-47, 30), Math.toRadians(260));
        TrajectoryActionBuilder tab1x = drive.actionBuilder(new Pose2d(-48,30, Math.toRadians(250)))
                .stopAndAdd(smethods.specPush())
                .strafeToSplineHeading(new Vector2d(-48,50), Math.toRadians(150))
                .stopAndAdd(smethods.specPuClose())
                .strafeToLinearHeading(new Vector2d(-55,30), Math.toRadians(260))
                .stopAndAdd(smethods.specPush())
                .strafeToSplineHeading(new Vector2d(-55,49), Math.toRadians(150))
                .stopAndAdd(smethods.specPuClose())
                .stopAndAdd(smethods.pickupSpecimen())
                .strafeToLinearHeading(new Vector2d(-35,49), Math.toRadians(-90))
                .strafeTo(new Vector2d(-35,58));
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-48,57, Math.toRadians(-90)))
                .stopAndAdd(smethods.closeClaw())
                .waitSeconds(.1)
                .stopAndAdd(smethods.liftSpecimenArm())
                .strafeTo(new Vector2d(8,27))
                .waitSeconds(.2)
                .stopAndAdd(smethods.openClaw())
                .strafeTo(new Vector2d(8,48))
                .stopAndAdd(smethods.pickupSpecimen())
                .strafeTo(new Vector2d(-35,45))
                .strafeTo(new Vector2d(-35,56.5))
                .stopAndAdd(smethods.closeClaw())
                .waitSeconds(.1)
                .stopAndAdd(smethods.liftSpecimenArm())
                .strafeTo(new Vector2d(3,29))
                .waitSeconds(.2)
                .stopAndAdd(smethods.openClaw())
                .strafeTo(new Vector2d(3,48));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(6.5,48, Math.toRadians(-90)))
                .stopAndAdd(smethods.pickupSpecimen())
                .strafeTo(new Vector2d(-35,45))
                .strafeTo(new Vector2d(-35,56.5))
                .stopAndAdd(smethods.closeClaw())
                .waitSeconds(.2)
                .stopAndAdd(smethods.liftSpecimenArm())
                .strafeTo(new Vector2d(5,29))
                .waitSeconds(.2)
                .stopAndAdd(smethods.openClaw())
                .strafeTo(new Vector2d(5,48))
                .strafeTo(new Vector2d(-45,54));

//        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(4.5,48, Math.toRadians(-90)))
//                .strafeTo(new Vector2d(-35,45))
//                .stopAndAdd(smethods.pickupSpecimen())
//                .strafeTo(new Vector2d(-35,56.5))
//                .stopAndAdd(smethods.closeClaw())
//                .waitSeconds(.2)
//                .stopAndAdd(smethods.liftSpecimenArm())
//                .strafeTo(new Vector2d(0,29))
//                .stopAndAdd(smethods.openClaw())




        waitForStart();
            if(isStopRequested()) return;

            Actions.runBlocking(
                    new SequentialAction(
                            tab1.build(),
                            tab1x.build(),
                            tab2.build(),
                            tab3.build()

                    )

            );
    }

}
