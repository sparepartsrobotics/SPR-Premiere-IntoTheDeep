package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
@Config
@Autonomous(name = "SpecimenBlue", group = "Autonomous")
public class SpecimenBlue extends LinearOpMode{
    public void runOpMode(){
        Pose2d initialPose = new Pose2d(-16, 63, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        BasketMethods bmethods = new BasketMethods(hardwareMap);
        SpecimenMethods smethods = new SpecimenMethods(hardwareMap);
        waitForStart();
            if(isStopRequested()) return;

            Actions.runBlocking(
                    drive.actionBuilder(initialPose)
                            .stopAndAdd(bmethods.initLinSlide())
                            .stopAndAdd(smethods.liftSpecimenArm())
                            .strafeTo(new Vector2d(3,36))
                            .lineToY(38)
                            //.strafeTo(new Vector2d(-40,60))
                            .splineToLinearHeading(new Pose2d(-32,30, Math.toRadians(250)), Math.toRadians(-90))
                            .stopAndAdd(smethods.specPush())
                            .waitSeconds(1)
                            .strafeToLinearHeading(new Vector2d(-40,50), Math.toRadians(150))


                            .build());
    }

}
