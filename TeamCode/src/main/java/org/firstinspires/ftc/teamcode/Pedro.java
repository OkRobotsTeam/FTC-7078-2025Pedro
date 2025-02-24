package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "Pedro for teh WIN", group = "Examples")
public class Pedro extends LinearOpMode {
    IDRobot robot = new IDRobot();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;


    private final Pose startPose = new Pose(38, 66, 0);
    private final Pose firstPose = new Pose(47, 35, 0);
    private final Pose secondPose = new Pose(55, 27, 0);
    private final Pose thirdPose = new Pose(30, 27, 0);
    private final Pose fourthPose = new Pose(35, 16, 0);
    private final Pose fifthPose = new Pose(36, 65, 0);
    private final Pose sixthPose = new Pose(35, 33, 180);
    private final Pose firstControlPoint = new Pose(12, 42, 0);
    private final Pose secondControlPoint = new Pose(13, 25, 0);

    private final Pose thirdControlPoint = new Pose(80, 40, 0);
    private final Pose fourthControlPoint = new Pose(86, 27, 0);
    private final Pose fifthControlPoint = new Pose(61, 32, 0);

    public void runOpMode() {
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        robot.init(this);
        FConstants fconstants = new FConstants();
        LConstants lconstants = new LConstants();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);


        waitForStart();

//        robot.move(-20,1,false);
        robot.move(10, 1, true);
        robot.runIntakeIn();
        sleep(20);
        robot.stopIntake();
//        robot.rotateArm(1);
//        while (robot.armRotation.getCurrentPosition() < 1400) {
//            sleep(20);
//        }
        robot.rotateArmToPosition(1700);
        robot.extendArmToPosition(-10);
        while (robot.armRotation.getCurrentPosition() < 400) {
            sleep(20);
        }
        robot.extendArmToPosition(2400);
        robot.setWristPosition(0.7);
        while (robot.armExtension.getCurrentPosition() < 2300 || robot.armRotation.getCurrentPosition() < 1600) {
            sleep(20);
        }

        robot.move(64, 1, true);
        robot.runIntakeIn();

        while (robot.armExtension.getCurrentPosition() < 1800) {
            sleep(20);
        }
        robot.extendArmToPosition(0);
        while (robot.armExtension.getCurrentPosition() > 630) {
            sleep(20);
        }

        follower.setStartingPose(startPose);

        PathChain path = follower.pathBuilder()
                //.addPath(new BezierCurve(new Point(startPose), new Point(firstControlPoint), new Point(firstPose))).setConstantHeadingInterpolation(0)
                .addPath(new BezierCurve(new Point(startPose), new Point(firstControlPoint), new Point(firstPose))).setConstantHeadingInterpolation(0)
                .addPath(new BezierCurve(new Point(firstPose), new Point(fifthControlPoint), new Point(secondPose))).setConstantHeadingInterpolation(0)
//                .addParametricCallback(0.2, () -> {robot.rotateArmToPosition(1700);})
//                .addParametricCallback(0.4, () -> {robot.extendArmToPosition(2400);})
//                .addParametricCallback(0.4, () -> {robot.setWristPosition(0.7);})
//                .addParametricCallback(1, () -> {robot.extendArmToPosition(0);})
//                .addParametricCallback(1, () -> {robot.rotateArmToPosition(1500);})
                //.addPath(new BezierCurve(new Point(firstPose), new Point(firstControlPoint), new Point(secondControlPoint), new Point(thirdControlPoint), new Point(secondPose))).setConstantHeadingInterpolation(0)
//                .addParametricCallback(0.6, () -> {robot.rotateArmToPosition(200);})
                .addPath(new BezierLine(new Point(secondPose), new Point(thirdPose))).setConstantHeadingInterpolation(0)
                .addPath(new BezierCurve(new Point(thirdPose), new Point(fourthControlPoint), new Point(fourthPose))).setConstantHeadingInterpolation(0)
//                .addPath(new BezierLine(new Point(fourthPose), new Point(fifthPose))).setConstantHeadingInterpolation(180)
//                .addPath(new BezierCurve(new Point(fifthPose), new Point(fourthControlPoint), new Point(sixthPose))).setConstantHeadingInterpolation(0)
//                .addPath(new BezierCurve(new Point(sixthPose), new Point(fifthControlPoint), new Point(seventhPose))).setConstantHeadingInterpolation(180)
//                .addPath(new BezierCurve(new Point(seventhPose), new Point(sixthControlPoint), new Point(eighthPose))).setConstantHeadingInterpolation(0)
                .build();

        PathChain toScoring = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fifthPose), new Point(sixthPose))).setLinearHeadingInterpolation(180, 0)
                .addPath(new BezierLine(new Point(sixthPose), new Point(fifthPose))).setLinearHeadingInterpolation(0, 180)
                .build();
        PathChain backToPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fifthPose), new Point(sixthPose))).setLinearHeadingInterpolation(180, 0)
                .addPath(new BezierLine(new Point(sixthPose), new Point(fifthPose))).setLinearHeadingInterpolation(0, 180)
                .build();


        doPath(path, "path", true, 100000);
        robot.odo.update();
        Debug.println("Weird X: ", robot.odo.getPosition().getX(DistanceUnit.CM), " Y: ", robot.odo.getPosition().getY(DistanceUnit.CM));
        robot.turn(180, 1);
        Debug.println("Weird X: ", robot.odo.getPosition().getX(DistanceUnit.CM), " Y: ", robot.odo.getPosition().getY(DistanceUnit.CM));

        robot.strafe(-40+robot.odo.getPosition().getY(DistanceUnit.CM), 1, false);
        robot.rotateArmToPosition(350);
        robot.setWristPosition(0.9);
        robot.extendArmToPosition(1800);
        robot.rotateArmToPosition(-240);
        robot.runIntakeIn();
        sleep(2000);
        robot.extendArmToPosition(500);
        robot.rotateArmToPosition(1700);
        robot.setWristPosition(0.7);
        sleep(2000);
        doPath(toScoring,"To Scoring" , false, 6000);


        waitForA();


        robot.extendArmToPosition(2400);
        while (robot.armExtension.getCurrentPosition() < 2300 || robot.armRotation.getCurrentPosition() < 1600) {
            sleep(20);
        }
        robot.move(10, 1, true);
        robot.runIntakeIn();
        while (robot.armExtension.getCurrentPosition() < 1800) {
            sleep(20);
        }
        robot.extendArmToPosition(0);
        while (robot.armExtension.getCurrentPosition() > 630) {
            sleep(20);
        }

        doPath(backToPickup, "back to pickup", true, 100000);
        
    }





    public void doPath(PathChain path, String pathName, boolean holdEnd, long timeout) {
        pathTimer = new Timer();
        double maxPower = 0.3;
        System.out.println("Starting path "+ pathName);
        follower.resetOffset();
//        Debug.println("PathInfo:  0:  X: " ,path.getPoint(0).getX(), " Y: ", path.getPoint(0).getY(), " 1 X: " ,path.getPoint(1).getX(), " Y: ", path.getPoint(1).getY());
        follower.followPath(path, holdEnd);
        follower.setMaxPower(0.3);
        while (follower.isBusy()  && opModeIsActive() ) {
            maxPower = Math.min(maxPower+0.05, 1);
            follower.setMaxPower(maxPower);
            follower.update();
            Point currentPathPoint = follower.getCurrentPath().getPoint(follower.getCurrentTValue());

            Debug.println("Path ", pathName, " T ", follower.getCurrentTValue(), " X (", follower.getPose().getX(), "|" , currentPathPoint.getX(), ") Y (",  follower.getPose().getY(),"|",currentPathPoint.getY(),")");
            robot.tickSleep();
            if (pathTimer.getElapsedTime() > timeout) {
                System.out.println("ERROR: PATH TIMEOUT");
                return;
            }
            telemetry.addData("Path", pathName);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();

        }
        System.out.println("Done with path "+ pathName);
        robot.setPowers(0,0,0,0);
        robot.setBraking();
        Debug.println("Path ", pathName, " T ", follower.getCurrentTValue(), " X ", follower.getPose().getX(), " Y " , follower.getPose().getY());

    }

    public void waitForA() {
        while (opModeIsActive() && gamepad1.a == false) {
            sleep(50);
        }
    }

}

