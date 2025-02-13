package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "Specimen2", group = "Examples")
public class Specimen2 extends LinearOpMode {
    IDRobot robot = new IDRobot();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;


    private final Pose firstPose = new Pose(40, 65, 0);
    private final Pose secondPose = new Pose(20, 65, 0);
    private final Pose thirdPose = new Pose(65, 24, 0);
    private final Pose fourthPose = new Pose(19, 15, 0);
    private final Pose fifthPose = new Pose(65, 54, 0);
    private final Pose parkControlPose = new Pose(19, 15, 0);
    private Path firstMove, secondMove, currentPath;

    public void runOpMode() {
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        robot.init(this);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        firstMove = new Path(new BezierLine(new Point(firstPose), new Point(secondPose)));
        firstMove.setLinearHeadingInterpolation(firstPose.getHeading(), secondPose.getHeading());
        secondMove = new Path(new BezierCurve(new Point(secondPose), /* Control Point */ new Point(parkControlPose), new Point(thirdPose), new Point(fourthPose), new Point(fifthPose)));
        waitForStart();

        robot.move(-20,1,false);
        follower.setStartingPose(firstPose);
        doPath(firstMove, "First Move", false,1000);
        doPath(secondMove, "Second Move", true,1000);
        robot.setBraking();
        sleep(5000);
        robot.move(50,.5, true);
        sleep(5000);
        robot.turn(-45,1);
//        follower.setStartingPose(new Pose(0,0,0));

//        while(opModeIsActive()) {
//            follower.update();
//            robot.tickSleep();
//            telemetry.addData("Path", "DONE");
//            telemetry.addData("x", follower.getPose().getX());
//            telemetry.addData("y", follower.getPose().getY());
//            telemetry.addData("heading", follower.getPose().getHeading());
//            telemetry.update();
//        }
    }





    public void doPath(Path path, String pathName, boolean holdEnd,  long timeout) {
        pathTimer = new Timer();
        currentPath = path;
        System.out.println("Starting path "+ pathName);
        follower.followPath(path, holdEnd);
        while (follower.isBusy()  && opModeIsActive() ) {
            follower.update();
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
    }

}

