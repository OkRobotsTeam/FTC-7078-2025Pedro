/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;



@Autonomous(name="Robot: Id Observation Specimen", group="Robot")

public class IdObservationSpecimen extends LinearOpMode {
IDRobot robot = new IDRobot();



    @Override
    public void runOpMode() {
        robot.init(this);

        telemetry.addData(">", "Robot Ready.  Press START.");


        telemetry.update();

        sleep(300);
        waitForStart();


        robot.odo.setPosition(robot.zeroPose);
        robot.move(10, 1, true);
        robot.runIntakeIn();
        sleep(20);
        robot.stopIntake();
        robot.rotateArmToPosition(1700);
        robot.extendArmToPosition(-10);
        while (robot.armRotation.getCurrentPosition() < 400) {
            sleep(20);
        }
        robot.extendArmToPosition(3800);
        robot.setWristPosition(0.7);
        while (robot.armExtension.getCurrentPosition() < 3700 || robot.armRotation.getCurrentPosition() < 1600) {
            sleep(20);
        }

        robot.move(64, 1, true);
        while (robot.armExtension.getCurrentPosition() < 2900) {
            sleep(20);
        }
        robot.extendArmToPosition(0);
        while (robot.armExtension.getCurrentPosition() > 1000) {
            sleep(20);
        }
        robot.move(-60, 1, true);
        robot.rotateArmToPosition(0);
        robot.setWristPosition(0.0);
        robot.strafe(-120, 1, true);


        double loopEndPosition = (robot.odo.getPosition().getHeading(AngleUnit.DEGREES));


        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;


            if (gamepad1.right_bumper) {
                x = x * 0.2;
                y = y * 0.2;
                rx = rx * 0.2;
            }

            robot.leftFront.setPower(y + x + rx);
            robot.leftBack.setPower(y - x + rx);
            robot.rightFront.setPower(y - x - rx);
            robot.rightBack.setPower(y + x - rx);

            robot.odo.update();
            Pose2D pos = robot.odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.CM), pos.getY(DistanceUnit.CM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.addData("End Loop Position", loopEndPosition);
            telemetry.addData("Path" , "Complete");
            robot.displayTargetEndPose();
            telemetry.update();
        }
    }

    }