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

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Library.MyOpMode;

/**
 * {@link AutoBlueRight} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@Autonomous(name = "Blue Right Gyro", group = "Sensor")
                            // Comment this out to add to the opmode list
public class AutoBlueRight extends MyOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        hMap(hardwareMap);

        // Set up our telemetry dashboard
        composeTelemetry();
        // Wait until we're told to go

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
        telemetry.addData("Column ", column);

        ElapsedTime time = new ElapsedTime();
        time.reset();
        resetStartTime();

        waitForStart();
        runtime.reset();
/**---------------------------------------------------------------------------------------------------------------*/
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


            liftLeft.setPower(.5);
            liftRight.setPower(-.5);
            sleep(250);
            liftLeft.setPower(0);
            liftRight.setPower(0);


            jewelArm.setPosition(.55);
            jewelHand.setPosition(.45);
            sleep(500);
            jewelArm.setPosition(.15);
            sleep(1000);
            if (jewelColor.red() < jewelColor.blue()) {
                jewelHand.setPosition((.3));
            } else if (jewelColor.red() > jewelColor.blue()) {
                jewelHand.setPosition((.6));
            }
            sleep(500);

            jewelArm.setPosition(.55);
            jewelHand.setPosition(.45);
            sleep(500);
            jewelHand.setPosition(.3);
            sleep(500);

            setMotors(-.25, -.25);
            sleep(1850);
            stopMotors();


            try {
                turnCorr(.1, -70, 4000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            sleep(1000);


        vfMovePerp('b', rangeL);


        rangeMovePID(6.25, rangeF);
        sleep(500);


        manipAuto(-.75);
        sleep(500);

        manipAuto(-.75);

//back up, push forward, back up
        setMotors(-.2, -.2);
        sleep(250);
        stopMotors();

        setMotors(.3, .3);
        sleep(250);
        stopMotors();

        manipAuto(-.75);
        sleep(200);

        setMotors(-.2, -.2);
        sleep(250);
        stopMotors();




//        vfMovePerp( 'r', rangeR);

//        rangeMoveStrafe(42.5, rangeR);
//        sleep(750);


            //Finish optimizing this Auto, then invert for the blue side.
            // Loop and update the dashboard
//        while (opModeIsActive()) {
//
//            telemetry.update();
//        }
        }
    }

