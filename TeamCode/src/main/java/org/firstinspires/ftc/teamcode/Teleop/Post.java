package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Library.MyOpMode;

@TeleOp(name="PSTeleop", group="Linear Opmode")
public class Post extends MyOpMode {
    // Declare OpMode members.
    double gamepadLeftY; //Forwards/Backwards Movement
    double gamepadLeftX; //Strafing Movement
    double gamepadRightX; //Turning Movement
    double gamepadRightY;

    double AngleLStick; //Left Stick Movement Control
    double AngleRStick; //Right Stick Movement Control

    double strafeSpeed = 1;
    double strafeMod = .25;
    double turnSpeed = .5; //Speed Manipulated by buttons.
    double mWS = 0;

    boolean slow = false;


    double liner = 0;
    double straf = 0;
    double turno = 0;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        hMapT(hardwareMap);
        jewelArm.setPosition(.65);
        jewelHand.setPosition(.4);
        manipWall.setPosition(.75);

        ElapsedTime delay = new ElapsedTime();
        delay.reset();

        ElapsedTime relicDelay = new ElapsedTime();
        relicDelay.reset();

        align = false;
        xDelay.reset();

        resetStartTime();
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.update();

            if (liner != 0 || straf != 0 || turno != 0) {
                setMotorsAll(liner, straf, turno);
            }

            if (!slow) {
                gamepadLeftY = gamepad1.left_stick_y;
                gamepadLeftX = gamepad1.left_stick_x;
                gamepadRightX = gamepad1.right_stick_x;
                gamepadRightY = gamepad1.right_stick_y;
                turnSpeed = .5;
            } else {
                gamepadLeftY = gamepad1.left_stick_y * .35;
                gamepadLeftX = gamepad1.left_stick_x * .35;
                gamepadRightX = gamepad1.right_stick_x * .35;
                turnSpeed = .15;
            }

            AngleLStick = Math.toDegrees(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x));
            if (AngleLStick < 0) {
                AngleLStick += 360;
            }
            AngleRStick = Math.toDegrees(Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x));
            if (AngleRStick < 0) {
                AngleRStick += 360;
            }
            telemetry.addData("align", align);
            telemetry.addData("slow", slow);
            telemetry.addData("Power Test", gamepadRightX);
            telemetry.update();

            /**Movement (Gamepad 1: Left Stick, Right Stick, DPAD, b) */
            if (45 <= AngleLStick && AngleLStick <= 135 || 225 <= AngleLStick && AngleLStick <= 315) { //DRIVING FORWARDS/BACKWARDS
//                motorFL.setPower(gamepadLeftY);
//                motorBL.setPower(gamepadLeftY);
//                motorFR.setPower(-gamepadLeftY);
//                motorBR.setPower(-gamepadLeftY);
                liner = -1*gamepadLeftY;
                //setMotors(gamepadLeftY, gamepadLeftY);
            } else {
                liner = 0;
            }
            if (315 < AngleLStick && AngleLStick < 360 || 0 <= AngleLStick && AngleLStick < 45 || 135 < AngleLStick && AngleLStick < 225) { //STRAFING LEFT/RIGHT
                if (0 < gamepadLeftX && gamepadLeftX <= .2) { //Make sure the motors don't run at too low a speed.
//                    gamepadLeftX = .25;
                    straf = .25;
                } else if (-.2 < gamepadLeftX && gamepadLeftX < 0) { //Make sure the motors don't run at too low a speed.
//                    gamepadLeftX = -.25;
                    straf = -.25;
                }
//                setMotorStrafe(gamepadLeftX);
                straf = gamepadLeftX;
            }
            else {
                straf = 0;
            }
            if (275 < AngleRStick && AngleRStick < 360 || 0 <= AngleRStick && AngleRStick < 85) { //TURNING RIGHT
//                setMotors(Math.abs(gamepadRightX), -Math.abs(gamepadRightX));
                  turno = gamepadRightX;
        }
            else if (95 <= AngleRStick && AngleRStick <= 265) { //TURNING LEFT
//                setMotors(-Math.abs(gamepadRightX), Math.abs(gamepadRightX));
                turno = gamepadRightX;
            }
            else {
                turno = 0;
            }

             if (gamepad1.left_bumper) { //Moves forwards/backwards slowly
                motorFL.setPower(-.25);
                motorBL.setPower(-.25);
                motorFR.setPower(.25);
                motorBR.setPower(.25);
            } else if (gamepad1.right_bumper) {
                motorFL.setPower(.25);
                motorBL.setPower(.25);
                motorFR.setPower(-.25);
                motorBR.setPower(-.25);
            } else if (gamepad1.x && xDelay.time() > .5) { //Lines up with the crypto-box for placement of glyph.
                if (!align) {
                    align = true;
                    rangeMovePID(7, rangeF);
                }
            }

            //Increasing/decreasing strafing power
            if (gamepad1.dpad_up && strafeSpeed <= .75) {
                strafeSpeed += strafeMod;
                sleep(250); //To be effective would need a delay.
                telemetry.addData("Strafe Speed:", strafeSpeed);
                telemetry.update();
            } else if (gamepad1.dpad_down && strafeSpeed >= 0.25) {
                strafeSpeed -= strafeMod;
                sleep(250);
                telemetry.addData("Strafe Speed:", strafeSpeed);
                telemetry.update();
            }

            /**End of Movement
             * Movement Modifiers (Gamepad 1: a,b,x,y)
             * Toggles to switch between slow mode for strafe and normal drive */
            if (delay.time() > .5) {
                if (gamepad1.a && !slow) {
                    delay.reset();
                    resetStartTime();
                    strafeSpeed = .25;
                    slow = true;
                } else if (gamepad1.a && slow) {
                    delay.reset();
                    resetStartTime();
                    strafeSpeed = 1;
                    slow = false;
                } else if (gamepad1.y && slow) {
                    delay.reset();
                    resetStartTime();
                    strafeSpeed = 1;
                    slow = false;
                }
            }
              //End of Movement Modifiers

            /**Manipulator (Gamepad 1,2: Right Trigger, Left Trigger) */
            if ((gamepad1.left_trigger > .15) || (gamepad2.left_trigger > .15)) {
                manip.setPower(-1);
            } else if ((gamepad1.right_trigger > .15) || (gamepad2.right_trigger > .15)) {
                manip.setPower(1);
            } else {
                manip.setPower(0);
            }

            /**Manipulator Wall (Gamepad 2: x,y) */
            //Deploy position for manipulator wall
            if (gamepad2.x){
                manipWall.setPosition(.035);
            }
            //Retract position for manipulator wall (init position)
            if (gamepad2.y){
                manipWall.setPosition(.75);
            }

            /**Lift Movement (Gamepad 2: Left Stick) */
            // Set's lift motor power to driver 2's left stick value
            if ((Math.abs(gamepad2.left_stick_y) > .05)) {
                    liftLeft.setPower(-gamepad2.left_stick_y * .5);
                    liftRight.setPower(gamepad2.left_stick_y * .5);
            }   else {
                    liftLeft.setPower(0);
                    liftRight.setPower(0);
                    liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            /**Relic (Gamepad 2: Right Bumper, Left Bumper, Right Stick, a, b) */
            //Open position for relic hand(with 3D printed part)
            if (gamepad2.left_bumper) {
                relicHand.setPosition(.4);
            }
            //Position for collecting the relic using the hand (with 3D print part)
            if (gamepad2.right_bumper) {
                relicHand.setPosition(0);
            }

            //Position for moving the relic wrist up or down
            if (relicDelay.milliseconds() > 75) {
                if (gamepad2.b) {
                    mWS += .05;
                    if (mWS > 1) {
                        mWS = 1;
                    }
                    relicFlip.setPosition(mWS);
                    relicDelay.reset();
                } else if (gamepad2.a) {
                    mWS -= .05;
                    if (mWS <= 0) {
                        mWS = 0;
                    }
                    relicFlip.setPosition(mWS);
                    relicDelay.reset();
                }
            }

            //When stick is pushed up the relic arm extends out
            //When stick is pushed down, the relic arm comes back in
            if ((Math.abs(gamepad2.right_stick_y) > .05)) {
                relicDrive.setPower(gamepad2.right_stick_y);

            }   else {
                relicDrive.setPower(0);
            }
         }
        }
    }






