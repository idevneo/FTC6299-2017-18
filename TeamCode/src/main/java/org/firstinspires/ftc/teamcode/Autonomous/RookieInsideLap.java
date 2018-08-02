package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Library.MyOpMode;

@Autonomous(name = "RookieInsideLap", group = "Linear Opmode")
public class RookieInsideLap extends MyOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        hMapRookie(hardwareMap);
        motorML.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorMR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
            telemetry.addLine("Initializing IMU...");
            telemetry.update();
        }

        // Set up our telemetry dashboard
//        composeTelemetry();

        // Wait until we're told to go
        waitForStart();
        runtime.reset();
/**---------------------------------------------------------------------------------------------------------------*/

        goStraight(0, 1, 1250);
        arcTurn(.8, 1);
        goStraight(1,1,2000);
        arcTurn(.8,2);
        goStraight(2,1,2000);
        arcTurn(.8, 3);
        goStraight(3,1,2000);
        arcTurn(.8, 4);
        //TODO lower lever and reverse into parking
        //lowerStick()
        goStraight(3,-1,2000);
        stopMotors();
        //auto portion ends
        //Controller picks up

        }

        public void goStraight (int turnNum, double power, double duration){
            double yaw = getGyroYaw();
            double currTime = runtime.milliseconds();
            double propP = 0;
            double error = 0;
            double l = power;
            double r = power;
            double target = (turnNum > 2) ? 90*(turnNum-2): -90*turnNum;
            while (currTime < duration){
                yaw = getGyroYaw();
                error = Math.abs((-90*turnNum)-getGyroYaw());
                power = power - (error/ Math.abs(target));
                l = (yaw > target) ? l - (error / Math.abs(target)): l;
                r = (yaw < target) ? r - (error / Math.abs(target)): r;
                setMotors(l,r);
            }
            stopMotors();


        }

        public void arcTurn(double power, int turnNum) {
            double yaw = getGyroYaw();
            double error = 0;
            double target = (turnNum > 2) ? 90*(turnNum-2): -90*turnNum;
            while (yaw > target) {
                yaw = getGyroYaw();
                error = Math.abs((-90 * turnNum) - getGyroYaw());
                power = power - (error / Math.abs(target) +.1);
                //power = (error * .5) + .1;
                setMotors(power - .2, power);
            }
            stopMotors();
        }



    }