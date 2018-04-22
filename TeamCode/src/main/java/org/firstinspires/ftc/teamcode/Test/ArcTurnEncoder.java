package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Library.MyOpMode;

/**
 * Created by applmgr on 4/14/18.
 */



@Autonomous(name="ArcTurnEncoder", group="Test")
public class ArcTurnEncoder extends MyOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        hMap(hardwareMap);
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
        // run until the end of the match (driver presses STOP)
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        try {
            arcTurnEncoder(0.5,-90,5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }
}
