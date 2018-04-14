package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Library.MyOpMode;

@Autonomous(name = "AutoBlueRight", group = "Linear Opmode")
public class AutoBlueRight extends MyOpMode {
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
        align = true;
        // Set up our telemetry dashboard
//        composeTelemetry();

        // Wait until we're told to go
        waitForStart();
        runtime.reset();
/**---------------------------------------------------------------------------------------------------------------*/
        vfValue();
        jewelKnockerBlue();

        setMotorsAll(-.4, 0, 0);
        sleep(1000);
        stopMotors();
        sleep(100);

       setTurnAuto(-85);
        sleep(1000);

        setMotorsAll(0,-.4,0);
        sleep(800);
        stopMotors();

        setMotorsAll(0,.4,0);
        sleep(825);
        stopMotors();

        vfMoveAlt();

        rangeMovePID(6, rangeF);

        manipAuto(-.75);
        sleep(500);
        manipAuto(-.75);

//back up, push forward, back up
        setMotorsAll(-.2,0,0);
        sleep(250);
        stopMotors();

        manip.setPower(-1);
        setMotorsAll(.3,0,0);
        sleep(250);
        stopMotors();

        setMotorsAll(-.2,0,0);
        sleep(250);
        stopMotors();
        manip.setPower(0);
        }
    }