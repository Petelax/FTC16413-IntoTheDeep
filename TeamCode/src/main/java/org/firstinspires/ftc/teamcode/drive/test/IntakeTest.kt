package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.ftclib.Intake

@TeleOp(group = "test")
class IntakeTest: OpMode() {
    private lateinit var hubs: List<LynxModule>
    private lateinit var elapsedtime: ElapsedTime
    private lateinit var gamepad: GamepadEx
    private lateinit var intake: Intake
    private var requested = 0.0

    override fun init() {
        elapsedtime = ElapsedTime()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        // this just sets the bulk reading mode for each hub
        hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        //voltage = hardwareMap.getAll(PhotonLynxVoltageSensor::class.java).iterator().next()

        gamepad = GamepadEx(gamepad1)

        intake = Intake(hardwareMap)

        elapsedtime.reset()
    }

    override fun loop() {
        for (hub in hubs) {
            hub.clearBulkCache()
        }

        if(gamepad1.a) {
            requested = 1.0
        } else if(gamepad1.b) {
            requested = 0.0
        } else if(gamepad1.x) {
            requested = 0.5
        }
        intake.setSpeed(requested)

        telemetry.addData("last", intake.getSpeed())
        telemetry.addData("current", requested)

        telemetry.update()
    }

}

