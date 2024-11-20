package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain

@TeleOp(group = "test")
class ModuleServoTest: OpMode() {
    private lateinit var hubs: List<LynxModule>
    private lateinit var elapsedtime: ElapsedTime
    private lateinit var drive: SwerveDrivetrain
    private lateinit var gamepad: GamepadEx
    var module = Module.LF

    override fun init() {
        elapsedtime = ElapsedTime()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        // this just sets the bulk reading mode for each hub
        hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        drive = SwerveDrivetrain(hardwareMap)
        gamepad = GamepadEx(gamepad1)

        elapsedtime.reset()
    }

    override fun loop() {
        for (hub in hubs) {
            hub.clearBulkCache()
        }

        if (gamepad.isDown(GamepadKeys.Button.X)) {
            module = Module.LF
        } else if (gamepad.isDown(GamepadKeys.Button.Y)) {
            module = Module.RF
        } else if (gamepad.isDown(GamepadKeys.Button.A)) {
            module = Module.LR
        } else if (gamepad.isDown(GamepadKeys.Button.B)) {
            module = Module.RR
        }

        val index = when(module) {
            Module.LF -> {telemetry.addLine("LF"); 0}
            Module.RF -> {telemetry.addLine("RF"); 1}
            Module.LR -> {telemetry.addLine("LR"); 2}
            Module.RR -> {telemetry.addLine("RR"); 3}
        }

        drive.testModule(index, gamepad.leftY, gamepad.rightX)

        val headings = drive.getModuleHeadings()

        telemetry.addData("module heading", headings[index])
        //telemetry.addData("rf heading", headings[1])
        //telemetry.addData("lr heading", headings[2])
        //telemetry.addData("rr heading", headings[3])

        telemetry.addData("gyro heading", drive.getHeading())

    }

    enum class Module {
        LF,
        RF,
        LR,
        RR
    }
}