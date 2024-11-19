package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.commands.drivebase.FieldCentricDrive
import org.firstinspires.ftc.teamcode.commands.subsystems.ElevatorCommand
import org.firstinspires.ftc.teamcode.commands.subsystems.ElevatorPIDCommand
import org.firstinspires.ftc.teamcode.commands.subsystems.HorizontalExtend
import org.firstinspires.ftc.teamcode.commands.subsystems.HorizontalExtensionCommand
import org.firstinspires.ftc.teamcode.commands.subsystems.IntakeRun
import org.firstinspires.ftc.teamcode.commands.subsystems.IntakeStop
import org.firstinspires.ftc.teamcode.commands.subsystems.PlaceSample
import org.firstinspires.ftc.teamcode.commands.subsystems.VerticalRetract
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.subsystems.Deposit
import org.firstinspires.ftc.teamcode.subsystems.DepositCommand
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.HorizontalArm
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension
import org.firstinspires.ftc.teamcode.subsystems.HorizontalWrist
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.VerticalArm
import org.firstinspires.ftc.teamcode.subsystems.VerticalWrist
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain

@TeleOp
class CommandTeleOp: CommandOpMode() {
    private lateinit var hubs: List<LynxModule>
    private lateinit var drive: SwerveDrivetrain
    private lateinit var elevator: Elevator
    private lateinit var horizontalExtension: HorizontalExtension
    private lateinit var horizontalArm: HorizontalArm
    private lateinit var horizontalWrist: HorizontalWrist
    private lateinit var intake: Intake
    private lateinit var verticalArm: VerticalArm
    private lateinit var verticalWrist: VerticalWrist
    private lateinit var deposit: Deposit
    private lateinit var dashboard: FtcDashboard
    private var lastTime = System.nanoTime()
    private lateinit var driveOp: GamepadEx
    private lateinit var toolOp: GamepadEx
    private lateinit var voltageSensor: VoltageSensor
    private var runIntake: Boolean = false

    override fun initialize() {
        hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        //voltageSensor = hardwareMap.get(VoltageSensor::class.java, "voltageSensor")
        drive = SwerveDrivetrain(hardwareMap, 12.0)

        elevator = Elevator(hardwareMap)
        verticalArm = VerticalArm(hardwareMap)
        verticalWrist = VerticalWrist(hardwareMap)
        deposit = Deposit(hardwareMap)

        horizontalExtension = HorizontalExtension(hardwareMap)
        horizontalArm = HorizontalArm(hardwareMap)
        horizontalWrist = HorizontalWrist(hardwareMap)
        intake = Intake(hardwareMap)


        dashboard = FtcDashboard.getInstance()
        driveOp = GamepadEx(gamepad1)
        toolOp = GamepadEx(gamepad2)


        val packet = TelemetryPacket()
        packet.addLine("init")
        dashboard.sendTelemetryPacket(packet)

        drive.defaultCommand = FieldCentricDrive(
                drive,
                { driveOp.leftX },
                { driveOp.leftY },
                { driveOp.rightX },
                { false },
                { true }
        )

        elevator.defaultCommand = ElevatorCommand(elevator) { toolOp.leftY }

        horizontalExtension.defaultCommand = HorizontalExtensionCommand(horizontalExtension) { -toolOp.rightY }

    }

    override fun run() {
        val currentTime = System.nanoTime()

        for (hub in hubs) {
            hub.clearBulkCache()
        }

        //val pose = drive.getPose()

        val packet = TelemetryPacket()
        /*
        packet.put("x", pose.x)
        packet.put("y", pose.y)
        packet.put("heading", pose.heading)
        packet.put("elevator height", elevator.getPosition())

        packet.fieldOverlay().setStroke("#3F51B5")
        Drawing.drawRobot(packet.fieldOverlay(), pose)

         */


        GamepadButton(toolOp, GamepadKeys.Button.A).whenPressed(IntakeRun(intake))
        GamepadButton(toolOp, GamepadKeys.Button.B).whenPressed(IntakeStop(intake))
        GamepadButton(toolOp, GamepadKeys.Button.Y).whenPressed(PlaceSample(horizontalExtension, horizontalArm, horizontalWrist, intake, elevator, verticalArm, verticalWrist, deposit))
        GamepadButton(toolOp, GamepadKeys.Button.X).whenPressed(HorizontalExtend(horizontalExtension, horizontalArm, horizontalWrist))
        GamepadButton(toolOp, GamepadKeys.Button.LEFT_BUMPER).whenPressed(DepositCommand(deposit, VerticalConstants.DepositPositions.OUT))
        GamepadButton(toolOp, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(DepositCommand(deposit, VerticalConstants.DepositPositions.IN))

        packet.put("game piece", intake.getGamePiece())
        packet.put("distance", intake.getDistance())

        GamepadButton(toolOp, GamepadKeys.Button.DPAD_DOWN).whenPressed(VerticalRetract(elevator, verticalArm, verticalWrist, deposit))
        GamepadButton(toolOp, GamepadKeys.Button.DPAD_RIGHT).whenPressed(ElevatorPIDCommand(elevator, 12.0))
        GamepadButton(toolOp, GamepadKeys.Button.DPAD_UP).whenPressed(ElevatorPIDCommand(elevator, VerticalConstants.ElevatorPositions.TOP))


        super.run()

        packet.put("ms", (currentTime-lastTime)/1E6)
        packet.put("ms pose", (currentTime-lastTime)/1E6)
        dashboard.sendTelemetryPacket(packet)

        lastTime = currentTime
    }
}