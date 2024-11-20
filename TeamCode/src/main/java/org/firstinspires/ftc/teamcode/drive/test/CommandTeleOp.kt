package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.commands.ftclib.drivebase.FieldCentricDrive
import org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.ElevatorCommand
import org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.HorizontalExtensionCommand
import org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.IntakeArmExtend
import org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.IntakeArmRetract
import org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.PlaceSample
import org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.VerticalRetract
import org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.VerticalSample
import org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.VerticalSpecimenPickup
import org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.VerticalSpecimenPlace
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.subsystems.ftclib.Deposit
import org.firstinspires.ftc.teamcode.subsystems.ftclib.DepositCommand
import org.firstinspires.ftc.teamcode.subsystems.ftclib.Elevator
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalArm
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalExtension
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalWrist
import org.firstinspires.ftc.teamcode.subsystems.ftclib.Intake
import org.firstinspires.ftc.teamcode.subsystems.ftclib.VerticalArm
import org.firstinspires.ftc.teamcode.subsystems.ftclib.VerticalWrist
import org.firstinspires.ftc.teamcode.subsystems.ftclib.swerve.SwerveDrivetrain

@TeleOp(group = "test")
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

        drive.defaultCommand =
            org.firstinspires.ftc.teamcode.commands.ftclib.drivebase.FieldCentricDrive(
                drive,
                { driveOp.leftX },
                { driveOp.leftY },
                { driveOp.rightX },
                { false },
                { true }
            )

        elevator.defaultCommand =
            org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.ElevatorCommand(elevator) { toolOp.leftY }

        horizontalExtension.defaultCommand =
            org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.HorizontalExtensionCommand(
                horizontalExtension
            ) { -toolOp.rightY }

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


        GamepadButton(toolOp, GamepadKeys.Button.Y).whenPressed(
            org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.PlaceSample(
                horizontalExtension,
                horizontalArm,
                horizontalWrist,
                intake,
                elevator,
                verticalArm,
                verticalWrist,
                deposit
            )
        )
        GamepadButton(toolOp, GamepadKeys.Button.X).whenPressed(
            org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.IntakeArmRetract(
                horizontalArm,
                horizontalWrist,
                intake
            )
        )
        GamepadButton(toolOp, GamepadKeys.Button.B).whenPressed(
            org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.IntakeArmExtend(
                horizontalArm,
                horizontalWrist,
                intake
            )
        )

        GamepadButton(toolOp, GamepadKeys.Button.LEFT_BUMPER).whenPressed(DepositCommand(deposit, VerticalConstants.DepositPositions.OUT))
        GamepadButton(toolOp, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(DepositCommand(deposit, VerticalConstants.DepositPositions.IN))

        GamepadButton(toolOp, GamepadKeys.Button.DPAD_DOWN).whenPressed(
            org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.VerticalRetract(
                elevator,
                verticalArm,
                verticalWrist,
                deposit
            )
        )
        GamepadButton(toolOp, GamepadKeys.Button.DPAD_RIGHT).whenPressed(
            org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.VerticalSpecimenPlace(
                elevator,
                verticalArm,
                verticalWrist
            )
        )
        GamepadButton(toolOp, GamepadKeys.Button.DPAD_LEFT).whenPressed(
            org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.VerticalSpecimenPickup(
                elevator,
                verticalArm,
                verticalWrist
            )
        )
        GamepadButton(toolOp, GamepadKeys.Button.DPAD_UP).whenPressed(
            org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.VerticalSample(
                elevator,
                verticalArm,
                verticalWrist
            )
        )


        super.run()

        packet.put("ms", (currentTime-lastTime)/1E6)
        packet.put("ms pose", (currentTime-lastTime)/1E6)
        dashboard.sendTelemetryPacket(packet)

        lastTime = currentTime
    }
}