package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.constants.DeviceIDs
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.utils.Cache

class VerticalWrist(hardwareMap: HardwareMap): SubsystemBase() {
    private var servo: ServoImplEx = hardwareMap.get(ServoImplEx::class.java, DeviceIDs.VERTICAL_WRIST)
    private var lastPosition = 0.0

    init {
        servo.pwmRange = PwmControl.PwmRange(510.0, 2490.0)
        servo.setPwmEnable()
        servo.position = VerticalConstants.VerticalWristPositions.INTAKE
    }

    fun setPosition(position: Double) {
        if (Cache.shouldUpdate(lastPosition, position, 0.005)) {
            servo.position = position
            lastPosition = position
        }
    }

}

class VerticalWristCommand(private val verticalWrist: VerticalWrist, private val position: Double): CommandBase() {
    override fun initialize() {
        verticalWrist.setPosition(position)
    }
    override fun execute() {
        super.execute()
    }
    override fun isFinished(): Boolean {
        return true
    }
    override fun end(interrupted: Boolean) {
        super.end(interrupted)
    }
}
