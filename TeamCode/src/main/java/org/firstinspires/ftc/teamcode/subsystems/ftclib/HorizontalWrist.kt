package org.firstinspires.ftc.teamcode.subsystems.ftclib

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.constants.DeviceIDs
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.utils.Cache

class HorizontalWrist(hardwareMap: HardwareMap): SubsystemBase() {
    private var servo: ServoImplEx = hardwareMap.get(ServoImplEx::class.java, DeviceIDs.HORIZONTAL_WRIST)
    private var lastPosition = -1.0

    init {
        servo.pwmRange = PwmControl.PwmRange(510.0, 2490.0)
        servo.position = HorizontalConstants.HorizontalWristPositions.IN
    }

    fun setPosition(position: Double) {
        if (Cache.shouldUpdate(lastPosition, position, 0.005)) {
            servo.position = position
            lastPosition = position
        }
    }

}
class HorizontalWristCommand(private val horizontalWrist: HorizontalWrist, private val position: Double): CommandBase() {
    override fun initialize() {
        horizontalWrist.setPosition(position)
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
