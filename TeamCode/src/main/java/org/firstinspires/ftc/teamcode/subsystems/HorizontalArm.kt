package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.constants.DeviceIDs
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.utils.Cache

class HorizontalArm(hardwareMap: HardwareMap): SubsystemBase() {
    private var servo: ServoImplEx = hardwareMap.get(ServoImplEx::class.java, DeviceIDs.HORIZONTAL_ARM)
    private var lastPosition = 0.0

    init {
        servo.pwmRange = PwmControl.PwmRange(510.0, 2490.0)
        servo.setPwmEnable()
        servo.position = HorizontalConstants.HorizontalArmPositions.IN
    }

    fun setPosition(position: Double) {
        if (Cache.shouldUpdate(lastPosition, position, 0.005)) {
            servo.position = position
            lastPosition = position
        }
    }

}

class HorizontalArmCommand(private val horizontalArm: HorizontalArm, private val position: Double): CommandBase() {
    override fun initialize() {
        horizontalArm.setPosition(position)
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
