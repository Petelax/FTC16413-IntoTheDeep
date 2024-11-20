package org.firstinspires.ftc.teamcode.commands.ftclib.subsystems

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalExtension
import java.util.function.DoubleSupplier

class HorizontalExtensionCommand(private var elevator: HorizontalExtension, private val speed: DoubleSupplier): CommandBase() {
    init {
        addRequirements(elevator)
    }
    override fun initialize() {

    }

    override fun execute() {
        elevator.setSpeed(speed.asDouble)
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        if (!interrupted)
            elevator.setSpeed(0.0)
    }

}