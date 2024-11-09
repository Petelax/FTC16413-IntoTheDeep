package org.firstinspires.ftc.teamcode.commands.subsystems

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.controller.PIDFController
import org.firstinspires.ftc.teamcode.constants.VerticalConstants.*
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import java.util.function.DoubleSupplier

class ElevatorCommand(private var elevator: Elevator, private val speed: DoubleSupplier): CommandBase() {
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