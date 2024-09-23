package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap

class AbsoluteAnalogEncoder {
    private var encoder: AnalogInput
    private var offset = 0.0
    private val range = 3.3
    private var reverse = 0

    /**
     * @param offset subtracts radian value from final value
     */
    constructor(hardwareMap: HardwareMap, name: String, offset: Double) {
        this.encoder = hardwareMap.get(AnalogInput::class.java, name)
        this.offset = offset
    }

    constructor(hardwareMap: HardwareMap, name: String, offset: Double, reverse: Boolean) {
        this.encoder = hardwareMap.get(AnalogInput::class.java, name)
        this.offset = offset
        this.reverse = if(reverse) { 1 } else { 0 }
    }

    constructor(encoder: AnalogInput) {
        this.encoder = encoder
    }


    fun getHeading(): Double {
        return ( (Math.PI*2*reverse) - ((((getVoltage()) / range) * Math.PI*2) - offset)).mod(2*Math.PI)
    }

    fun getVoltage(): Double {
        return encoder.voltage
    }

}