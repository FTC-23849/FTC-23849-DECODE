package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

/**
 * Minimal goBILDA Prism RGB LED Driver (I2C) driver.
 *
 * Uses Prism register map from goBILDA user guide:
 * - I2C address: 0x38 :contentReference[oaicite:4]{index=4}
 * - Layers at registers 0x08..0x11 with sub-registers (selected animation, brightness, start/stop, color) :contentReference[oaicite:5]{index=5}
 * - Solid Color animation ID = 0x01 :contentReference[oaicite:6]{index=6}
 */
@I2cDeviceType
@DeviceProperties(
        name = "Custom goBILDA® Prism RGB LED Driver",
        description = "Prism RGB LED Driver (I2C)",
        xmlTag = "gobildaPrismRgbLedDriver"
)
public class CustomGoBildaPrismRgbLedDriver extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public static final I2cAddr DEFAULT_I2C_ADDR = I2cAddr.create7bit(0x38); // :contentReference[oaicite:7]{index=7}

    // Layer registers: 0x08 = layer0, 0x09 = layer1, ...
    private static final int REG_LAYER_BASE = 0x08;

    // Common sub-registers for animations on a layer :contentReference[oaicite:8]{index=8}
    private static final int SUB_SELECTED_ANIMATION = 0x00;
    private static final int SUB_BRIGHTNESS        = 0x01; // 0..100
    private static final int SUB_START_INDEX       = 0x02; // pixel index
    private static final int SUB_STOP_INDEX        = 0x03; // pixel index
    private static final int SUB_PRIMARY_COLOR     = 0x04; // R,G,B bytes

    // Animation IDs
    private static final int ANIM_SOLID_COLOR = 0x01; // :contentReference[oaicite:9]{index=9}

    public CustomGoBildaPrismRgbLedDriver(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
        this.deviceClient.setI2cAddress(DEFAULT_I2C_ADDR);
        super.registerArmingStateCallback(true);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        // Nothing required here for basic operation.
        // We program layers directly.
        return true;
    }

    @NonNull
    @Override
    public Manufacturer getManufacturer() {
        return HardwareDevice.Manufacturer.Other;
    }

    @NonNull
    @Override
    public String getDeviceName() {
        return "goBILDA Prism RGB LED Driver";
    }

    /** Configure a layer as a solid-color segment. */
    public synchronized void configureSolidLayer(int layer, int startIndex, int stopIndex, int brightness0to100,
                                                 int r0to255, int g0to255, int b0to255) {
        layer = clamp(layer, 0, 9);
        writeLayer(layer, SUB_SELECTED_ANIMATION, (byte) ANIM_SOLID_COLOR);
        setLayerBrightness(layer, brightness0to100);
        setLayerRange(layer, startIndex, stopIndex);
        setLayerColor(layer, r0to255, g0to255, b0to255);
    }

    /** Set only the color (fast update). */
    public synchronized void setLayerColor(int layer, int r0to255, int g0to255, int b0to255) {
        layer = clamp(layer, 0, 9);
        byte r = (byte) clamp(r0to255, 0, 255);
        byte g = (byte) clamp(g0to255, 0, 255);
        byte b = (byte) clamp(b0to255, 0, 255);
        writeLayer(layer, SUB_PRIMARY_COLOR, r, g, b);
    }

    /** Set brightness 0..100. */
    public synchronized void setLayerBrightness(int layer, int brightness0to100) {
        layer = clamp(layer, 0, 9);
        byte br = (byte) clamp(brightness0to100, 0, 100);
        writeLayer(layer, SUB_BRIGHTNESS, br);
    }

    /** Set start/stop indices for the layer segment. */
    public synchronized void setLayerRange(int layer, int startIndex, int stopIndex) {
        layer = clamp(layer, 0, 9);
        byte start = (byte) clamp(startIndex, 0, 255);
        byte stop  = (byte) clamp(stopIndex, 0, 255);
        writeLayer(layer, SUB_START_INDEX, start);
        writeLayer(layer, SUB_STOP_INDEX, stop);
    }

    /** Helper: write to a layer register with a sub-register + payload. */
    private void writeLayer(int layer, int subReg, byte... payload) {
        int reg = REG_LAYER_BASE + layer;

        // Prism expects: [subReg, data...]
        byte[] data = new byte[payload.length + 1];
        data[0] = (byte) (subReg & 0xFF);
        System.arraycopy(payload, 0, data, 1, payload.length);

        deviceClient.write(reg, data);
    }

    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
