#include <gpio.h>

gpio::gpio()
{
    adcTimer.init();                // Initialize the ADC conversion timer
    adcTimer.set(ADC_CONVERT_TIME); // Set the timer interval for ADC conversion
    adcTimer.start();               // Start the timer
}

void gpio::init()
{
    pinMode(DEBUG_LED, OUTPUT);

    pinMode(EN_BOOST, OUTPUT);
    pinMode(EN_OUTPUT, OUTPUT);
    pinMode(PG_BOOST, INPUT);

    pinMode(SENS_ENABLE, OUTPUT);

    pinMode(BAT1_CHGR_ENABLE, OUTPUT);
    pinMode(BAT1_CHGR_STATUS, INPUT);
    pinMode(BAT1_CHGR_VOLTAGE_SWITCH, OUTPUT);
    pinMode(BAT2_CHGR_ENABLE, OUTPUT);
    pinMode(BAT2_CHGR_STATUS, INPUT);
    pinMode(BAT2_CHGR_VOLTAGE_SWITCH, OUTPUT);

    pinMode(LIGHT_BAR1_ENABLE, OUTPUT);
    pinMode(LIGHT_BAR2_ENABLE, OUTPUT);
    pinMode(LIGHT_BAR1, INPUT);
    pinMode(LIGHT_BAR2, INPUT);

    pinMode(RF_IN1, INPUT_PULLUP);
    pinMode(RF_IN2, INPUT_PULLUP);

    pinMode(KEY_SWITCH_1, INPUT);
    pinMode(KEY_SWITCH_2, INPUT);

    pinMode(FRONT_PANEL_BUTTON, INPUT_PULLUP);
    pinMode(BUZZER, OUTPUT);

    pinMode(DIP_SWITCH_1, INPUT_PULLUP);
    pinMode(DIP_SWITCH_2, INPUT_PULLUP);
    pinMode(DIP_SWITCH_3, INPUT_PULLUP);

    pinMode(WISCH1_PIN, OUTPUT);
    pinMode(WISCH2_PIN, OUTPUT);

    pinMode(MOTOR_OPTO_1, INPUT);
    pinMode(MOTOR_OPTO_2, INPUT);

    pinMode(EXPANSION_GPIO, INPUT);

    digitalWrite(EN_BOOST, LOW);
    digitalWrite(EN_OUTPUT, LOW);

    digitalWrite(SENS_ENABLE, LOW);

    digitalWrite(BAT1_CHGR_ENABLE, HIGH);
    digitalWrite(BAT2_CHGR_ENABLE, HIGH);
    digitalWrite(BAT1_CHGR_VOLTAGE_SWITCH, LOW);
    digitalWrite(BAT2_CHGR_VOLTAGE_SWITCH, LOW);

    digitalWrite(LIGHT_BAR1_ENABLE, LOW);
    digitalWrite(LIGHT_BAR2_ENABLE, LOW);

    digitalWrite(BUZZER, LOW);

    digitalWrite(WISCH1_PIN, LOW);
    digitalWrite(WISCH2_PIN, LOW);

    digitalWrite(DEBUG_LED, LOW);
}

void gpio::handler()
{
    if (adcTimer.repeat())
    {
        convertADCValues();
    }
    debounceInputs();
    if (wisch1Timer.done())
    {
        digitalWrite(WISCH1_PIN, LOW); // Turn off Wisch 1 after the timer is done
        wisch1Timer.reset();           // Reset the timer for Wisch 1
        wisch1Timer.stop();            // Stop the timer for Wisch 1
    }
    if (wisch2Timer.done())
    {
        digitalWrite(WISCH2_PIN, LOW); // Turn off Wisch 2 after the timer is done
        wisch2Timer.reset();           // Reset the timer for Wisch 2
        wisch2Timer.stop();            // Stop the timer for Wisch 2
    }
}

void gpio::convertADCValues()
{
    int vref_int_voltage = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(adc_values[5], ADC_RESOLUTION_12B);                                // Calculate the internal reference voltage
    solar_voltage = __HAL_ADC_CALC_DATA_TO_VOLTAGE(vref_int_voltage, adc_values[1], ADC_RESOLUTION_12B) * 17.54385964912281;    // Convert to voltage
    battery1_voltage = __HAL_ADC_CALC_DATA_TO_VOLTAGE(vref_int_voltage, adc_values[2], ADC_RESOLUTION_12B) * 17.54385964912281; // Convert to voltage
    battery2_voltage = __HAL_ADC_CALC_DATA_TO_VOLTAGE(vref_int_voltage, adc_values[3], ADC_RESOLUTION_12B) * 17.54385964912281; // Convert to voltage
    output_current = __HAL_ADC_CALC_DATA_TO_VOLTAGE(vref_int_voltage, adc_values[4], ADC_RESOLUTION_12B);                       // Convert to current
    int poti_time_temp = map(adc_values[0], 0, 4095, 180000, 5000);                                                             // Map the potentiometer value to a time range
    // only update if the value has changed by at least 1000 ms
    if (abs(poti_time_temp - poti_time) >= 1000)
    {
        poti_time = poti_time_temp; // Update the potentiometer time
    }
}

void gpio::debounceInputs()
{
    // Debounce key switches
    debounced_key_switch_1 = debounce(!digitalRead(KEY_SWITCH_1));
    debounced_key_switch_2 = debounce(!digitalRead(KEY_SWITCH_2));

    // Debounce light bars
    debounced_light_bar1 = debounce(!digitalRead(LIGHT_BAR1));
    debounced_light_bar2 = debounce(!digitalRead(LIGHT_BAR2));

    // Debounce front panel button
    debounced_front_panel_button = debounce(!digitalRead(FRONT_PANEL_BUTTON));

    // Debounce DIP switches
    debounced_dip_switch_1 = debounce(!digitalRead(DIP_SWITCH_1));
    debounced_dip_switch_2 = debounce(!digitalRead(DIP_SWITCH_2));
    debounced_dip_switch_3 = debounce(!digitalRead(DIP_SWITCH_3));
}

bool gpio::debounce(bool signal)
{
    // raw_input is the last reading we saw
    // debounced    is the last *stable* output
    static bool raw_input = false;
    static bool debounced = false;
    static Neotimer timer(50); // 50 ms debounce window

    // 1) on any change of the raw reading, restart the timer
    if (signal != raw_input)
    {
        raw_input = signal;
        timer.restart();
    }

    // 2) once the timer finishes, accept the new value as stable
    if (timer.done())
    {
        debounced = raw_input;
    }

    // 3) always return the stable (debounced) state
    return debounced;
}

void gpio::triggerWisch1()
{
    if (wisch1Timer.started() == false)
    {
        wisch1Timer.reset();
        wisch1Timer.start();
        digitalWrite(WISCH1_PIN, HIGH);
    }
}
void gpio::triggerWisch2()
{
    if (wisch2Timer.started() == false)
    {
        wisch2Timer.reset();
        wisch2Timer.start();
        digitalWrite(WISCH2_PIN, HIGH);
    }
}

int gpio::getMotorDirection()
{
    if (digitalRead(MOTOR_OPTO_1) == HIGH && digitalRead(MOTOR_OPTO_2) == HIGH)
    {
        return 0; // Both sensors are HIGH, motor is stopped
    }
    else if (digitalRead(MOTOR_OPTO_1) == LOW)
    {
        return 1; // Motor is moving downwards
    }
    else if (digitalRead(MOTOR_OPTO_2) == LOW)
    {
        return 2; // Motor is moving upwards
    }
    return 0; // Default case
}