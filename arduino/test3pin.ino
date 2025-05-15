void setup()
{
    Serial.begin(9600);
    pinMode(13, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(11, OUTPUT);
}

void loop()
{
    if (Serial.available())
    {
        String input = Serial.readStringUntil('\n');
        input.trim(); // Clean up any newline or spaces

        int pinIndex = input.indexOf(":");
        if (pinIndex > 0)
        {
            String pinStr = input.substring(0, pinIndex); // e.g., "PIN13"
            String state = input.substring(pinIndex + 1); // e.g., "HIGH"

            // Extract pin number
            if (pinStr.startsWith("PIN"))
            {
                int pin = pinStr.substring(3).toInt(); // Get number after "PIN"
                if (state == "HIGH")
                {
                    digitalWrite(pin, HIGH);
                }
                else if (state == "LOW")
                {
                    digitalWrite(pin, LOW);
                }
            }
        }
    }
}
