Berging is de sensor in de berging die via LoRa temperatuur en luchtvochtigheid opstuurt.

Bridge vormt de bridge tussen LoRa en de MQTT broker. Via WiFi dus. Waarom WifI? Omdat dat het kan.

Balkon is de 2e sensor. Deze komt op het balkon (waar anders?) om temperatuur, luchtvochtigheid en lichtsterkte te meten. Dus het buiten klimaat als het ware. Reeds.
De balkon sesnor is een Heltec boardje waar ADC0 aangesloten is via een 1/6 spanningsdeler op de 5V. Als het boardje aan de lader hangt, wordt dit gedetecteerd en de OTA server opgestart.
