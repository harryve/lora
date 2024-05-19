Sender is de sensor in de berging die via LoRa temperatuur en luchtvochtigheid opstuurt.

Receiver vormt de bridge tussen LoRa en de MQTT broker. Via WiFi dus. Waarom WifI? Omdat dat het kan.

Heltec is de 2e sensor. Deze komt op het balkon om temperatuur, luchtvochtigheid en lichtsterkte te meten. Dus het buiten klimaat als het ware. Reeds.
Op het Heltec bordje is ADC0 aangesloten via een 1/6 spanningsdeler op de 5V. Als het boardje aan de lader hangt, wordt dit gedetecteerd en de OTA server opgestart.
