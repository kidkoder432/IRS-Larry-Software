/*
                case 'B': {
                    Serial.println("Editing config file");
                    Serial.println("Available keys: ");

                    int i = 1;
                    for (auto& entry : config) {
                        Serial.print(i);
                        Serial.print(": ");
                        Serial.println(entry.first.c_str());
                        i++;
                    }

                    bool continueEdit = true;
                    while (continueEdit) {

                        int selectedKey = 0;
                        String keyStr;
                        while (selectedKey < 1 || selectedKey > config.size()) {
                            Serial.print("Enter key to edit (q to finish): ");
                            Serial.setTimeout(1000000);
                            keyStr = Serial.readStringUntil('\n');

                            keyStr.trim();
                            keyStr.toLowerCase();
                            Serial.println(keyStr);

                            if (keyStr == "q") {
                                Serial.println("Finished editing config file");
                                continueEdit = false;
                                break;
                            }
                            selectedKey = keyStr.toInt();
                        }

                        if (!continueEdit) {
                            break;
                        }

                        i = 1;
                        std::string key = "";
                        float selectedEntry = 0.0;
                        for (auto& entry : config) {
                            if (selectedKey == i) {
                                Serial.print("Selected key: ");
                                key = entry.first;
                                Serial.println(entry.first.c_str());
                                selectedEntry = entry.second;
                                break;
                            }
                            i++;
                        }

                        Serial.print("Current key value is: ");
                        Serial.println(selectedEntry);

                        Serial.print("Enter new value (q to finish): ");
                        String valueStr = Serial.readStringUntil('\n');
                        valueStr.toLowerCase();
                        valueStr.trim();
                        Serial.println(valueStr);
                        if (valueStr == "q") {
                            Serial.println("Finished editing config file");
                            continueEdit = false;
                            break;
                        }
                        valueStr.toLowerCase();
                        float value;
                        if (valueStr == "true") {
                            value = 1.0;
                        }
                        else if (valueStr == "false") {
                            value = 0.0;
                        }
                        else {
                            value = valueStr.toDouble();
                        }

                        config[key] = value;
                    }

                    Serial.println("Done editing config file");
                    Serial.println("Would you like to save the changes? (y/n)");

                    String saveStr = Serial.readStringUntil('\n');
                    saveStr.toLowerCase();
                    saveStr.trim();
                    if (saveStr == "y") {
                        bool success = saveConfig(sd, config);
                        if (success) {
                            Serial.println("Changes saved");
                            Serial.println("Please reset the Arduino to apply changes");
                        }
                        else {
                            Serial.println("Error: Changes not saved!");
                        }

                    }
                    else {
                        Serial.println("Changes not saved");
                    }

                    break;
                } */


// Helper function to list files recursively
// void listFiles(fs::FS& fs, const char* dirname, uint8_t levels) {
//     File root = fs.open(dirname);
//     if (!root) {
//         Serial.println("Failed to open directory");
//         return;
//     }
//     if (!root.isDirectory()) {
//         Serial.println("Not a directory");
//         return;
//     }

//     File file = root.openNextFile();
//     while (file) {
//         if (file.isDirectory()) {
//             Serial.print("DIR : ");
//             Serial.println(file.name());
//             if (levels) {
//                 listFiles(fs, file.name(), levels - 1);
//             }
//         }
//         else {
//             Serial.print("FILE: ");
//             Serial.print(file.name());
//             Serial.print("\tSIZE: ");
//             Serial.println(file.size());
//         }
//         file = root.openNextFile();
//     }
// }

// void checkSDCard() {
//     // Initialize SD card
//     if (!SD.begin(10)) {
//         Serial.println("Initialization failed. Things to check:");
//         Serial.println("* Is a card inserted?");
//         Serial.println("* Is your wiring correct?");
//         Serial.println("* Did you set the correct CS pin?");
//         return;
//     }
//     else {
//         Serial.println("Wiring is correct and a card is present.");
//     }

//     // Print card type
//     Serial.println();
//     Serial.print("Card type:         ");
//     switch (SD.cardType()) {
//         case CARD_NONE:
//             Serial.println("No SD card attached");
//             return;
//         case CARD_MMC:
//             Serial.println("MMC");
//             break;
//         case CARD_SD:
//             Serial.println("SDSC");
//             break;
//         case CARD_SDHC:
//             Serial.println("SDHC");
//             break;
//         default:
//             Serial.println("Unknown");
//     }

//     // Print volume information
//     uint64_t cardSize = SD.cardSize() / (1024 * 1024);
//     Serial.print("Card size (MB):    ");
//     Serial.println(cardSize);

//     uint64_t usedBytes = SD.usedBytes() / (1024 * 1024);
//     uint64_t totalBytes = SD.totalBytes() / (1024 * 1024);
//     Serial.print("Used space (MB):   ");
//     Serial.println(usedBytes);
//     Serial.print("Total space (MB):  ");
//     Serial.println(totalBytes);

//     // List files on the card
//     Serial.println("\nFiles found on the card (name and size): ");
//     listFiles(SD, "/", 0);
// }