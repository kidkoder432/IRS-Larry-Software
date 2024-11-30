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
                        double selectedEntry = 0.0;
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
                        double value;
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