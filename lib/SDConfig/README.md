# SDConfig

## Introduction

SDConfig is an Arduino library for reading settings from a configuration file on an SD card.

It is based in SDConfigFile library by Bradford Needham (https://github.com/bneedhamia/sdconfigfile)

Given a configuration file with settings whose format is:

    # for comments
    setting=value

    # this is an example

    # for any string
    setting1=some_string
    # able to be parsed as a boolean
    setting2=true
    # able to be parsed as an integer
    setting3=123
    # able to be parsed as an IP address
    setting4=10.1.1.2

It is necessary to specify the configuration file name (i.e. **config.cfg**) to load its contents through the library methods.

## Usage

The following code shows the basic usage of the library taking the settings from the previous configuration file example:

    #include <SD.h>
    #include <SDConfig.h>

    SDConfig cfg;

    setup(){
      int pinSelect = 4;
      int maxLineLength = 10;
      SD.begin(pinSelect);
    }
    
    loop(){
      //Initialize SDConfig object
      if(cfg.begin(fileName, maxLineLength)) { 
        while (cfg.readNextSetting()) {
          if(cfg.nameIs("setting1")){
            char *string = cfg.copyValue();
          }else if (cfg.nameIs("setting2")) {
            boolean bool = cfg.getBooleanValue();
          }else if(cfg.nameIs("setting3")){
              int i = cfg.getIntValue();
          }else if(cfg.nameIs("setting4")){
              IPAddress ip = cfg.getIPAddress();
          }else{
            Serial.Print("The name of this setting is:");
            Serial.println(cfg.getName());
          }
        }
      cfg.end();
      }
      //Sentences using the initialised variables
    }