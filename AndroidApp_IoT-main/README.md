# Example Android App for IoT 

Simple Android App to monitor the cough activity with the delay of 1.6s 
(see the Firmware implementation)

The application unpack the audio signal from the microphone together with spike activity and translates it to the screen.
Meantime, each data window is uploaded to the FireBase

## Table of Contents
- [Version](#Version)
- [FireBase](#FireBase)
- [Packaging](#Packaging)

## Version
This app was tested on Android 7.0 Device and was developed on 
Android Studio Giraffe | 2022.3.1 Patch 4

## FireBase
Before start:
1. Modify the google-services.json:
   The file is automatically generated when you create a project in the FireBase Console. 
   Please refer to the documentation available online 
   https://firebase.google.com/docs/database?hl=it
2. Remember to ease the rules in the database (They should be permissive if you're testing):
   ```bash
   {
      "rules": {
      ".read": true,
      ".write": true
      }
   }
## Packaging

The data arrives as a byte array in the following format for the first package
- [AA][AA][n_coughs][size_of_window][data_0][data_1]....[data_n-1]
and for the second and following packages
- [BB][BB][data_n][data_n+1]....[data_size_of_window-1]

## Author
This application was created by Anna Burdina. The original repository of the app is available here : https://github.com/annaburds/AndroidApp_IoT