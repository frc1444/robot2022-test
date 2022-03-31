# Robot 2022 Test
Development repository for 2022 robot code. Based on WPILib templates.


### Commands
NOTE: These commands work on Linux/Mac but must be altered on Windows.
To alter them on Windows, replace `./gradlew` with `gradlew.bat`

```shell script
# Building: (Make sure code is correct)
./gradlew build

# Downloading everything for WPI (Do this if you aren't going to have internet! (Like before competition!))
./gradlew downloadDepsPreemptively

# Deploying:
./gradlew deploy

# Launching Shuffleboard: (The dashboard we use)
./gradlew shuffleboard

# Launching OutlineViewer: (Good for debugging NetworkTable values)
./gradlew outlineviewer

# Running WPI Simulation
./gradlew simulateJava

```
