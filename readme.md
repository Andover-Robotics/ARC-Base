## ARC-Base
1. copy [OpenRC-Turbo](https://github.com/OpenFTC/OpenRC-Turbo) by making a new repo and importing it
2. run `git submodule add https://github.com/Andover-Robotics/ARC-Core` and Add "ARC-Core" to the Android project by adding `, ':ARC-Core'` prior to `':TeamCode'` in settings.gradle
3. install [ftcLib](https://docs.ftclib.org/ftclib/installation)
4. install [Roadrunner](https://acme-robotics.gitbook.io/road-runner/)
    - don't add the `maven { url = 'fjdsklafjklsd' }`
    - switch `0.5.3` with `0.5.4`
5. install [ftc-Dashboard](https://acmerobotics.github.io/ftc-dashboard/gettingstarted)
    - add *only* the exclusion to the FtcRobotController and TeamCode build.gradle
    - maybe replace `0.4.3` with `0.4.0` if 0.4.3 causes errors
    - don't add the `maven { url = 'https://maven.brott.dev/' }`
    - if this doesn't work, copy the FtcRobotControllerActivity in FtcRobotController from here
6. add code in TeamCode build.gradle
```java
buildscript {
  ext.kotlin_version = '1.5.20'

  repositories {
      mavenCentral()
  }
  dependencies {
      classpath "org.jetbrains.kotlin:kotlin-gradle-plugin:$kotlin_version"
  }
}
apply plugin: 'kotlin-android'

```
and 
```java
dependencies {
  implementation "org.jetbrains.kotlin:kotlin-stdlib:$kotlin_version"
  implementation "org.jetbrains.kotlin:kotlin-reflect:$kotlin_version"
  implementation 'org.apache.commons:commons-math3:3.6.1'
  implementation 'org.openftc:rev-extensions-2:1.2' //outdated: may not work with sdk 7.0
  implementation 'org.openftc:easyopencv:1.5.0'
  ...
```
7. add code to the end of build.common.gradle
```java
configurations.all {
    resolutionStrategy {
        force 'androidx.core:core-ktx:1.6.0'
    }
}
```
8. Change gradle version to 7.0.1/7.0.2 in project structure 
    - Alternatively change it when a prompt shows up or change it in `gradle-wrapper.properties`
9. copy this project's TeamCode folder
    - Autonomous pipeline does not work with the newest version of easyopencv
10. add the stuff to the control hubs
    - Copy the files from filesForDynamicLoad into the `first/vision` folder on the control hub
      - the filesForDynamicLoad folder can be found with the Project view instead of the Android view
      - the `first/vision` folder can be found by the file explorer when connected to the control hub
    - Add the game assets from the [OpenRC-Turbo](https://github.com/OpenFTC/OpenRC-Turbo/releases) into the `first/vision` folder
    - Add [libOpenCvAndroid453.so](https://github.com/OpenFTC/OpenCV-Repackaged/tree/master/doc/native_libs/armeabi-v7a) into the `first` folder
      - may or may not work i guess
    - Additional files may be needed
11. to set up floobits
    - install floobits plugin
    - got to tools>floobits>share project publicly