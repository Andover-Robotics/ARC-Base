## ARC-Base
1. Copy [OpenRC-Turbo](https://github.com/OpenFTC/OpenRC-Turbo)
2. run `git submodule add https://github.com/Andover-Robotics/ARC-Core` and Add "ARC-Core" to the Android project by adding `, ':ARC-Core'` prior to `':TeamCode'` in settings.gradle
3. install [ftcLib](https://docs.ftclib.org/ftclib/installation)
4. install [Roadrunner](https://acme-robotics.gitbook.io/road-runner/)
    - don't add the maven { url = 'fjdsklafjklsd' }
    - switch `0.5.3` with `0.5.4`
5. install [ftc-Dashboard](https://acmerobotics.github.io/ftc-dashboard/gettingstarted)
    - add *only* the exclusion to the FtcRobotController and TeamCode build.gradle
    - maybe replace `0.4.3` with `0.4.0` if 0.4.3 causes errors
    - don't add the `maven { url = 'https://maven.brott.dev/' }`
    - if this doesn't work, copy the FtcRobotControllerActivity in FtcRobotController from here
6. remove code
    - `implementation 'org.ftclib.ftclib:*'` in TeamCode build.gradle 
7. add code in TeamCode build.gradle
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
  implementation 'org.openftc:rev-extensions-2:1.2'
  implementation 'org.openftc:easyopencv:1.1'
  implementation 'com.arcrobotics:ftclib:1.2.0-beta' // FTCLib update?
  ...
```
8. add code to the end of build.common.gradle
```java
configurations.all {
    resolutionStrategy {
        force 'androidx.core:core-ktx:1.6.0'
    }
}
```
9. Change gradle version to 7.0.1 in `gradle-wrapper.properties` 
    - Also follow the android studio prompt to update gradle when it appears
10. add `android.useAndroidX=true` to gradle.properties
11. copy this project's TeamCode folder
12. add the stuff to the control hubs
    - add here
    
