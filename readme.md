This code can be used to characterize the Romi robot through SysId. 

To use it follow these steps:
1. Connect to your Romi robot via WiFi
2. Run this code by using **WPILib: Simulate Robot Code**.
3. Start the SysId tool by selecting **WPILib: Start Tool** and then selecting **SysId**.
4. Connect SysId to the Romi:
    1. In the **Generator** section:
        1. Type **localhost** into the **Team/IP** box.
        2. Select **Romi** in the **Analysis Type** dropdown.
    2. In the **Logger** section:
        1. Select **Client** in the **Mode** dropdown.
        2. Type **localhost** into the **Team/IP** box.
        3. Click **Apply**.
        4. **NT Connected** should now appear in green, indicating that SysId is reading from the Network Tables.
        5. In **Project Parameters**, Units should be set to Meters and Units Per Rotation should be 1.0 (the code already reports distance in meters).
5. Ensure that you have sufficient open space (at least 10', with 20' being better) for the Romi to run the characterization routines.
6. At this point, you can follow the steps for (Running the Identification Routine)[file:///C:/Users/Public/wpilib/2022/documentation/rtd/frc-docs-latest/docs/software/pathplanning/system-identification/identification-routine.html#running-the-identification-routine] in the WPILib documentaton.