1. Install the things of power
pip3 install pyinstaller pillow

2. Create / choose an icon - from web

3. Build the executable
pyinstaller --onefile --windowed --icon=icon.png gui.py

Explanation, nerd version but fun:

--onefile = single executable

--windowed = no terminal window pops up

--icon=icon.png = shiny desktop candy

After a bit of scrolling text, you get:
     dist/gui


That gui file is now your executable.

4. Create a real Ubuntu desktop app with icon
    nano ~/Desktop/XArmRunner.desktop

    Paste this inside:

        [Desktop Entry]
        Name=XArm Control Panel
        Comment=Run and control xArm7 script
        Exec=/home/vmlabs/xarm7_cowork/gui/dist/gui
        Icon=/home/vmlabs/xarm7_cowork/gui/icon.png
        Terminal=false
        Type=Application
        Categories=Development;Robotics;

Save (Ctrl+O, Enter, Ctrl+X)

Now make it executable:

chmod +x ~/Desktop/XArmRunner.desktop


Right-click it → Allow Launching

Now… double click it.

You just made a native Ubuntu application. Like a deity.

5. Add to applications menu

If you want it in the system app list too:

cp ~/Desktop/XArmRunner.desktop ~/.local/share/applications/


Now it shows in the launcher when you press the Super key.