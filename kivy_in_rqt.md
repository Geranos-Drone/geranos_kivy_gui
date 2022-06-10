Show Kivy GUI in rqt

cd ~/catkin_ws/src
git clone git@github.com:awesomebytes/rqt_embed_window.git
catkin build rqt_embed_window

make sure your conda pythton3 environment is set in the run_py3.sh script

rqt

Plugins --> Miscellaneous Tools --> Embed a graphical Window
Enter in window:
~/catkin_ws/src/mav_ui/geranos_gui_test/scripts/run_py3.sh

Geranos


or rqt
Perspective --> Import..
~/catkin_ws/src/mav_tools/mav_startup/scripts/geranos/Geranos_Boreas_New_GUI.perspective