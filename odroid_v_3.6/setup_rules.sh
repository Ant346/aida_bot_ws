#! /bin/bash
   sudo cp /home/aida/projects/aida_bot_ws/odroid_v_3.6/99-canable2.rules /etc/udev/rules.d/99-canable2.rules
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   echo "Rules setup complete"