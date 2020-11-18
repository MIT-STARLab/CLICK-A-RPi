# CLICK-A-RPi



# Keep-alive

Configure executables as services

1. Create a user-level systemd config folder
   - \# ``mkdir ~/.config/systemd/user/``

2. Create systemd scripts 
   - for each executable (``camera-ipc.service``)
    ```
     [Unit]
     Description=Manage Service camera-ipc
     
     [Service]
     ExecStart=/home/pi/CLICK-A/github/camera/ipc/build/camera-ipc
     Restart=always
     RestartSec=3
     
     [Install]
     WantedBy=default.target

   ```
      
   - and each python script (``commandhandler.service``)
    ```
     [Unit]
     Description=Manage Service commandhandler
     
     [Service]
     ExecStart=/usr/bin/python3 /home/pi/CLICK-A/github/commandhandler/commandhandler.py
     Restart=always
     RestartSec=3
     
     [Install]
     WantedBy=default.target
   ```
   
   (please adjust folder structure accordingly)

3. Auto-start services on boot
   - enable lingering for user pi \
   \# ``loginctl enable-linger pi``
   - enable each service \
   \# ``systemctl --user enable camera`` \
   \# ``systemctl --user enable commandhandler``, ...

4. Start the services
   - \# ``systemctl --user start camera``
   - \# ``systemctl --user start commandhandler``
   - ...
   
