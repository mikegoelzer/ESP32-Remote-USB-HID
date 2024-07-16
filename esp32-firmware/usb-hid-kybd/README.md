# ESP32 Motherboard Power/Reset

Provides an HTTP interface to power on/off and reset a Gigabyte z790 Aorus Elite motherboard.

### Configure the project

```sh
cp sdkconfig.defaults.example sdkconfig.defaults
# Edit sdkconfig.defaults with your WiFi credentials
```

```sh
get_idf
idf.py set-target esp32
```

```sh
source find-esp32-port.source    # sets ESP32_PORT=/dev/[whatever]
```

### (Re-)Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```sh
idf.py build flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

To rebuild:

```sh
rm sdkconfig                # if Kconfig.h was changed
idf.py clean
idf.py set-target esp32
source find-esp32-port.source   # if needed
idf.py build
idf.py flash monitor
```

## Usage

### HTTP Endpoints

Status:

```sh
% curl http://10.0.0.24:52000/status
http [STATE_SYSTEM_ON]> idle (hdd=1)
```

Power on and off:

```sh
% curl http://10.0.0.24:52000/on
http [STATE_SYSTEM_OFF]> power on flag set (hdd=0)
```

```sh
% curl http://10.0.0.24:52000/off
http [STATE_SYSTEM_ON]> power off flag set (hdd=1)
```

Other commands:

```sh
% curl http://10.0.0.24:52000/help
    http [STATE_SYSTEM_OFF]>
    *** HELP ***
    /help       (this help)
    /power      (cycles power regardless of current state)
    /on         (turns on)
    /off        (turns off)
    /reset      (performs a hard reset)
    /status     (shows current status)
    /send-login (send u/p again)
```

### Buttons

 - Left button = Power On (same as `/on`)
 - Right button = Power Off (same as '/off')

### LEDs

  - Solid red = system off
  - Blinking green = system starting up
  - Solid green = system running
  - Blinking red = system shutting down 

## System-specific Configuration

  - Undef `SYSTEM_PRESENTS_LOGOUT_DIALOG` 
  
    Configure the Ubuntu machine not to prompt "are you sure?" on ACPI button shutdowns:

        ```sh
    $ gsettings set org.gnome.SessionManager logout-prompt false
    ```

    Source:  [askubuntu.com/questions/1272300/how-to-disable-shutdown-confirmation-on-ubuntu-20-04](https://askubuntu.com/questions/1272300/how-to-disable-shutdown-confirmation-on-ubuntu-20-04)

  - Undef `SYSTEM_PRESENTS_LOGIN_DIALOG` 
  
    Configure the Ubuntu machine with automatic login and no Gnome keychain password.

    - Enable automatic login:

        ```
        % sudo apt-get install -y ubuntu-gnome-desktop gnome-keyring gnome-keyring-pkcs11 libpam-gnome-keyring seahorse

        % sudo vi /etc/gdm3/custom.conf
        ```

        Edit the file:

        ```
        # /etc/gdm3/custom.conf

        AutomaticLoginEnable=True
        AutomaticLogin=mwg
        ```

    - Use `seahorse` in the GUI to set the keychain password for the `mwg` user to empty.


    Refs: [Configure automatic login](https://help.gnome.org/admin/system-admin-guide/stable/login-automatic.html.en) and [How can I stop being prompted to unlock 'default' keyring on boot?](https://askubuntu.com/a/224777)


## Troubleshooting
* If the server log shows "httpd_parse: parse_block: request URI/header too long", especially when handling POST requests, then you probably need to increase HTTPD_MAX_REQ_HDR_LEN, which you can find in the project configuration menu (`idf.py menuconfig`): Component config -> HTTP Server -> Max HTTP Request Header Length
