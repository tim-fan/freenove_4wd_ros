# systemd

Files for configuring systemd to run driver at boot.

## Setup

1) edit [`ros_driver.service`](./ros_driver.service):
    1) set `USER` as your username
    1) set `WorkingDirectory` as the path to your colcon workspace
1) copy [`ros_driver.service`](./ros_driver.service) into `/etc/systemd/system/`:
    ```bash
    sudo cp ros_driver.service /etc/systemd/system/
    ```
1) Reload and enable the systemd service:
    ```bash
    sudo systemctl daemon-reload
    sudo systemctl enable ros_driver.service
    ```

## Test

After the above, the service can be run immediately (without a reboot) with:

```bash
sudo systemctl start ros_driver.service
```

The check status to confirm driver is running with no errors:
```
systemctl status ros_driver.service
```
