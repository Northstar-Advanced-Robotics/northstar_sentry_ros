# Systemd Service Management Commands

This document lists the most useful commands for managing a systemd `.service` file on Ubuntu/Linux (including Jetson devices), along with clear descriptions of what each command does.

---

## ✔ Summary Table

| Task            | Command                                |
| --------------- | -------------------------------------- |
| Enable on boot  | `systemctl enable localization`        |
| Disable on boot | `systemctl disable localization`       |
| Start           | `systemctl start localization`         |
| Stop            | `systemctl stop localization`          |
| Restart         | `systemctl restart localization`       |
| Status          | `systemctl status localization`        |
| View logs       | `journalctl -u localization`           |
| Follow logs     | `journalctl -u localization -f`        |
| Reload systemd  | `systemctl daemon-reload localization` |
| Mask service    | `systemctl mask localization`          |
| Unmask service  | `systemctl unmask localization`        |

---


## 🟢 Enable / Disable a Service

### **Enable service at startup**

```bash
sudo systemctl enable localization.service
```

Makes the service start automatically on boot.

### **Disable service at startup**

```bash
sudo systemctl disable localization.service
```

Prevents the service from starting on boot.

---

## ▶️ Start / Stop / Restart a Service

### **Start the service now**

```bash
sudo systemctl start localization.service
```

Runs the service immediately.

### **Stop the service now**

```bash
sudo systemctl stop localization.service
```

Stops a running service.

### **Restart the service**

```bash
sudo systemctl restart localization.service
```

Stops and re-launches the service.

---

## 🔍 Check Service Status

### **Show current status**

```bash
systemctl status localization.service
```

Shows whether the service is running, failed, or stopped.
Includes logs and the last start attempt.

---

## 📜 View Logs

### **View all logs for the service**

```bash
journalctl -u localization.service
```

Shows the full log history.

### **Follow logs in real-time**

```bash
journalctl -u localization.service -f
```

Displays live logs as the program runs.

---

## 🔄 Reload Systemd When Service Files Change

### **Reload systemd after editing a .service file**

```bash
sudo systemctl daemon-reload
```

Required after modifying a service file.

---

## 🛑 Mask / Unmask a Service

### **Mask the service**

```bash
sudo systemctl mask localization.service
```

Prevents the service from being started *manually or automatically*. More strict than disable.

### **Unmask the service**

```bash
sudo systemctl unmask localization.service
```

Allows the service to be started again.

---

## 📂 Check if the Service is Enabled or Disabled

### **Query enable/disable status**

```bash
systemctl is-enabled localization.service
```

Outputs `enabled`, `disabled`, or `static`.

---

## 🧹 Clear Logs (optional)

### **Clear old logs**

```bash
sudo journalctl --vacuum-time=1d
```

Deletes logs older than 1 day.

---

## 📝 Edit a Service File

### **Open the service file**

```bash
sudo nano /etc/systemd/system/localization.service
```

Edit your service configuration.

---

## 🗑️ Remove a Service

### **Delete the service file**

```bash
sudo rm /etc/systemd/system/localization.service
```

### **Reload systemd**

```bash
sudo systemctl daemon-reload
```

Final cleanup.

---

