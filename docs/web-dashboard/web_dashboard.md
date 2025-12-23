# Tea Brewer Web Dashboard

The Tea Brewer includes a web-based dashboard that provides remote control and monitoring capabilities over WiFi. The dashboard mirrors the functionality of the physical LVGL display, allowing you to brew tea from any device on your network.

## Accessing the Dashboard

### Initial Setup (Captive Portal)

On first boot or after a WiFi reset, the Tea Brewer creates an Access Point (AP) for configuration:

1. **Connect to the AP** - Look for a WiFi network named `TeaBrewer` (or similar)
2. **Captive Portal** - A configuration page will automatically open
3. **Enter WiFi Credentials** - Provide your home WiFi network name and password
4. **Save & Connect** - The device will restart and connect to your network

### Accessing the Dashboard

Once connected to your WiFi network, access the dashboard via:

- **mDNS**: `http://teabrewer.local` (recommended)
- **IP Address**: Check your router or serial monitor for the assigned IP

## Dashboard Screenshot

![Tea Brewer Dashboard](dashboard_screenshot.png)

## Features

### Current Temperature
- **Real-time temperature display** with glowing effect
- **Motor status indicators**:
  - Connection status (Connected/Disconnected)
  - Homing status (Homed/Not Homed)
  - Current position percentage

### Brew Status
- **Current state**: Idle, Brewing, Infusing, Teabag Dropoff, or Scheduled
- **Current tea type** being brewed
- **Progress bar** during active brewing (shows percentage complete)

### Select Tea Type
- **6 tea types** with color-coded buttons:
  - Green Tea
  - Black Tea
  - Herbal Tea
  - Fruit Tea
  - White Tea
  - Functional Tea
- **Temperature setting** (75°C - 100°C in 5° increments)
- **Infusion time** configuration (minutes and seconds)
- **Save Tea Settings** button to persist changes

### Brew Controls
- **Start Brewing** - Begin brewing with the selected tea type
- **Stop** - Emergency stop during brewing

### Schedule Brew
- **Set time** (hour and minute) for scheduled brewing
- **Target temperature** for pre-heating
- **Set Schedule** / **Cancel Schedule** buttons
- Visual indicator showing scheduled time

### Settings
- **Idle Position** - Set and save the motor's idle/rest position
- **Go to Idle Position** - Move motor to the saved idle position

### Reset WiFi
- **Reset WiFi button** (top-right corner)
- Clears saved WiFi credentials
- Restarts device in AP mode for reconfiguration
- Confirmation dialog to prevent accidental resets

## Technical Details

### API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Dashboard HTML page |
| `/api/status` | GET | Current system status (JSON) |
| `/api/tea/:index` | GET | Get tea settings for index 0-5 |
| `/api/tea/:index` | POST | Save tea settings |
| `/api/brew` | POST | Start brewing |
| `/api/brew/stop` | POST | Stop brewing |
| `/api/schedule` | POST | Set scheduled brew |
| `/api/schedule/cancel` | POST | Cancel scheduled brew |
| `/api/settings/idle_position` | POST | Save idle position |
| `/api/motor/go_idle` | POST | Move to idle position |
| `/api/wifi/reset` | POST | Reset WiFi and restart |

### Network Features

- **mDNS (Bonjour)** - Access via `teabrewer.local`
- **HTTP Server** - Port 80
- **Auto-refresh** - Status updates every 1 second
- **Toast Notifications** - Visual feedback for all actions

### Browser Compatibility

The dashboard works on all modern browsers:
- Chrome / Edge
- Firefox
- Safari (macOS / iOS)
- Android browsers

## Toast Notifications

The dashboard provides visual feedback through toast notifications:

| Action | Message | Type |
|--------|---------|------|
| Save tea settings | "Tea settings saved!" | Success |
| Start brew | "Brewing started!" | Success |
| Stop brew | "Brewing stopped" | Warning |
| Set schedule | "Schedule set for HH:MM" | Success |
| Cancel schedule | "Schedule cancelled" | Warning |
| Save idle position | "Idle position saved!" | Success |
| Go to idle | "Moving to idle position..." | Info |
| Reset WiFi | "Resetting WiFi..." | Warning |
| Any failure | "Failed to..." | Error |
