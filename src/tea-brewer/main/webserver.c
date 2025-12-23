/**
 * @file webserver.c
 * @brief Web server dashboard for Tea Brewer
 */

#include "webserver.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "mdns.h"
#include "settings.h"
#include "uart_comm.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "webserver";

static httpd_handle_t server = NULL;

/* Forward declarations from ui_events - avoid including LVGL headers */
typedef enum {
    WEB_BREW_STATE_IDLE = 0,
    WEB_BREW_STATE_BREWING,
    WEB_BREW_STATE_INFUSING,
    WEB_BREW_STATE_TEABAG_DROPOFF,
    WEB_BREW_STATE_SCHEDULED
} web_brew_state_t;

extern uint8_t current_tea_index;
extern web_brew_state_t ui_get_brew_state(void);
extern uint8_t ui_get_brew_progress(void);
extern void ui_start_schedule_brew(uint8_t hour, uint8_t minute, uint8_t target_temp);
extern void ui_cancel_schedule_brew(void);
extern void brewNow(void *e);
extern void stopBrewing(void *e);
extern void wifi_config_reset(void);

/* Tea names - must match ui.c */
static const char* tea_names[MAX_TEA_TYPES] = {
    "Green Tea",
    "Black Tea",
    "Herbal Tea",
    "Fruit Tea",
    "White Tea",
    "Functional Tea"
};

/* Tea colors (hex values) */
static const char* tea_colors_hex[MAX_TEA_TYPES] = {
    "#92A202",  // Green Tea
    "#CE9958",  // Black Tea
    "#E7C789",  // Herbal Tea
    "#D76C6C",  // Fruit Tea
    "#C4BCB5",  // White Tea
    "#E9D257"   // Functional Tea
};

/* External from ui_events.c */
extern uint8_t current_tea_index;

/* Dashboard HTML - embedded directly */
static const char DASHBOARD_HTML[] = 
"<!DOCTYPE html>"
"<html lang=\"en\">"
"<head>"
"<meta charset=\"UTF-8\">"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
"<link rel=\"icon\" href=\"data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'><text y='.9em' font-size='90'>🍵</text></svg>\">"
"<title>Tea Brewer Dashboard</title>"
"<style>"
"*{box-sizing:border-box;margin:0;padding:0}"
"body{font-family:'Segoe UI',Tahoma,sans-serif;background:#ECE2CE;color:#333;min-height:100vh;padding:20px}"
".container{max-width:1200px;margin:0 auto}"
".header{display:flex;justify-content:space-between;align-items:center;margin-bottom:30px}"
".header h1{margin:0;text-align:center;flex:1;font-size:2em;color:#5a4a3a}"
".btn-reset-wifi{padding:10px 15px;background:#e74c3c;color:#fff;border:none;border-radius:8px;cursor:pointer;font-size:0.85em;font-weight:bold;transition:all 0.3s}"
".btn-reset-wifi:hover{background:#c0392b}"
".card{background:#fff;border-radius:15px;padding:20px;margin-bottom:20px;box-shadow:0 4px 15px rgba(0,0,0,0.1)}"
".card h2{margin-bottom:15px;font-size:1.3em;border-bottom:1px solid #ddd;padding-bottom:10px;color:#5a4a3a}"
".temp-display{text-align:center;font-size:3em;font-weight:bold;color:#4ecdc4;text-shadow:0 0 20px rgba(78,205,196,0.5)}"
".status-row{display:flex;justify-content:space-between;margin:10px 0;padding:8px 0;border-bottom:1px solid #eee}"
".status-label{color:#888}"
".status-value{font-weight:bold;color:#333}"
".tea-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin:15px 0}"
".tea-btn{padding:15px 10px;border:none;border-radius:10px;cursor:pointer;font-size:0.9em;font-weight:bold;color:#fff;transition:all 0.3s;text-shadow:1px 1px 2px rgba(0,0,0,0.3)}"
".tea-btn:hover{transform:scale(1.05);box-shadow:0 5px 15px rgba(0,0,0,0.3)}"
".tea-btn.selected{box-shadow:0 0 0 3px #5a4a3a,0 5px 20px rgba(0,0,0,0.2)}"
".control-row{display:flex;gap:10px;margin:15px 0;flex-wrap:wrap}"
".btn{padding:15px 25px;border:none;border-radius:10px;cursor:pointer;font-size:1em;font-weight:bold;transition:all 0.3s;flex:1;min-width:120px}"
".btn-brew{background:#4ecdc4;color:#fff}"
".btn-brew:hover{background:#44a08d}"
".btn-stop{background:#e74c3c;color:#fff}"
".btn-stop:hover{background:#c0392b}"
".btn-schedule{background:#9b59b6;color:#fff}"
".btn-schedule:hover{background:#8e44ad}"
".btn-cancel{background:#95a5a6;color:#fff}"
".settings-row{display:flex;align-items:center;justify-content:space-between;margin:15px 0;padding:10px;background:#f5f0e6;border-radius:8px}"
".settings-row label{flex:1;color:#5a4a3a}"
".settings-row input,.settings-row select{padding:10px;border-radius:5px;border:1px solid #ddd;background:#fff;color:#333;width:100px;font-size:1em}"
".schedule-inputs{display:flex;gap:10px;align-items:center;flex-wrap:wrap}"
".schedule-inputs input{width:60px;text-align:center}"
".schedule-inputs span{font-size:1.5em;color:#5a4a3a}"
".progress-bar{width:100%;height:20px;background:#e0d6c6;border-radius:10px;overflow:hidden;margin:10px 0}"
".progress-fill{height:100%;background:#4ecdc4;transition:width 0.5s;border-radius:10px}"
".hidden{display:none}"
".motor-status{display:flex;gap:20px;justify-content:center;flex-wrap:wrap;margin-top:10px}"
".motor-indicator{padding:5px 15px;border-radius:20px;font-size:0.85em}"
".motor-connected{background:#27ae60;color:#fff}"
".motor-disconnected{background:#e74c3c;color:#fff}"
".motor-homed{background:#3498db;color:#fff}"
".motor-not-homed{background:#f39c12;color:#fff}"
".two-column{display:grid;grid-template-columns:1fr 1fr;gap:20px}"
"@media(max-width:600px){.tea-grid{grid-template-columns:repeat(2,1fr)}.temp-display{font-size:2.5em}h1{font-size:1.5em}.two-column{grid-template-columns:1fr}}"
".toast-container{position:fixed;bottom:20px;right:20px;z-index:1000;display:flex;flex-direction:column;gap:10px}"
".toast{padding:15px 25px;border-radius:10px;color:#fff;font-weight:bold;box-shadow:0 4px 15px rgba(0,0,0,0.2);animation:slideIn 0.3s ease,fadeOut 0.3s ease 2.7s forwards;max-width:350px}"
".toast-success{background:#27ae60}"
".toast-error{background:#e74c3c}"
".toast-info{background:#3498db}"
".toast-warning{background:#f39c12}"
"@keyframes slideIn{from{transform:translateX(100%);opacity:0}to{transform:translateX(0);opacity:1}}"
"@keyframes fadeOut{from{opacity:1}to{opacity:0}}"
"</style>"
"</head>"
"<body>"
"<div class=\"container\">"
"<div class=\"header\">"
"<div style=\"width:120px\"></div>"
"<h1>🍵 Tea Brewer Dashboard</h1>"
"<button class=\"btn-reset-wifi\" onclick=\"resetWifi()\">Reset WiFi</button>"
"</div>"

"<div class=\"two-column\">"
"<div class=\"column-left\">"

"<div class=\"card\">"
"<h2>Current Temperature</h2>"
"<div class=\"temp-display\" id=\"temperature\">--.- °C</div>"
"<div class=\"motor-status\">"
"<span class=\"motor-indicator\" id=\"motor-conn\">Disconnected</span>"
"<span class=\"motor-indicator\" id=\"motor-home\">Not Homed</span>"
"<span class=\"motor-indicator\" id=\"motor-pos\">Position: --%</span>"
"</div>"
"</div>"

"<div class=\"card\">"
"<h2>Brew Status</h2>"
"<div class=\"status-row\"><span class=\"status-label\">State:</span><span class=\"status-value\" id=\"brew-state\">Idle</span></div>"
"<div class=\"status-row\"><span class=\"status-label\">Current Tea:</span><span class=\"status-value\" id=\"current-tea\">-</span></div>"
"<div id=\"progress-container\" class=\"hidden\">"
"<div class=\"status-row\"><span class=\"status-label\">Progress:</span><span class=\"status-value\" id=\"progress-value\">0%</span></div>"
"<div class=\"progress-bar\"><div class=\"progress-fill\" id=\"progress-bar\" style=\"width:0%\"></div></div>"
"</div>"
"</div>"

"<div class=\"card\">"
"<h2>Select Tea Type</h2>"
"<div class=\"tea-grid\" id=\"tea-grid\"></div>"
"<div class=\"settings-row\"><label>Temperature (°C):</label><select id=\"tea-temp\"><option value=\"75\">75</option><option value=\"80\">80</option><option value=\"85\">85</option><option value=\"90\">90</option><option value=\"95\">95</option><option value=\"100\">100</option></select></div>\n"
"<div class=\"settings-row\"><label>Infusion Time:</label><div class=\"schedule-inputs\"><input type=\"number\" id=\"infuse-min\" min=\"0\" max=\"14\" value=\"2\"><span>:</span><input type=\"number\" id=\"infuse-sec\" min=\"0\" max=\"59\" value=\"0\"></div></div>"
"<button class=\"btn\" style=\"background:#3498db;color:#fff;margin-top:10px\" onclick=\"saveTeaSettings()\">Save Tea Settings</button>"
"</div>"

"</div>"
"<div class=\"column-right\">"

"<div class=\"card\">"
"<h2>Brew Controls</h2>"
"<div class=\"control-row\">"
"<button class=\"btn btn-brew\" id=\"btn-brew\" onclick=\"startBrew()\">Start Brewing</button>"
"<button class=\"btn btn-stop hidden\" id=\"btn-stop\" onclick=\"stopBrew()\">Stop</button>"
"</div>"
"</div>"

"<div class=\"card\">"
"<h2>Schedule Brew</h2>"
"<div class=\"settings-row\"><label>Time:</label><div class=\"schedule-inputs\"><input type=\"number\" id=\"sched-hour\" min=\"0\" max=\"23\" value=\"7\"><span>:</span><input type=\"number\" id=\"sched-min\" min=\"0\" max=\"59\" value=\"0\"></div></div>"
"<div class=\"settings-row\"><label>Target Temp (°C):</label><input type=\"number\" id=\"sched-temp\" min=\"25\" max=\"100\" value=\"60\"></div>"
"<div class=\"control-row\">"
"<button class=\"btn btn-schedule\" id=\"btn-schedule\" onclick=\"setSchedule()\">Set Schedule</button>"
"<button class=\"btn btn-cancel hidden\" id=\"btn-cancel-sched\" onclick=\"cancelSchedule()\">Cancel Schedule</button>"
"</div>"
"<div id=\"schedule-info\" class=\"hidden\" style=\"margin-top:15px;padding:15px;background:rgba(155,89,182,0.3);border-radius:10px\">"
"<span>Scheduled for: <strong id=\"schedule-time\">--:--</strong></span>"
"</div>"
"</div>"

"<div class=\"card\">"
"<h2>Settings</h2>"
"<div class=\"settings-row\"><label>Idle Position (%):</label><input type=\"number\" id=\"idle-pos\" min=\"0\" max=\"100\" value=\"0\"></div>"
"<div class=\"control-row\"><button class=\"btn\" style=\"background:#27ae60;color:#fff\" onclick=\"saveIdlePosition()\">Save Idle Position</button><button class=\"btn\" style=\"background:#3498db;color:#fff\" onclick=\"goToIdlePosition()\">Go to Idle Position</button></div>\n"
"</div>"

"</div>"
"</div>"

"</div>"

"<div class=\"toast-container\" id=\"toast-container\"></div>"

"<script>"
"function showToast(message,type='info'){const container=document.getElementById('toast-container');const toast=document.createElement('div');toast.className='toast toast-'+type;toast.textContent=message;container.appendChild(toast);setTimeout(()=>toast.remove(),3000);}"
"const teaNames=['Green Tea','Black Tea','Herbal Tea','Fruit Tea','White Tea','Functional Tea'];"
"const teaColors=['#92A202','#CE9958','#E7C789','#D76C6C','#C4BCB5','#E9D257'];"
"let selectedTea=0;"
"let currentState='idle';"
"let idlePosLoaded=false;"

"function initTeaGrid(){"
"const grid=document.getElementById('tea-grid');"
"teaNames.forEach((name,i)=>{"
"const btn=document.createElement('button');"
"btn.className='tea-btn'+(i===selectedTea?' selected':'');"
"btn.style.background=teaColors[i];"
"btn.textContent=name;"
"btn.onclick=()=>selectTea(i);"
"btn.id='tea-'+i;"
"grid.appendChild(btn);"
"});"
"}"

"function selectTea(index){"
"document.querySelectorAll('.tea-btn').forEach(b=>b.classList.remove('selected'));"
"document.getElementById('tea-'+index).classList.add('selected');"
"selectedTea=index;"
"loadTeaSettings(index);"
"}"

"async function loadTeaSettings(index){"
"try{"
"const res=await fetch('/api/tea/'+index);"
"const data=await res.json();"
"document.getElementById('tea-temp').value=data.temperature;"
"const mins=Math.floor(data.infusion_time/60);"
"const secs=data.infusion_time%60;"
"document.getElementById('infuse-min').value=mins;"
"document.getElementById('infuse-sec').value=secs;"
"}catch(e){console.error(e);}"
"}"

"async function saveTeaSettings(){"
"const temp=parseInt(document.getElementById('tea-temp').value);"
"const mins=parseInt(document.getElementById('infuse-min').value);"
"const secs=parseInt(document.getElementById('infuse-sec').value);"
"const totalSecs=mins*60+secs;"
"try{"
"await fetch('/api/tea/'+selectedTea,{"
"method:'POST',"
"headers:{'Content-Type':'application/json'},"
"body:JSON.stringify({temperature:temp,infusion_time:totalSecs})"
"});"
"showToast('Tea settings saved!','success');"
"}catch(e){console.error(e);showToast('Failed to save settings','error');}"
"}"

"async function startBrew(){"
"try{"
"await fetch('/api/brew',{"
"method:'POST',"
"headers:{'Content-Type':'application/json'},"
"body:JSON.stringify({tea_index:selectedTea})"
"});"
"showToast('Brewing started!','success');"
"}catch(e){console.error(e);showToast('Failed to start brew','error');}"
"}"

"async function stopBrew(){"
"try{await fetch('/api/brew/stop',{method:'POST'});showToast('Brewing stopped','warning');}catch(e){console.error(e);showToast('Failed to stop brew','error');}"
"}"

"async function setSchedule(){"
"const hour=parseInt(document.getElementById('sched-hour').value);"
"const minute=parseInt(document.getElementById('sched-min').value);"
"const temp=parseInt(document.getElementById('sched-temp').value);"
"try{"
"await fetch('/api/schedule',{"
"method:'POST',"
"headers:{'Content-Type':'application/json'},"
"body:JSON.stringify({hour:hour,minute:minute,target_temp:temp,tea_index:selectedTea})"
"});"
"showToast('Schedule set for '+hour.toString().padStart(2,'0')+':'+minute.toString().padStart(2,'0'),'success');"
"}catch(e){console.error(e);showToast('Failed to set schedule','error');}"
"}"

"async function cancelSchedule(){"
"try{await fetch('/api/schedule/cancel',{method:'POST'});showToast('Schedule cancelled','warning');}catch(e){console.error(e);showToast('Failed to cancel schedule','error');}"
"}"

"async function saveIdlePosition(){"
"const pos=parseInt(document.getElementById('idle-pos').value);"
"try{"
"await fetch('/api/settings/idle_position',{"
"method:'POST',"
"headers:{'Content-Type':'application/json'},"
"body:JSON.stringify({position:pos})"
"});"
"showToast('Idle position saved!','success');"
"}catch(e){console.error(e);showToast('Failed to save idle position','error');}"
"}"

"async function goToIdlePosition(){"
"try{"
"await fetch('/api/motor/go_idle',{method:'POST'});"
"showToast('Moving to idle position...','info');"
"}catch(e){console.error(e);showToast('Failed to move motor','error');}"
"}"

"async function resetWifi(){"
"if(!confirm('Reset WiFi settings? The device will restart and create a new access point for configuration.'))return;"
"try{"
"showToast('Resetting WiFi...','warning');"
"await fetch('/api/wifi/reset',{method:'POST'});"
"}catch(e){console.error(e);}"
"}"

"function updateUI(data){"
"document.getElementById('temperature').textContent=data.temperature.toFixed(1)+' °C';"
"document.getElementById('brew-state').textContent=data.brew_state.charAt(0).toUpperCase()+data.brew_state.slice(1);"
"document.getElementById('current-tea').textContent=teaNames[data.current_tea_index]||'-';"

"const connEl=document.getElementById('motor-conn');"
"connEl.textContent=data.motor_connected?'Connected':'Disconnected';"
"connEl.className='motor-indicator '+(data.motor_connected?'motor-connected':'motor-disconnected');"

"const homeEl=document.getElementById('motor-home');"
"homeEl.textContent=data.motor_homed?'Homed':'Not Homed';"
"homeEl.className='motor-indicator '+(data.motor_homed?'motor-homed':'motor-not-homed');"

"document.getElementById('motor-pos').textContent='Position: '+data.motor_position.toFixed(0)+'%';"

"const progContainer=document.getElementById('progress-container');"
"const btnBrew=document.getElementById('btn-brew');"
"const btnStop=document.getElementById('btn-stop');"
"const btnSched=document.getElementById('btn-schedule');"
"const btnCancelSched=document.getElementById('btn-cancel-sched');"
"const schedInfo=document.getElementById('schedule-info');"

"currentState=data.brew_state;"

"if(data.brew_state==='idle'){"
"progContainer.classList.add('hidden');"
"btnBrew.classList.remove('hidden');"
"btnStop.classList.add('hidden');"
"btnSched.classList.remove('hidden');"
"btnCancelSched.classList.add('hidden');"
"schedInfo.classList.add('hidden');"
"}else if(data.brew_state==='scheduled'){"
"progContainer.classList.add('hidden');"
"btnBrew.classList.add('hidden');"
"btnStop.classList.add('hidden');"
"btnSched.classList.add('hidden');"
"btnCancelSched.classList.remove('hidden');"
"schedInfo.classList.remove('hidden');"
"document.getElementById('schedule-time').textContent=data.schedule_hour.toString().padStart(2,'0')+':'+data.schedule_minute.toString().padStart(2,'0');"
"}else{"
"progContainer.classList.remove('hidden');"
"document.getElementById('progress-value').textContent=data.progress+'%';"
"document.getElementById('progress-bar').style.width=data.progress+'%';"
"btnBrew.classList.add('hidden');"
"btnStop.classList.remove('hidden');"
"btnSched.classList.add('hidden');"
"btnCancelSched.classList.add('hidden');"
"schedInfo.classList.add('hidden');"
"}"

"if(!idlePosLoaded){document.getElementById('idle-pos').value=data.idle_position;idlePosLoaded=true;}"
"}"

"async function pollStatus(){"
"try{"
"const res=await fetch('/api/status');"
"const data=await res.json();"
"updateUI(data);"
"}catch(e){console.error(e);}"
"}"

"initTeaGrid();"
"loadTeaSettings(0);"
"pollStatus();"
"setInterval(pollStatus,1000);"
"</script>"
"</body>"
"</html>";

/* ============================================
   API HANDLERS
   ============================================ */

/* Brew state names */
static const char* brew_state_strings[] = {
    "idle",
    "brewing",
    "infusing",
    "teabag_dropoff",
    "scheduled"
};

/* GET /api/status - Get current status */
static esp_err_t status_get_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    
    /* Get temperature */
    float obj_temp, amb_temp;
    uart_comm_get_cached_temperature(&obj_temp, &amb_temp);
    cJSON_AddNumberToObject(root, "temperature", obj_temp);
    cJSON_AddNumberToObject(root, "ambient_temp", amb_temp);
    
    /* Get motor status */
    motor_status_t motor_status;
    uart_comm_get_cached_status(&motor_status);
    cJSON_AddBoolToObject(root, "motor_connected", motor_status.is_connected);
    cJSON_AddBoolToObject(root, "motor_homed", motor_status.is_homed);
    cJSON_AddBoolToObject(root, "motor_calibrated", motor_status.is_calibrated);
    cJSON_AddNumberToObject(root, "motor_position", motor_status.position_percent);
    
    /* Get pot presence */
    bool pot_present;
    uint16_t pot_distance;
    uart_comm_get_cached_pot_presence(&pot_present, &pot_distance);
    cJSON_AddBoolToObject(root, "pot_present", pot_present);
    cJSON_AddNumberToObject(root, "pot_distance", pot_distance);
    
    /* Get current tea index */
    extern uint8_t current_tea_index;
    cJSON_AddNumberToObject(root, "current_tea_index", current_tea_index);
    
    /* Get actual brew state from ui_events */
    web_brew_state_t brew_state = ui_get_brew_state();
    cJSON_AddStringToObject(root, "brew_state", brew_state_strings[brew_state]);
    
    /* Get brew progress */
    uint8_t progress = ui_get_brew_progress();
    cJSON_AddNumberToObject(root, "progress", progress);
    
    /* Get schedule info from settings */
    sys_param_t *params = settings_get_parameter();
    cJSON_AddNumberToObject(root, "schedule_hour", params->schedule_hour);
    cJSON_AddNumberToObject(root, "schedule_minute", params->schedule_minute);
    cJSON_AddNumberToObject(root, "schedule_target_temp", params->schedule_target_temp);
    
    cJSON_AddNumberToObject(root, "idle_position", settings_get_idle_position());
    
    char *json_str = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json_str);
    
    free(json_str);
    cJSON_Delete(root);
    
    return ESP_OK;
}

/* GET /api/tea/:index - Get tea settings */
static esp_err_t tea_get_handler(httpd_req_t *req)
{
    /* Get tea index from URI */
    const char *uri = req->uri;
    int tea_index = 0;
    const char *idx_str = strrchr(uri, '/');
    if (idx_str) {
        tea_index = atoi(idx_str + 1);
    }
    
    if (tea_index < 0 || tea_index >= MAX_TEA_TYPES) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid tea index");
        return ESP_FAIL;
    }
    
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "name", tea_names[tea_index]);
    cJSON_AddStringToObject(root, "color", tea_colors_hex[tea_index]);
    cJSON_AddNumberToObject(root, "temperature", settings_get_tea_temperature(tea_index));
    cJSON_AddNumberToObject(root, "infusion_time", settings_get_tea_infusion_time(tea_index));
    
    char *json_str = cJSON_Print(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json_str);
    
    free(json_str);
    cJSON_Delete(root);
    
    return ESP_OK;
}

/* POST /api/tea/:index - Set tea settings */
static esp_err_t tea_post_handler(httpd_req_t *req)
{
    /* Get tea index from URI */
    const char *uri = req->uri;
    int tea_index = 0;
    const char *idx_str = strrchr(uri, '/');
    if (idx_str) {
        tea_index = atoi(idx_str + 1);
    }
    
    if (tea_index < 0 || tea_index >= MAX_TEA_TYPES) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid tea index");
        return ESP_FAIL;
    }
    
    /* Read request body */
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *temp = cJSON_GetObjectItem(root, "temperature");
    cJSON *infusion = cJSON_GetObjectItem(root, "infusion_time");
    
    if (temp && cJSON_IsNumber(temp)) {
        settings_set_tea_temperature(tea_index, (uint8_t)temp->valueint);
    }
    
    if (infusion && cJSON_IsNumber(infusion)) {
        settings_set_tea_infusion_time(tea_index, (uint16_t)infusion->valueint);
    }
    
    cJSON_Delete(root);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
    
    return ESP_OK;
}

/* POST /api/brew - Start brewing */
static esp_err_t brew_post_handler(httpd_req_t *req)
{
    char buf[128];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *tea_idx = cJSON_GetObjectItem(root, "tea_index");
    if (tea_idx && cJSON_IsNumber(tea_idx)) {
        current_tea_index = (uint8_t)tea_idx->valueint;
    }
    
    cJSON_Delete(root);
    
    /* Start brewing via the same path as UI */
    ESP_LOGI(TAG, "Web: Starting brew for tea %d", current_tea_index);
    
    /* Call brewNow from ui_events - this will handle the full brewing sequence */
    brewNow(NULL);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
    
    return ESP_OK;
}

/* POST /api/brew/stop - Stop brewing */
static esp_err_t brew_stop_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Web: Stopping brew");
    
    /* Call stopBrewing from ui_events */
    stopBrewing(NULL);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
    
    return ESP_OK;
}

/* POST /api/schedule - Set schedule */
static esp_err_t schedule_post_handler(httpd_req_t *req)
{
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *hour = cJSON_GetObjectItem(root, "hour");
    cJSON *minute = cJSON_GetObjectItem(root, "minute");
    cJSON *target_temp = cJSON_GetObjectItem(root, "target_temp");
    cJSON *tea_idx = cJSON_GetObjectItem(root, "tea_index");
    
    if (!hour || !minute || !target_temp || !tea_idx) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing parameters");
        return ESP_FAIL;
    }
    
    current_tea_index = (uint8_t)tea_idx->valueint;
    
    ESP_LOGI(TAG, "Web: Setting schedule for %02d:%02d, target %d°C, tea %d",
             hour->valueint, minute->valueint, target_temp->valueint, current_tea_index);
    
    ui_start_schedule_brew((uint8_t)hour->valueint, (uint8_t)minute->valueint, (uint8_t)target_temp->valueint);
    
    cJSON_Delete(root);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
    
    return ESP_OK;
}

/* POST /api/schedule/cancel - Cancel schedule */
static esp_err_t schedule_cancel_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Web: Cancelling schedule");
    
    ui_cancel_schedule_brew();
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
    
    return ESP_OK;
}

/* POST /api/settings/idle_position - Set idle position */
static esp_err_t idle_position_handler(httpd_req_t *req)
{
    char buf[128];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *pos = cJSON_GetObjectItem(root, "position");
    if (pos && cJSON_IsNumber(pos)) {
        int position = pos->valueint;
        if (position >= 0 && position <= 100) {
            settings_set_idle_position(position);
            ESP_LOGI(TAG, "Web: Set idle position to %d%%", position);
        }
    }
    
    cJSON_Delete(root);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
    
    return ESP_OK;
}

/* GET / - Dashboard HTML */
static esp_err_t dashboard_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, DASHBOARD_HTML, sizeof(DASHBOARD_HTML) - 1);
    return ESP_OK;
}

/* ============================================
   SERVER INITIALIZATION
   ============================================ */

static const httpd_uri_t uri_dashboard = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = dashboard_get_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_status = {
    .uri = "/api/status",
    .method = HTTP_GET,
    .handler = status_get_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_tea_0 = { .uri = "/api/tea/0", .method = HTTP_GET, .handler = tea_get_handler, .user_ctx = NULL };
static const httpd_uri_t uri_tea_1 = { .uri = "/api/tea/1", .method = HTTP_GET, .handler = tea_get_handler, .user_ctx = NULL };
static const httpd_uri_t uri_tea_2 = { .uri = "/api/tea/2", .method = HTTP_GET, .handler = tea_get_handler, .user_ctx = NULL };
static const httpd_uri_t uri_tea_3 = { .uri = "/api/tea/3", .method = HTTP_GET, .handler = tea_get_handler, .user_ctx = NULL };
static const httpd_uri_t uri_tea_4 = { .uri = "/api/tea/4", .method = HTTP_GET, .handler = tea_get_handler, .user_ctx = NULL };
static const httpd_uri_t uri_tea_5 = { .uri = "/api/tea/5", .method = HTTP_GET, .handler = tea_get_handler, .user_ctx = NULL };

static const httpd_uri_t uri_tea_post_0 = { .uri = "/api/tea/0", .method = HTTP_POST, .handler = tea_post_handler, .user_ctx = NULL };
static const httpd_uri_t uri_tea_post_1 = { .uri = "/api/tea/1", .method = HTTP_POST, .handler = tea_post_handler, .user_ctx = NULL };
static const httpd_uri_t uri_tea_post_2 = { .uri = "/api/tea/2", .method = HTTP_POST, .handler = tea_post_handler, .user_ctx = NULL };
static const httpd_uri_t uri_tea_post_3 = { .uri = "/api/tea/3", .method = HTTP_POST, .handler = tea_post_handler, .user_ctx = NULL };
static const httpd_uri_t uri_tea_post_4 = { .uri = "/api/tea/4", .method = HTTP_POST, .handler = tea_post_handler, .user_ctx = NULL };
static const httpd_uri_t uri_tea_post_5 = { .uri = "/api/tea/5", .method = HTTP_POST, .handler = tea_post_handler, .user_ctx = NULL };

static const httpd_uri_t uri_brew = {
    .uri = "/api/brew",
    .method = HTTP_POST,
    .handler = brew_post_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_brew_stop = {
    .uri = "/api/brew/stop",
    .method = HTTP_POST,
    .handler = brew_stop_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_schedule = {
    .uri = "/api/schedule",
    .method = HTTP_POST,
    .handler = schedule_post_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_schedule_cancel = {
    .uri = "/api/schedule/cancel",
    .method = HTTP_POST,
    .handler = schedule_cancel_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_idle_position = {
    .uri = "/api/settings/idle_position",
    .method = HTTP_POST,
    .handler = idle_position_handler,
    .user_ctx = NULL
};

/* POST /api/motor/go_idle - Move motor to idle position */
static esp_err_t go_idle_handler(httpd_req_t *req)
{
    int idle_pos = settings_get_idle_position();
    ESP_LOGI(TAG, "Web: Moving motor to idle position: %d%%", idle_pos);
    uart_comm_move_to_percent((float)idle_pos);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
    
    return ESP_OK;
}

static const httpd_uri_t uri_go_idle = {
    .uri = "/api/motor/go_idle",
    .method = HTTP_POST,
    .handler = go_idle_handler,
    .user_ctx = NULL
};

/* POST /api/wifi/reset - Reset WiFi and restart to AP mode */
static esp_err_t wifi_reset_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Web: WiFi reset requested");
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\",\"message\":\"Restarting...\"}");
    
    /* Delay to allow response to be sent */
    vTaskDelay(pdMS_TO_TICKS(500));
    
    wifi_config_reset();
    esp_restart();
    
    return ESP_OK;
}

static const httpd_uri_t uri_wifi_reset = {
    .uri = "/api/wifi/reset",
    .method = HTTP_POST,
    .handler = wifi_reset_handler,
    .user_ctx = NULL
};

esp_err_t webserver_init(void)
{
    if (server != NULL) {
        ESP_LOGW(TAG, "Web server already running");
        return ESP_OK;
    }
    
    /* Initialize mDNS */
    esp_err_t mdns_ret = mdns_init();
    if (mdns_ret == ESP_OK) {
        mdns_hostname_set("teabrewer");
        mdns_instance_name_set("Tea Brewer Dashboard");
        mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
        ESP_LOGI(TAG, "mDNS initialized: teabrewer.local");
    } else {
        ESP_LOGW(TAG, "mDNS init failed: %s", esp_err_to_name(mdns_ret));
    }
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 25;
    config.stack_size = 8192;
    config.lru_purge_enable = true;
    
    ESP_LOGI(TAG, "Starting web server on port %d", config.server_port);
    
    esp_err_t ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start web server: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Register URI handlers */
    httpd_register_uri_handler(server, &uri_dashboard);
    httpd_register_uri_handler(server, &uri_status);
    
    httpd_register_uri_handler(server, &uri_tea_0);
    httpd_register_uri_handler(server, &uri_tea_1);
    httpd_register_uri_handler(server, &uri_tea_2);
    httpd_register_uri_handler(server, &uri_tea_3);
    httpd_register_uri_handler(server, &uri_tea_4);
    httpd_register_uri_handler(server, &uri_tea_5);
    
    httpd_register_uri_handler(server, &uri_tea_post_0);
    httpd_register_uri_handler(server, &uri_tea_post_1);
    httpd_register_uri_handler(server, &uri_tea_post_2);
    httpd_register_uri_handler(server, &uri_tea_post_3);
    httpd_register_uri_handler(server, &uri_tea_post_4);
    httpd_register_uri_handler(server, &uri_tea_post_5);
    
    httpd_register_uri_handler(server, &uri_brew);
    httpd_register_uri_handler(server, &uri_brew_stop);
    httpd_register_uri_handler(server, &uri_schedule);
    httpd_register_uri_handler(server, &uri_schedule_cancel);
    httpd_register_uri_handler(server, &uri_idle_position);
    httpd_register_uri_handler(server, &uri_go_idle);
    httpd_register_uri_handler(server, &uri_wifi_reset);
    
    ESP_LOGI(TAG, "Web server started successfully");
    
    return ESP_OK;
}

esp_err_t webserver_stop(void)
{
    if (server == NULL) {
        return ESP_OK;
    }
    
    esp_err_t ret = httpd_stop(server);
    server = NULL;
    
    ESP_LOGI(TAG, "Web server stopped");
    
    return ret;
}

bool webserver_is_running(void)
{
    return server != NULL;
}
