String generateLoginPage() {
  return R"rawliteral(
  <!DOCTYPE html>
  <html lang='en'>
  <head>
  <meta charset='UTF-8'>
  <meta name='viewport' content='width=device-width, initial-scale=1.0'>
  <title>AeroNero - Login</title>
  <style>
    * {
      margin: 0;
      padding: 0;
      box-sizing: border-box;
    }
    
    body {
      font-family: 'Segoe UI', system-ui, -apple-system, sans-serif;
      background: linear-gradient(135deg, #f8fafc 0%, #e2e8f0 100%);
      min-height: 100vh;
      display: flex;
      align-items: center;
      justify-content: center;
      color: #1a202c;
    }
    
    .login-container {
      background: white;

      padding: 48px;
      border-radius: 12px;
      box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.1), 0 2px 4px -1px rgba(0, 0, 0, 0.06);
      width: 100%;
      max-width: 400px;
      border: 1px solid #e2e8f0;
    }
    
    .logo-container {
      text-align: center;
      
      margin-bottom: 32px;
    }
    
    .logo {
      margin-bottom: 16px;
      margin-left: 80px;
    }
    
    .tagline {
      color: #64748b;
      font-size: 14px;
      font-weight: 500;
    }
    
    .form-group {
      margin-bottom: 20px;
    }
    
    .form-group label {
      display: block;
      margin-bottom: 6px;
      color: #374151;
      font-weight: 500;
      font-size: 14px;
    }
    
    .form-group input {
      width: 100%;
      padding: 12px 16px;
      border: 1px solid #d1d5db;
      border-radius: 8px;
      font-size: 16px;
      transition: border-color 0.2s, box-shadow 0.2s;
      background: #fff;
    }
    
    .form-group input:focus {
      outline: none;
      border-color: #00a6e0;
      box-shadow: 0 0 0 3px rgba(0, 166, 224, 0.1);
    }
    
    .login-button {
      width: 100%;
      background: #00a6e0;
      color: white;
      padding: 14px;
      border: none;
      border-radius: 8px;
      font-size: 16px;
      font-weight: 600;
      cursor: pointer;
      transition: background-color 0.2s;
    }
    
    .login-button:hover {
      background: #0090c7;
    }
    
    .login-button:active {
      background: #007ba7;
    }
    
    @media (max-width: 480px) {
      .login-container {
        margin: 20px;
        padding: 32px 24px;
      }
    }
  </style>
  </head>
  <body>
  <div class='login-container'>
    <div class='logo-container'>
      <div class='logo'>
        <svg width="200" height="60" viewBox="0 0 400 120">
          <circle cx="80" cy="28" r="26" fill="none" stroke="#f3bf00" stroke-width="4"/>
          <circle cx="116" cy="28" r="26" fill="none" stroke="#00a6e0" stroke-width="4"/>
          <g transform="translate(60,18)" fill="none" stroke="#f3bf00" stroke-width="3" stroke-linecap="round">
             <path d="M2 2 C8 6, 20 -2, 26 2" />
            <path d="M2 10 C8 14, 20 6, 26 10" />
            <path d="M2 18 C8 22, 20 14, 26 18" />
           
          </g>
          <g transform="translate(108,18)" fill="none" stroke="#00a6e0" stroke-width="3" stroke-linecap="round">
            <path d="M2 2 C8 6, 20 -2, 26 2" />
            <path d="M2 10 C8 14, 20 6, 26 10" />
            <path d="M2 18 C8 22, 20 14, 26 18" />
          </g>
          <text x="14" y="86" font-family="Arial, Helvetica, sans-serif" font-weight="700" font-size="36" fill="#00a6e0">AERONERO</text>
          <text x="110" y="108" font-family="Arial, Helvetica, sans-serif" font-size="12" fill="#9aaec1" text-anchor="middle">life.water.</text>
        </svg>
      </div>
      <p class='tagline'>Configuration Portal</p>
    </div>
    <form action='/login' method='post'>
      <div class='form-group'>
        <label for='username'>Username</label>
        <input type='text' id='username' name='username' required>
      </div>
      <div class='form-group'>
        <label for='password'>Password</label>
        <input type='text' id='password' name='password' required>
      </div>
      <button type='submit'  class='login-button'>Login</button>
    </form>
  </div>
  </body>
  </html>
  )rawliteral";
}
String generateMainPage() {
  String html = R"rawliteral(
  <!DOCTYPE html>
  <html lang='en'>
  <head>
  <meta charset='UTF-8'>
  <meta name='viewport' content='width=device-width, initial-scale=1.0'>
  <title>AeroNero - Dashboard</title>
  <style>
    * {
      margin: 0;
      padding: 0;
      box-sizing: border-box;
    }
    
    body {
      font-family: 'Inter', 'Segoe UI', system-ui, -apple-system, sans-serif;
      background: linear-gradient(180deg, #ffffff 0%, #f7f8fc 100%);
      color: #1e293b;
      line-height: 1.5;
      font-size: 16px;
    }
    
    .header {
      background: #ffffff;
      border-bottom: 1px solid #e2e8f0;
      padding: 12px 0;
      position: sticky;
      top: 0;
      z-index: 100;
      box-shadow: 0 1px 3px rgba(0, 0, 0, 0.08);
    }
    
    .header-content {
      max-width: 1280px;
      margin: 0 auto;
      padding: 0 24px;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    
    .header-logo {
      display: flex;
      align-items: center;
      gap: 12px;
    }
    
    .header-info h1 {
      font-size: 22px;
      font-weight: 700;
      color: #1e293b;
    }
    
    .header-info p {
      font-size: 13px;
      color: #64748b;
      font-weight: 400;
    }
    
    .menu-toggle {
      display: none;
      background: none;
      border: none;
      font-size: 24px;
      cursor: pointer;
      padding: 8px;
      color: #1e293b;
      z-index: 1000;
    }
    
    .container {
      max-width: 1280px;
      margin: 0 auto;
      padding: 24px;
    }
    
    .nav-tabs {
      display: flex;
      gap: 8px;
      margin-bottom: 32px;
      background: #ffffff;
      padding: 8px;
      border-radius: 8px;
      border: 1px solid #e2e8f0;
    }
    
    .nav-tab {
      padding: 10px 20px;
      background: none;
      border: none;
      border-radius: 6px;
      cursor: pointer;
      font-weight: 600;
      font-size: 14px;
      color: #475569;
      transition: all 0.2s ease;
    }
    
    .nav-tab.active,
    .nav-tab:hover {
      background: #3b82f6;
      color: #ffffff;
      box-shadow: 0 1px 3px rgba(0, 0, 0, 0.1);
    }
    
    .tab-content {
      display: none;
    }
    
    .tab-content.active {
      display: block;
      animation: fadeIn 0.3s ease-in;
    }
    
    @keyframes fadeIn {
      from { opacity: 0; transform: translateY(8px); }
      to { opacity: 1; transform: translateY(0); }
    }
    
    .metrics-grid {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 24px;
      margin-bottom: 32px;
    }
    
    .metric-card {
      background: #ffffff;
      padding: 20px;
      border-radius: 10px;
      border: 1px solid #e5e7eb;
      position: relative;
      box-shadow: 0 2px 6px rgba(0, 0, 0, 0.06);
      transition: transform 0.2s ease, box-shadow 0.2s ease;
    }
    
    .metric-card:hover {
      transform: translateY(-3px);
      box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
    }
    
    .metric-card.valid::after {
      content: '';
      position: absolute;
      bottom: 0;
      left: 0;
      right: 0;
      height: 4px;
      background: #22c55e;
      border-radius: 0 0 10px 10px;
    }
    
    .metric-card.invalid::after {
      content: '';
      position: absolute;
      bottom: 0;
      left: 0;
      right: 0;
      height: 4px;
      background: #ef4444;
      border-radius: 0 0 10px 10px;
    }
    
    .metric-label {
      font-size: 13px;
      color: #64748b;
      font-weight: 600;
      margin-bottom: 8px;
      text-transform: uppercase;
      letter-spacing: 0.8px;
    }
    
    .metric-value {
      font-size: 26px;
      font-weight: 700;
      color: #1e293b;
      margin-bottom: 6px;
    }
    
    .metric-status {
      font-size: 12px;
      padding: 4px 10px;
      border-radius: 12px;
      font-weight: 600;
      display: inline-block;
    }
    
    .log-card {
      border-radius: 8px;
      box-shadow: 0 2px 6px rgba(0,0,0,0.1);
      overflow: hidden;
      font-family: monospace;
    }
    
    .log-card-header {
      background-color: #1a1a1a;
      color: #09ff00;
      padding: 10px 15px;
    }
    
    .log-card-title {
      margin: 0;
      font-size: 1.2rem;
    }
    
    .log-card-content.terminal-logs {
      background-color: #000;
      color: #09ff00;
      padding: 10px 15px;
      max-height: 300px;
      overflow-y: auto;
    }
    
    .log-line {
      margin-bottom: 5px;
    }
    
    .log-event {
      font-weight: bold;
    }
    
    .status-ok {
      background: #dcfce7;
      color: #15803d;
    }
    
    .status-error {
      background: #fee2e2;
      color: #b91c1c;
    }
    
    .card {
      background: #f8fafc;
      border-radius: 10px;
      border: 1px solid #e2e8f0;
      margin-bottom: 24px;
      box-shadow: 0 2px 6px rgba(0, 0, 0, 0.06);
    }
    
    .card-header {
      padding: 16px 20px;
      border-bottom: 1px solid #e2e8f0;
      background: #ffffff;
      border-radius: 10px 10px 0 0;
    }
    
    .card-title {
      font-size: 18px;
      font-weight: 700;
      color: #1e293b;
    }
    
    .card-content {
      padding: 20px;
    }
    
    .table {
      width: 100%;
      border-collapse: collapse;
    }
    
    .table th {
      text-align: left;
      padding: 12px 16px;
      background: #ffffff;
      font-weight: 600;
      font-size: 13px;
      color: #475569;
      border-bottom: 1px solid #e2e8f0;
    }
    
    .table td {
      padding: 12px 16px;
      border-bottom: 1px solid #f1f5f9;
      font-size: 14px;
      color: #374151;
    }
    
    .table tbody tr:hover {
      background: #f9fafb;
    }
    
    .form-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
      gap: 20px;
    }
    
    .form-group {
      margin-bottom: 16px;
    }
    
    .form-group label {
      display: block;
      margin-bottom: 6px;
      color: #374151;
      font-weight: 600;
      font-size: 13px;
    }
    
    .form-group input,
    .form-group select {
      width: 100%;
      padding: 10px 14px;
      border: 1px solid #d1d5db;
      border-radius: 6px;
      font-size: 14px;
      background: #ffffff;
      transition: all 0.2s ease;
    }
    
    .form-group input:focus,
    .form-group select:focus {
      outline: none;
      border-color: #3b82f6;
      box-shadow: 0 0 0 3px rgba(59, 130, 246, 0.1);
    }
    
    .btn {
      padding: 10px 20px;
      border: none;
      border-radius: 6px;
      font-weight: 600;
      cursor: pointer;
      transition: all 0.2s ease;
      font-size: 14px;
    }
    
    .btn-primary {
      background: #3b82f6;
      color: #ffffff;
    }
    
    .btn-primary:hover {
      background: #2563eb;
      transform: translateY(-1px);
      box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
    }
    
    .mobile-nav {
      display: none;
      position: fixed;
      top: 0;
      left: 0;
      right: 0;
      bottom: 0;
      background: rgba(0, 0, 0, 0.5);
      z-index: 300;
      opacity: 0;
      visibility: hidden;
      transition: opacity 0.3s ease, visibility 0.3s ease;
    }
    
    .mobile-nav.active {
      opacity: 1;
      visibility: visible;
    }
    
    .mobile-nav-content {
      position: absolute;
      top: 0;
      right: 0;
      background: #ffffff;
      height: 100vh;
      width: 280px;
      padding: 24px;
      transform: translateX(100%);
      transition: transform 0.3s ease;
      box-shadow: -2px 0 8px rgba(0, 0, 0, 0.1);
      overflow-y: auto;
    }
    
    .mobile-nav.active .mobile-nav-content {
      transform: translateX(0);
    }
    
    .mobile-nav-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 24px;
      padding-bottom: 16px;
      border-bottom: 1px solid #e2e8f0;
    }
    
    .close-btn {
      background: none;
      border: none;
      font-size: 24px;
      cursor: pointer;
      color: #475569;
      padding: 4px;
      width: 32px;
      height: 32px;
      display: flex;
      align-items: center;
      justify-content: center;
    }
    
    .mobile-nav-tabs {
      display: flex;
      flex-direction: column;
      gap: 8px;
    }
    
    .fw-card-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      background-color: #ffffff;
      color: #000000;
      padding: 10px 15px;
      border-radius: 8px 8px 0 0;
      font-family: monospace;
    }
    
    .fw-card-title {
      margin: 0;
      font-size: 1.2rem;
    }
    
    .fw-version {
      font-size: 1rem;
      color: #000000;
      font-weight: bold;
    }
    
    .mobile-nav-tab {
      padding: 12px 16px;
      background: none;
      border: none;
      border-radius: 6px;
      cursor: pointer;
      font-weight: 600;
      font-size: 14px;
      color: #475569;
      text-align: left;
      transition: all 0.2s ease;
      width: 100%;
    }
    
    .mobile-nav-tab.active,
    .mobile-nav-tab:hover {
      background: #3b82f6;
      color: #ffffff;
    }
    
    footer {
      align-items: center;
      margin: 20px;
      color: #475569;
      text-align: center;
    }
    
    @media (max-width: 768px) {
      .header-content {
        padding: 0 16px;
      }
      
      .menu-toggle {
        display: block;
      }
      
      .nav-tabs {
        display: none;
      }
      
      .mobile-nav {
        display: block;
      }
      
      .container {
        padding: 16px;
      }
      
      .metrics-grid {
        grid-template-columns: 1fr;
        gap: 16px;
      }
      
      .form-grid {
        grid-template-columns: 1fr;
        gap: 16px;
      }
      
      .card-header,
      .card-content {
        padding: 16px;
      }
      
      .table th,
      .table td {
        padding: 8px 12px;
        font-size: 13px;
      }
      
      .header-info h1 {
        font-size: 18px;
      }
      
      .header-info p {
        font-size: 12px;
      }
    }
    
    @media (max-width: 480px) {
      .header-content {
        padding: 0 12px;
      }
      
      .container {
        padding: 12px;
      }
      
      .metric-value {
        font-size: 22px;
      }
      
      .mobile-nav-content {
        width: 260px;
      }
    }
  </style>
  </head>
  <body>
  <header class='header'>
    <div class='header-content'>
      <div class='header-logo'>
        <svg width="200" height="60" viewBox="0 0 400 120">
          <circle cx="80" cy="28" r="26" fill="none" stroke="#f3bf00" stroke-width="4"/>
          <circle cx="116" cy="28" r="26" fill="none" stroke="#00a6e0" stroke-width="4"/>
          <g transform="translate(60,18)" fill="none" stroke="#f3bf00" stroke-width="3" stroke-linecap="round">
            <path d="M2 2 C8 6, 20 -2, 26 2" />
            <path d="M2 10 C8 14, 20 6, 26 10" />
            <path d="M2 18 C8 22, 20 14, 26 18" />
          </g>
          <g transform="translate(108,18)" fill="none" stroke="#00a6e0" stroke-width="3" stroke-linecap="round">
            <path d="M2 2 C8 6, 20 -2, 26 2" />
            <path d="M2 10 C8 14, 20 6, 26 10" />
            <path d="M2 18 C8 22, 20 14, 26 18" />
          </g>
          <text x="14" y="86" font-family="Arial, Helvetica, sans-serif" font-weight="700" font-size="36" fill="#00a6e0">AERONERO</text>
          <text x="110" y="108" font-family="Arial, Helvetica, sans-serif" font-size="12" fill="#9aaec1" text-anchor="middle">life.water.</text>
        </svg>
        <div class='header-info'>
          <h1>Configuration Portal</h1>
          <p>IoT Management System</p>
        </div>
      </div>
      <button class='menu-toggle' id='menuToggle'>☰</button>
    </div>
  </header>
  
  <div class='mobile-nav' id='mobileNav'>
    <div class='mobile-nav-content'>
      <div class='mobile-nav-header'>
        <h3>Menu</h3>
        <button class='close-btn' id='closeMobileNav'>×</button>
      </div>
      <div class='mobile-nav-tabs'>
        <button class='mobile-nav-tab active' data-tab='dashboard'>Dashboard</button>
        <button class='mobile-nav-tab' data-tab='communication'>Communication</button>
        <button class='mobile-nav-tab' data-tab='thresholds'>Thresholds</button>
        <button class='mobile-nav-tab' data-tab='ota'>OTA Update</button>
      </div>
    </div>
  </div>
  
  <div class='container'>
    <div class='nav-tabs'>
      <button class='nav-tab active' data-tab='dashboard'>Dashboard</button>
      <button class='nav-tab' data-tab='communication'>Communication</button>
      <button class='nav-tab' data-tab='thresholds'>Thresholds</button>
      <button class='nav-tab' data-tab='ota'>OTA Update</button>
    </div>
    
    <!-- Dashboard Tab -->
    <div id='dashboard' class='tab-content active'>
      <div class='metrics-grid'>
  )rawliteral";

  // Generate sensor cards with updated ranges
  float temperature = sht.readSample() ? sht.getTemperature() : NAN;
  float humidity    = sht.readSample() ? sht.getHumidity() : NAN;

  html += "<div class='metric-card " + String(isnan(temperature) || temperature == 0.0 || temperature < -40 || temperature > 125 ? "invalid" : "valid") + "'>";
  html += "<div class='metric-label'>Temperature</div>";
  html += "<div class='metric-value'>" + String(temperature, 1) + "°C</div>";
  html += "<span class='metric-status " + String(isnan(temperature) || temperature < -40 || temperature > 125 ? "status-error'>Out of Range" : "status-ok'>Normal") + "</span>";
  html += "</div>";
  
  html += "<div class='metric-card " + String(isnan(humidity) || humidity < 1 || humidity > 100 ? "invalid" : "valid") + "'>";
  html += "<div class='metric-label'>Humidity</div>";
  html += "<div class='metric-value'>" + String(humidity, 1) + "%</div>";
  html += "<span class='metric-status " + String(isnan(humidity) || humidity < 1 || humidity > 100 ? "status-error'>Out of Range" : "status-ok'>Normal") + "</span>";
  html += "</div>";
  
  html += "<div class='metric-card " + String(phValue < 1 || phValue > 14 ? "invalid" : "valid") + "'>";
  html += "<div class='metric-label'>pH Level</div>";
  html += "<div class='metric-value'>" + String(phValue, 2) + "</div>";
  html += "<span class='metric-status " + String(phValue < 1 || phValue > 14 ? "status-error'>Out of Range" : "status-ok'>Normal") + "</span>";
  html += "</div>";
  
  html += "<div class='metric-card " + String(tdsValue < 1 || tdsValue > 1000 ? "invalid" : "valid") + "'>";
  html += "<div class='metric-label'>TDS</div>";
  html += "<div class='metric-value'>" + String(tdsValue, 0) + " ppm</div>";
  html += "<span class='metric-status " + String(tdsValue < 1 || tdsValue > 1000 ? "status-error'>Out of Range" : "status-ok'>Normal") + "</span>";
  html += "</div>";
  
  html += "<div class='metric-card " + String(distance < 3 || distance > 500 ? "invalid" : "valid") + "'>";
  html += "<div class='metric-label'>Water Level</div>";
  html += "<div class='metric-value'>" + String(distance) + " cm</div>";
  html += "<span class='metric-status " + String(distance < 3 || distance > 500 ? "status-error'>Out of Range" : "status-ok'>Normal") + "</span>";
  html += "</div>";
  
  html += "<div class='metric-card " + String(totalLiters < 0 ? "invalid" : "valid") + "'>";
  html += "<div class='metric-label'>Total Volume</div>";
  html += "<div class='metric-value'>" + String(totalLiters, 2) + " L</div>";
  html += "<span class='metric-status " + String(totalLiters < 0 ? "status-error'>Out of Range" : "status-ok'>Normal") + "</span>";
  html += "</div>";
  
  html += "<div class='metric-card " + String(AQI < 1 || AQI > 5 ? "invalid" : "valid") + "'>";
  html += "<div class='metric-label'>AQI</div>";
  html += "<div class='metric-value'>" + String(AQI) + "</div>";
  html += "<span class='metric-status " + String(AQI < 1 || AQI > 5 ? "status-error'>Out of Range" : "status-ok'>Normal") + "</span>";
  html += "</div>";
  
  html += "<div class='metric-card " + String(TVOC < 1 || TVOC > 65000 ? "invalid" : "valid") + "'>";
  html += "<div class='metric-label'>TVOC</div>";
  html += "<div class='metric-value'>" + String(TVOC, 0) + " ppb</div>";
  html += "<span class='metric-status " + String(TVOC < 1 || TVOC > 65000 ? "status-error'>Out of Range" : "status-ok'>Normal") + "</span>";
  html += "</div>";
  
  html += "<div class='metric-card " + String(ECO2 < 400 || ECO2 > 65000 ? "invalid" : "valid") + "'>";
  html += "<div class='metric-label'>ECO2</div>";
  html += "<div class='metric-value'>" + String(ECO2, 0) + " ppm</div>";
  html += "<span class='metric-status " + String(ECO2 < 400 || ECO2 > 65000 ? "status-error'>Out of Range" : "status-ok'>Normal") + "</span>";
  html += "</div>";
  
  html += "</div>";
  
  // Recent Logs
  html += "<div class='log-card'>";
  html += "<div class='log-card-header'>";
  html += "<h2 class='log-card-title'>Recent Logs</h2>";
  html += "</div>";
  html += "<div class='log-card-content terminal-logs'>";
  for (int i = 0; i < MAX_LOGS && i < 10; i++) {
    int idx = (logIndex - 1 - i + MAX_LOGS) % MAX_LOGS;
    if (serialLogs[idx].length() > 0) {
      html += "<div class='log-line'><span class='log-event'>[" + String(millis() / 1000) + "s]</span> " + serialLogs[idx] + "</div>";
    }
  }
  html += "</div>";
  html += "</div>";
  html += "</div>";
  
  // Communication Tab
  html += "<div id='communication' class='tab-content'>";
  html += "<form action='/save' method='post'>";
  html += "<div class='card'>";
  html += "<div class='card-header'>";
  html += "<h2 class='card-title'>Device Configuration</h2>";
  html += "</div>";
  html += "<div class='card-content'>";
  html += "<div class='form-grid'>";
  
  html += "<div class='form-group'>";
  html += "<label for='deviceId'>Device ID</label>";
  html += "<input type='text' id='deviceId' name='deviceId' value='" + String(config.deviceId) + "' required>";
  html += "</div>";
  
  html += "<div class='form-group'>";
  html += "<label for='connectionType'>Connection Type</label>";
  html += "<select id='connectionType' name='type' onchange='updateConnectionType()' required>";
  html += "<option value='WiFi'" + String(strcmp(config.type, "WiFi") == 0 ? " selected" : "") + ">WiFi</option>";
  html += "<option value='GSM'" + String(strcmp(config.type, "GSM") == 0 ? " selected" : "") + ">GSM/Cellular</option>";
  html += "</select>";
  html += "</div>";
  
  html += "</div>";
  html += "</div>";
  html += "</div>";
  
  html += "<div id='wifiFields' class='card' style='display: " + String(strcmp(config.type, "WiFi") == 0 ? "block" : "none") + ";'>";
  html += "<div class='card-header'>";
  html += "<h2 class='card-title'>WiFi Settings</h2>";
  html += "</div>";
  html += "<div class='card-content'>";
  html += "<div class='form-grid'>";
  
  html += "<div class='form-group'>";
  html += "<label for='wifiSsid'>Network Name (SSID)</label>";
  html += "<input type='text' id='wifiSsid' name='wifi_ssid' value='" + String(config.wifi_ssid) + "'>";
  html += "</div>";
  
  html += "<div class='form-group'>";
  html += "<label for='wifiPassword'>Network Password</label>";
  html += "<input type='text' id='wifiPassword' name='wifi_password' value='" + String(config.wifi_password) + "'>";
  html += "</div>";
  
  html += "</div>";
  html += "</div>";
  html += "</div>";
  
  html += "<div id='gsmFields' class='card' style='display: " + String(strcmp(config.type, "GSM") == 0 ? "block" : "none") + ";'>";
  html += "<div class='card-header'>";
  html += "<h2 class='card-title'>GSM Settings</h2>";
  html += "</div>";
  html += "<div class='card-content'>";
  html += "<div class='form-grid'>";
  
  html += "<div class='form-group'>";
  html += "<label for='gsmUsername'>Username</label>";
  html += "<input type='text' id='gsmUsername' name='gsm_username' value='" + String(config.gsm_username) + "'>";
  html += "</div>";
  
  html += "<div class='form-group'>";
  html += "<label for='gsmPassword'>Password</label>";
  html += "<input type='text' id='gsmPassword' name='gsm_password' value='" + String(config.gsm_password) + "'>";
  html += "</div>";
  
  html += "<div class='form-group'>";
  html += "<label for='gsmApn'>APN</label>";
  html += "<input type='text' id='gsmApn' name='gsm_apn' value='" + String(config.gsm_apn) + "'>";
  html += "</div>";
  
  html += "</div>";
  html += "</div>";
  html += "</div>";
  
  html += "<div style='text-align: center; margin-top: 24px;'>";
  html += "<button type='submit' class='btn btn-primary'>Save Configuration</button>";
  html += "</div>";
  html += "</form>";
  html += "</div>";
  
  // Thresholds Tab
  html += "<div id='thresholds' class='tab-content'>";
  html += "<form action='/save_thresholds' method='post'>";
  html += "<div class='card'>";
  html += "<div class='card-header'>";
  html += "<h2 class='card-title'>System Thresholds</h2>";
  html += "</div>";
  html += "<div class='card-content'>";
  html += "<table class='table'>";
  html += "<thead>";
  html += "<tr><th>Parameter</th><th>Current</th><th>New Value</th><th>Unit</th></tr>";
  html += "</thead>";
  html += "<tbody>";
  
  html += "<tr>";
  html += "<td>Temperature Min</td>";
  html += "<td>" + String(config.tempThresholdMin, 1) + "</td>";
  html += "<td><input type='number' step='0.1' name='tempThresholdMin' value='" + String(config.tempThresholdMin, 1) + "' style='width: 100px; padding: 6px; border: 1px solid #d1d5db; border-radius: 4px;'></td>";
  html += "<td>°C</td>";
  html += "</tr>";
  
  html += "<tr>";
  html += "<td>Temperature Max</td>";
  html += "<td>" + String(config.tempThresholdMax, 1) + "</td>";
  html += "<td><input type='number' step='0.1' name='tempThresholdMax' value='" + String(config.tempThresholdMax, 1) + "' style='width: 100px; padding: 6px; border: 1px solid #d1d5db; border-radius: 4px;'></td>";
  html += "<td>°C</td>";
  html += "</tr>";
  
  html += "<tr>";
  html += "<td>Humidity Min</td>";
  html += "<td>" + String(config.humidityThresholdMin, 1) + "</td>";
  html += "<td><input type='number' step='0.1' name='humidityThresholdMin' value='" + String(config.humidityThresholdMin, 1) + "' style='width: 100px; padding: 6px; border: 1px solid #d1d5db; border-radius: 4px;'></td>";
  html += "<td>%</td>";
  html += "</tr>";
  
  html += "<tr>";
  html += "<td>Humidity Max</td>";
  html += "<td>" + String(config.humidityThresholdMax, 1) + "</td>";
  html += "<td><input type='number' step='0.1' name='humidityThresholdMax' value='" + String(config.humidityThresholdMax, 1) + "' style='width: 100px; padding: 6px; border: 1px solid #d1d5db; border-radius: 4px;'></td>";
  html += "<td>%</td>";
  html += "</tr>";
  
  html += "<tr>";
  html += "<td>Compressor On Level</td>";
  html += "<td>" + String(config.compressorOnLevel, 1) + "</td>";
  html += "<td><input type='number' step='0.1' name='compressorOnLevel' value='" + String(config.compressorOnLevel, 1) + "' style='width: 100px; padding: 6px; border: 1px solid #d1d5db; border-radius: 4px;'></td>";
  html += "<td>cm</td>";
  html += "</tr>";
    
  html += "<tr>";
  html += "<td>Compressor Off Level</td>";
  html += "<td>" + String(config.compressorOffLevel, 1) + "</td>";
  html += "<td><input type='number' step='0.1' name='compressorOffLevel' value='" + String(config.compressorOffLevel, 1) + "' style='width: 100px; padding: 6px; border: 1px solid #d1d5db; border-radius: 4px;'></td>";
  html += "<td>cm</td>";
  html += "</tr>";
  
  html += "<tr>";
  html += "<td>Pump Off Level</td>";
  html += "<td>" + String(config.pumpOffLevel, 1) + "</td>";
  html += "<td><input type='number' step='0.1' name='pumpOffLevel' value='" + String(config.pumpOffLevel, 1) + "' style='width: 100px; padding: 6px; border: 1px solid #d1d5db; border-radius: 4px;'></td>";
  html += "<td>cm</td>";
  html += "</tr>";
  
  html += "<tr>";
  html += "<td>Publish Interval</td>";
  html += "<td>" + String(config.publishInterval / 60000) + "</td>";
  html += "<td><input type='number' name='publishInterval' value='" + String(config.publishInterval / 60000) + "' min='1' max='1440' style='width: 100px; padding: 6px; border: 1px solid #d1d5db; border-radius: 4px;'></td>";
  html += "<td>min</td>";
  html += "</tr>";
  
  html += "</tbody>";
  html += "</table>";
  html += "</div>";
  html += "</div>";
  
  html += "<div style='text-align: center; margin-top: 24px;'>";
  html += "<button type='submit' class='btn btn-primary'>Update Thresholds</button>";
  html += "</div>";
  html += "</form>";
  html += "</div>";
  

  
  // OTA Update Tab
  html += "<div id='ota' class='tab-content'>";
  html += "<div class='card'>";
  html += "<div class='fw-card-header'>";
  html += "<h2 class='fw-card-title'>Firmware Information</h2>";
  html += "<span class='fw-version'>" + String(current_firmware_version) + "</span>";
  html += "</div>";
  html += "</div>";
  
  html += "<div class='card'>";
  html += "<div class='card-header'>";
  html += "<h2 class='card-title'>Firmware Update</h2>";
  html += "</div>";
  html += "<div class='card-content'>";
  html += "<form action='/ota_submit' method='post'>";
  html += "<div class='form-grid'>";
  
  html += "<div class='form-group'>";
  html += "<label for='otaSsid'>WiFi Network</label>";
  html += "<input type='text' id='otaSsid' name='ota_ssid'  value='" + String(config.last_ota_ssid) + "' required>";
  html += "</div>";
  
  html += "<div class='form-group'>";
  html += "<label for='otaPassword'>WiFi Password</label>";
  html += "<input type='text' id='otaPassword' name='ota_password'  value='" + String(config.last_ota_password) + "' required>";
  html += "</div>";
  
  html += "<div class='form-group' style='grid-column: 1 / -1;'>";
  html += "<label for='otaUrl'>Firmware URL</label>";
  html += "<input type='url' id='otaUrl' name='ota_url'  value='" + String(config.last_ota_link) + "' required>";
  html += "</div>";
  
  html += "</div>";
  
  html += "<div style='background: #fef3cd; border: 1px solid #facc15; border-radius: 6px; padding: 16px; margin: 20px 0; color: #92400e;'>";
  html += "<strong>⚠️ Warning:</strong> Device will restart during update. Ensure stable power and network.";
  html += "</div>";
  
  html += "<div style='text-align: center; margin-top: 24px;'>";
  html += "<button type='submit' class='btn btn-primary'>Start Update</button>";
  html += "</div>";
  html += "</form>";
  html += "</div>";
  html += "</div>";
  html += "</div>";
  
  html += "</div>";
  html += "<footer>&copy;2025 All Rights Reserved By Aeronero</footer>";
  
  html += "<script>";
  html += "let currentTab = 'dashboard';";
  html += "document.addEventListener('DOMContentLoaded', function() {";
  html += "  initializeEventListeners();";
  html += "  showTab('dashboard');";
  html += "  updateConnectionType();";
  html += "  setInterval(updateDashboardData, 5000);";
  html += "});";
  
  html += "function initializeEventListeners() {";
  html += "  const menuToggle = document.getElementById('menuToggle');";
  html += "  if (menuToggle) {";
  html += "    menuToggle.addEventListener('click', function(e) {";
  html += "      e.preventDefault();";
  html += "      e.stopPropagation();";
  html += "      toggleMobileNav();";
  html += "    });";
  html += "  }";
  
  html += "  const closeMobileNav = document.getElementById('closeMobileNav');";
  html += "  if (closeMobileNav) {";
  html += "    closeMobileNav.addEventListener('click', function(e) {";
  html += "      e.preventDefault();";
  html += "      e.stopPropagation();";
  html += "      closeMobileNavigation();";
  html += "    });";
  html += "  }";
  
  html += "  const mobileNav = document.getElementById('mobileNav');";
  html += "  if (mobileNav) {";
  html += "    mobileNav.addEventListener('click', function(e) {";
  html += "      if (e.target === this) {";
  html += "        closeMobileNavigation();";
  html += "      }";
  html += "    });";
  html += "  }";
  
  html += "  const navTabs = document.querySelectorAll('.nav-tab');";
  html += "  navTabs.forEach(tab => {";
  html += "    tab.addEventListener('click', function() {";
  html += "      const tabName = this.getAttribute('data-tab');";
  html += "      showTab(tabName);";
  html += "    });";
  html += "  });";
  
  html += "  const mobileNavTabs = document.querySelectorAll('.mobile-nav-tab');";
  html += "  mobileNavTabs.forEach(tab => {";
  html += "    tab.addEventListener('click', function() {";
  html += "      const tabName = this.getAttribute('data-tab');";
  html += "      showTab(tabName);";
  html += "      closeMobileNavigation();";
  html += "    });";
  html += "  });";
  html += "}";
  
  html += "function showTab(tabName) {";
  html += "  document.querySelectorAll('.tab-content').forEach(tab => {";
  html += "    tab.classList.remove('active');";
  html += "  });";
  html += "  document.querySelectorAll('.nav-tab, .mobile-nav-tab').forEach(link => {";
  html += "    link.classList.remove('active');";
  html += "  });";
  html += "  const selectedTab = document.getElementById(tabName);";
  html += "  if (selectedTab) {";
  html += "    selectedTab.classList.add('active');";
  html += "  }";
  html += "  document.querySelectorAll(`[data-tab=\"${tabName}\"]`).forEach(link => {";
  html += "    link.classList.add('active');";
  html += "  });";
  html += "  currentTab = tabName;";
  html += "}";
  
  html += "function toggleMobileNav() {";
  html += "  const nav = document.getElementById('mobileNav');";
  html += "  if (nav) {";
  html += "    nav.classList.toggle('active');";
  html += "    if (nav.classList.contains('active')) {";
  html += "      document.body.style.overflow = 'hidden';";
  html += "    } else {";
  html += "      document.body.style.overflow = '';";
  html += "    }";
  html += "  }";
  html += "}";
  
  html += "function closeMobileNavigation() {";
  html += "  const nav = document.getElementById('mobileNav');";
  html += "  if (nav) {";
  html += "    nav.classList.remove('active');";
  html += "    document.body.style.overflow = '';";
  html += "  }";
  html += "}";
  
  html += "function updateConnectionType() {";
  html += "  const type = document.getElementById('connectionType').value;";
  html += "  const wifiFields = document.getElementById('wifiFields');";
  html += "  const gsmFields = document.getElementById('gsmFields');";
  html += "  if (type === 'WiFi') {";
  html += "    wifiFields.style.display = 'block';";
  html += "    gsmFields.style.display = 'none';";
  html += "  } else {";
  html += "    wifiFields.style.display = 'none';";
  html += "    gsmFields.style.display = 'block';";
  html += "  }";
  html += "}";
  
  html += "function updateDashboardData() {";
  html += "  if (currentTab === 'dashboard') {";
  html += "    location.reload();"; // Simple refresh to get new sensor data
  html += "  }";
  html += "}";
  
  html += "document.addEventListener('keydown', function(e) {";
  html += "  if (e.key === 'Escape') {";
  html += "    closeMobileNavigation();";
  html += "  }";
  html += "});";
  
  html += "window.addEventListener('resize', function() {";
  html += "  if (window.innerWidth > 768) {";
  html += "    closeMobileNavigation();";
  html += "  }";
  html += "});";
  html += "</script>";
  
  html += "</body>";
  html += "</html>";
  
  return html;
}