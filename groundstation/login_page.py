"""Login page for the ground station.

Kept as a Python string rather than a file on disk so PyInstaller bundles
it into the .exe automatically - a separate .html would have to be
declared as a data file and would go missing the first time someone
forgot. Styled to match the dashboard.
"""

LOGIN_HTML = """<!doctype html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SeaYou - Sign in</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body {
    margin: 0; min-height: 100vh; display: grid; place-items: center;
    background: #0a0f16; color: #e8eef5;
    font-family: ui-sans-serif, system-ui, "Segoe UI", Roboto, sans-serif;
  }
  .card {
    width: min(92vw, 360px); padding: 32px 28px; border-radius: 14px;
    background: rgba(255,255,255,.04); border: 1px solid rgba(255,255,255,.10);
    box-shadow: 0 24px 60px -24px rgba(0,0,0,.8);
  }
  h1 { margin: 0 0 2px; font-size: 30px; letter-spacing: -.5px; color: #FF6B35; }
  .sub {
    margin: 0 0 22px; font-size: 11px; letter-spacing: .16em;
    text-transform: uppercase; color: #8b98a8;
  }
  label { display: block; font-size: 10px; letter-spacing: .14em;
          text-transform: uppercase; color: #8b98a8; margin: 14px 0 5px; }
  input {
    width: 100%; padding: 10px 12px; border-radius: 7px; font-size: 14px;
    background: rgba(0,0,0,.45); border: 1px solid rgba(255,255,255,.14);
    color: #fff; font-family: inherit;
  }
  input:focus { outline: none; border-color: #FF6B35; }
  button {
    width: 100%; margin-top: 22px; padding: 11px; border: 0; border-radius: 7px;
    background: #FF6B35; color: #1a0d06; font-weight: 700; font-size: 14px;
    letter-spacing: .04em; cursor: pointer;
  }
  button:hover { background: #ff7d4d; }
  .err {
    margin: 14px 0 0; padding: 9px 11px; border-radius: 7px; font-size: 12px;
    background: rgba(255,59,48,.12); border: 1px solid rgba(255,59,48,.35);
    color: #ff8a80;
  }
  .note { margin: 20px 0 0; font-size: 11px; line-height: 1.5; color: #6f7d8d; }
</style>
</head><body>
  <form class="card" method="post" action="/login">
    <h1>SeaYou</h1>
    <p class="sub">Ground Station</p>
    <label for="u">Username</label>
    <input id="u" name="username" autocomplete="username" autofocus required>
    <label for="p">Password</label>
    <input id="p" name="password" type="password" autocomplete="current-password" required>
    <button type="submit">Sign in</button>
    <!--ERROR-->
    <p class="note">
      New accounts are <strong>viewers</strong> and cannot fly the aircraft.
      Ask the operator to grant pilot access.
    </p>
  </form>
</body></html>
"""
