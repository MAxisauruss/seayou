import "./AssetMap.css";
import { Page } from "../types";
import DroneMap from "../components/DroneMap";

interface AssetMapProps {
  onNavigate: (page: Page) => void;
  activePage: Page;
}

export default function AssetMap({ onNavigate, activePage }: AssetMapProps) {
  return (
    <div className="asset-map-page font-body-md text-body-md">
     {/* Top Logo */}
      <header className="fixed top-0 left-5 w-full z-50 flex items-center">
        <div className="flex items-center gap-4">

          <h1 className="font-display-lg text-display-lg-mobile md:text-display-lg text-primary tracking-tight">SeaYou</h1>
        </div>
        
      </header>

      {/* Side Navigation Bar */}
      <nav className="hidden md:flex flex-col h-full py-6 gap-4 fixed left-0 top-0 w-64 bg-surface-container-low/80 backdrop-blur-2xl border-r border-white/10 shadow-2xl z-40 pt-20">
        <div className="px-6 mb-4">
          <div className="flex items-center gap-3 mb-2">
            <div className="w-10 h-10 rounded-full bg-secondary-container flex items-center justify-center">
              <span className="material-symbols-outlined text-white" data-icon="person">
                person
              </span>
            </div>
            <div>
              <h3 className="font-headline-md text-[14px] text-primary">Sector Alpha</h3>
              <p className="font-label-caps text-label-caps text-on-surface-variant">Active Ops</p>
            </div>
          </div>
        </div>
        <div className="flex flex-col gap-1">
          <button
            className={`mx-2 flex items-center gap-3 px-4 py-3 rounded-[30px] font-bold text-white shadow-[0_30px_10px_-20px_rgba(0,0,0,0.2)] transition-all duration-300 ${
              activePage === "rescue-response"
                ? "bg-[length:300%] hover:bg-[length:320%] bg-left hover:bg-right"
                : "bg-transparent hover:bg-white/5"
            }`}
            style={
              activePage === "rescue-response"
                ? {
                    background: "linear-gradient(15deg, #de6f3d, #f09f33, #de6f3d)",
                    textShadow: "2px 2px 3px rgb(255, 132, 0)",
                  }
                : undefined
            }
            onClick={() => onNavigate?.("rescue-response")}
          >
            <span className="material-symbols-outlined" data-icon="dashboard" style={{ color: "#fbfbfb", transition: "0.3s ease" }}>
              dashboard
            </span>
            <span className="font-label-caps text-label-caps" style={{ color: "#ffffff", transition: "0.3s ease" }}>
              Dashboard
            </span>
          </button>

          <button
            className={`text-on-surface-variant hover:text-on-surface flex items-center gap-3 px-4 py-3 mx-2 hover:bg-[#FF6B35]/5 transition-all dashboard-btn ${
              activePage === "asset-map" ? "bg-[#FF6B35]/10 rounded-lg" : ""
            }`}
            onClick={() => onNavigate?.("asset-map")}
          >
            <span className="material-symbols-outlined" data-icon="explore">
              explore
            </span>
            <span className="font-label-caps text-label-caps">Asset Map</span>
          </button>
          <button
            className={`text-on-surface-variant hover:text-on-surface flex items-center gap-3 px-4 py-3 mx-2 hover:bg-white/5 transition-all ${
              activePage === "live-feeds" ? "bg-[#FF6B35]/10 rounded-lg" : ""
            }`}
            onClick={() => onNavigate?.("live-feeds")}
          >
            <span className="material-symbols-outlined" data-icon="videocam">
              videocam
            </span>
            <span className="font-label-caps text-label-caps">Live Feeds</span>
          </button>
          <button
            className={`text-on-surface-variant hover:text-on-surface flex items-center gap-3 px-4 py-3 mx-2 hover:bg-white/5 transition-all ${
              activePage === "rescue-logs" ? "bg-[#FF6B35]/10 rounded-lg" : ""
            }`}
            onClick={() => onNavigate?.("rescue-logs")}
          >
            <span
              className="material-symbols-outlined"
              data-icon="history"
              data-weight="fill"
              style={{ fontVariationSettings: "'FILL' 1" }}
            >
              history
            </span>
            <span className="font-label-caps text-label-caps">Rescue Logs</span>
          </button>
          <button
            className="text-on-surface-variant hover:text-on-surface flex items-center gap-3 px-4 py-3 mx-2 hover:bg-white/5 transition-all"
            onClick={() => onNavigate?.("rescue-logs")}
          >
            <span className="material-symbols-outlined" data-icon="bar_chart">
              bar_chart
            </span>
            <span className="font-label-caps text-label-caps">Analytics</span>
          </button>
        </div>
        <div className="mt-auto px-4 pb-4">
          <button className="w-full py-4 bg-secondary-container text-white font-bold rounded-lg emergency-glow border border-secondary pulse-emergency flex items-center justify-center gap-2 active:scale-95 transition-transform">
            <span className="material-symbols-outlined" data-icon="warning">
              warning
            </span>
            SOS EMERGENCY
          </button>
          <div className="mt-6 flex flex-col gap-2">
            <a className="flex items-center gap-3 px-4 py-2 text-on-surface-variant text-[12px] hover:text-on-surface" href="#">
              <span className="material-symbols-outlined text-[18px]" data-icon="help">
                help
              </span>
              Support
            </a>
            <a className="flex items-center gap-3 px-4 py-2 text-on-surface-variant text-[12px] hover:text-on-surface" href="#">
              <span className="material-symbols-outlined text-[18px]" data-icon="dns">
                dns
              </span>
              System Status
            </a>
          </div>
        </div>
      </nav>

      {/* Main Content Area */}
      <main className="md:ml-64 pt-16 h-screen relative flex flex-col">
        {/* Live drone map. This was a still image of a nautical chart -
            it is now the real thing, driven by the GPS dongle on the Pi.
            The overlay panels below are still design mock-ups. */}
        <div className="absolute inset-0 z-0 bg-[#050B14]">
          {/* The floats below are drawn OVER this map, so tell it which
              edges are hidden - otherwise "follow" parks the drone dead
              centre, behind the drift-prediction panel. Left/right are
              the w-80 panels plus their 8-unit offset; the bottom is the
              projection timeline. */}
          <DroneMap insets={{ top: 0, right: 352, bottom: 190, left: 352 }} />
        </div>

        {/* Left Float: Search Inputs */}
        <div className="absolute top-24 left-8 z-10 w-80 space-y-4">
          <div className="glass-panel p-6 rounded-xl">
            <h2 className="font-label-caps text-label-caps text-tertiary mb-4">DRIFT PREDICTION PARAMETERS</h2>
            <div className="space-y-4">
              <div>
                <label className="font-label-caps text-[10px] text-on-surface-variant mb-1 block">
                  LAST SEEN LOCATION (LAT/LONG)
                </label>
                <div className="flex gap-2">
                  <input
                    className="w-1/2 bg-surface-container-lowest border border-white/10 rounded px-3 py-2 text-telemetry-sm font-telemetry-sm text-primary focus:border-tertiary outline-none transition-colors"
                    placeholder="52.3702° N"
                    type="text"
                  />
                  <input
                    className="w-1/2 bg-surface-container-lowest border border-white/10 rounded px-3 py-2 text-telemetry-sm font-telemetry-sm text-primary focus:border-tertiary outline-none transition-colors"
                    placeholder="4.8952° E"
                    type="text"
                  />
                </div>
              </div>
              <div>
                <label className="font-label-caps text-[10px] text-on-surface-variant mb-1 block">ASSET TYPE</label>
                <select className="w-full bg-surface-container-lowest border border-white/10 rounded px-3 py-2 text-telemetry-sm font-telemetry-sm text-on-surface focus:border-tertiary outline-none">
                  <option>Life Raft (Low Profile)</option>
                  <option>Small Vessel (&lt; 10m)</option>
                  <option>Person Overboard</option>
                </select>
              </div>
              <button className="w-full py-2 bg-primary text-on-primary font-label-caps text-label-caps rounded hover:bg-primary-fixed transition-all active:scale-95">
                GENERATE HEATMAP
              </button>
            </div>
          </div>
          <div className="glass-panel p-4 rounded-xl">
            <div className="flex justify-between items-center mb-4">
              <h3 className="font-label-caps text-label-caps text-on-surface">LEGEND</h3>
            </div>
            <div className="space-y-2">
              <div className="flex items-center gap-3">
                <div className="w-3 h-3 rounded-full bg-[#FF6B35] shadow-[0_0_8px_rgba(255,107,53,0.8)]"></div>
                <span className="text-[10px] font-label-caps uppercase text-on-surface-variant">Predicted Search Zone</span>
              </div>
              <div className="flex items-center gap-3">
                <div className="w-3 h-3 rounded-full bg-tertiary shadow-[0_0_8px_rgba(233,196,0,0.8)]"></div>
                <span className="text-[10px] font-label-caps uppercase text-on-surface-variant">Last Known Position</span>
              </div>
              <div className="flex items-center gap-3">
                <div className="h-0.5 w-6 bg-primary opacity-50 relative">
                  <div className="absolute right-0 top-1/2 -translate-y-1/2 w-1.5 h-1.5 border-t border-r border-primary rotate-45"></div>
                </div>
                <span className="text-[10px] font-label-caps uppercase text-on-surface-variant">Current Vectors</span>
              </div>
            </div>
          </div>
        </div>

        {/* Right Float: Environmental Sidebar */}
        <div className="absolute top-24 right-8 z-10 w-80 space-y-4">
          <div className="glass-panel p-6 rounded-xl border-t-2 border-primary/30">
            <h2 className="font-label-caps text-label-caps text-tertiary mb-6">ENVIRONMENTAL REAL-TIME</h2>
            <div className="grid grid-cols-1 gap-6">
              <div className="flex items-center justify-between">
                <div>
                  <p className="font-label-caps text-label-caps text-on-surface-variant">TIDE HEIGHT</p>
                  <p className="font-telemetry-lg text-telemetry-lg text-white">+2.45 M</p>
                </div>
                <span className="material-symbols-outlined text-primary text-3xl">water_damage</span>
              </div>
              <div className="h-[1px] bg-white/5"></div>
              <div className="flex items-center justify-between">
                <div>
                  <p className="font-label-caps text-label-caps text-on-surface-variant">SWELL DIRECTION</p>
                  <p className="font-telemetry-lg text-telemetry-lg text-white">284° WNW</p>
                </div>
                <div className="w-10 h-10 border border-primary/20 rounded-full flex items-center justify-center">
                  <span className="material-symbols-outlined text-primary rotate-[284deg]">navigation</span>
                </div>
              </div>
              <div className="h-[1px] bg-white/5"></div>
              <div className="flex items-center justify-between">
                <div>
                  <p className="font-label-caps text-label-caps text-on-surface-variant">WIND VELOCITY</p>
                  <div className="flex items-baseline gap-2">
                    <p className="font-telemetry-lg text-telemetry-lg text-white">18.4</p>
                    <p className="text-xs font-telemetry-sm text-on-surface-variant">KNOTS</p>
                  </div>
                </div>
                <span className="material-symbols-outlined text-primary text-3xl">air</span>
              </div>
            </div>
          </div>
          {/* Weather Alert */}
          <div className="glass-panel p-4 rounded-xl border border-error/30 bg-error-container/10">
            <div className="flex gap-3">
              <span className="material-symbols-outlined text-error" style={{ fontVariationSettings: "'FILL' 1" }}>
                warning
              </span>
              <div>
                <p className="font-label-caps text-label-caps text-error">SMALL CRAFT ADVISORY</p>
                <p className="text-[11px] text-on-error-container leading-tight mt-1">
                  High swell predicted for Sector Alpha within T+4 hours. Exercise extreme caution.
                </p>
              </div>
            </div>
          </div>
        </div>

        {/* Bottom Controller: Time Slider */}
        <div className="absolute bottom-12 left-1/2 -translate-x-1/2 z-20 w-full max-w-4xl px-8">
          <div className="glass-panel p-6 rounded-2xl border-t border-white/20 shadow-2xl">
            <div className="flex items-center justify-between mb-4">
              <div className="flex items-center gap-4">
                <button className="w-10 h-10 rounded-full bg-white/5 flex items-center justify-center hover:bg-white/10 active:scale-90 transition-all">
                  <span className="material-symbols-outlined text-white">play_arrow</span>
                </button>
                <div>
                  <p className="font-label-caps text-label-caps text-on-surface">PROJECTION TIMELINE</p>
                  <p className="font-telemetry-sm text-telemetry-sm text-primary">T + 12:45:00</p>
                </div>
              </div>
              <div className="flex gap-2">
                <span className="px-3 py-1 bg-primary-container text-primary rounded text-[10px] font-bold border border-primary/20">
                  LIVE VIEW
                </span>
                <span className="px-3 py-1 bg-surface-variant text-on-surface-variant rounded text-[10px] font-bold">
                  PREDICTIVE
                </span>
              </div>
            </div>
            <div className="relative h-12 flex items-center">
              <div className="absolute w-full h-1 bg-white/10 rounded-full"></div>
              <div className="absolute w-1/2 h-1 bg-primary rounded-full shadow-[0_0_10px_rgba(185,199,228,0.5)]"></div>
              <div className="absolute inset-0 flex justify-between px-1 items-end pb-1">
                <span className="text-[9px] font-telemetry-sm text-on-surface-variant">T+0</span>
                <span className="text-[9px] font-telemetry-sm text-on-surface-variant">T+6H</span>
                <span className="text-[9px] font-telemetry-sm text-primary font-bold">T+12H</span>
                <span className="text-[9px] font-telemetry-sm text-on-surface-variant">T+18H</span>
                <span className="text-[9px] font-telemetry-sm text-on-surface-variant">T+24H</span>
                <span className="text-[9px] font-telemetry-sm text-on-surface-variant">T+48H</span>
              </div>
              <div className="absolute left-1/2 -translate-x-1/2 w-5 h-5 bg-white rounded-full shadow-[0_0_15px_rgba(255,255,255,0.6)] cursor-pointer active:scale-125 transition-transform border-2 border-primary"></div>
            </div>
          </div>
        </div>

        {/* The mock-up had a hardcoded CURSOR LAT / LONG / DEPTH readout
            here. Removed: the map below it is now real, and invented
            coordinates sitting on a real map are indistinguishable from
            live ones. DroneMap's own status strip shows the true
            position, or says plainly that there is no fix. */}
      </main>

      {/* Mobile BottomNavBar */}
      <nav className="md:hidden fixed bottom-0 left-0 w-full bg-surface-container-low/80 backdrop-blur-2xl flex justify-around items-center h-16 border-t border-white/10 z-50">
        <a className="flex flex-col items-center text-on-surface-variant" href="#">
          <span className="material-symbols-outlined">dashboard</span>
          <span className="text-[10px] font-label-caps">Home</span>
        </a>
        <a className="flex flex-col items-center text-primary" href="#">
          <span className="material-symbols-outlined" style={{ fontVariationSettings: "'FILL' 1" }}>
            explore
          </span>
          <span className="text-[10px] font-label-caps">Predict</span>
        </a>
        <a className="flex flex-col items-center text-on-surface-variant" href="#">
          <span className="material-symbols-outlined">videocam</span>
          <span className="text-[10px] font-label-caps">Feeds</span>
        </a>
        <a className="flex flex-col items-center text-on-surface-variant" href="#">
          <span className="material-symbols-outlined">bar_chart</span>
          <span className="text-[10px] font-label-caps">Data</span>
        </a>
      </nav>
    </div>
  );
}
