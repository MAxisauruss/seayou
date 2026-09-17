import "./RescueResponse.css";
import { Page } from "../types";

interface RescueResponseProps {
  onNavigate: (page: Page) => void;
  activePage: Page;
}

export default function RescueResponse({ onNavigate, activePage }: RescueResponseProps) {
  return (
    <div className="rescue-response-page overflow-hidden">

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
            className={`mx-2 flex items-center gap-3 px-4 py-3 rounded-[30px] font-bold text-white shadow-[0_30px_10px_-20px_rgba(0,0,0,0.2)] transition-all duration-300  dashboard-btn${
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
            className={`text-on-surface-variant hover:text-on-surface flex items-center gap-3 px-4 py-3 mx-2 hover:bg-[#FF6B35]/5 transition-all ${
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

      {/* Main Content Canvas */}
      <main className="md:ml-64 pt-0 h-screen flex flex-col">
        {/* Top High Alert Header */}
        <section className="h-24 bg-error-container/40 backdrop-blur-md flex items-center justify-between px-8 border-b border-error/30 relative overflow-hidden">
          <div className="absolute inset-0 opacity-10 bg-[radial-gradient(circle_at_center,_var(--tw-gradient-stops))] from-error via-transparent to-transparent"></div>
          <div className="flex items-center gap-8 relative z-10">
            <div className="flex flex-col">
              <span className="font-label-caps text-label-caps text-on-error-container">TIME SINCE INCIDENT</span>
              <div className="font-telemetry-lg text-[32px] text-error font-bold tracking-widest">00:04:23.44</div>
            </div>
            <div className="h-12 w-px bg-white/10"></div>
            <div className="flex flex-col">
              <span className="font-label-caps text-label-caps text-on-error-container">NEAREST MED STATION</span>
              <div className="font-telemetry-lg text-telemetry-lg text-white">2.4 KM - STATION BLUE</div>
            </div>
          </div>
          <div className="flex items-center gap-4 relative z-10">
            <button className="bg-surface-container-high border border-white/10 px-6 py-3 rounded-lg flex items-center gap-2 hover:bg-white/5 transition-all">
              <span className="material-symbols-outlined text-secondary" data-icon="airplanemode_active">
                airplanemode_active
              </span>
              <span className="font-label-caps text-label-caps">DEPLOY BUOY</span>
            </button>
            <button className="bg-[#FF6B35] px-8 py-3 rounded-lg flex items-center gap-2 text-white font-bold shadow-[0_0_15px_rgba(255,107,53,0.5)] active:scale-95 transition-all">
              <span className="material-symbols-outlined" data-icon="medical_services">
                medical_services
              </span>
              <span className="font-label-caps text-label-caps">MEDICAL EVAC</span>
            </button>
          </div>
        </section>

        <div className="flex-1 flex overflow-hidden">
          {/* Tactical Map Center */}
          <section className="flex-1 relative bg-surface-container-lowest">
            <div className="absolute inset-0">
              <img
                alt="A dark, high-contrast tactical maritime map with coastlines, depth contours, and a digital grid overlay"
                className="w-full h-full object-cover opacity-40 mix-blend-luminosity"
                data-location="North Sea Rescue Zone"
                src="https://lh3.googleusercontent.com/aida-public/AB6AXuAcGSTHYN0pwR9jx4mXVFVCiQz9B7gcFRenRqYQCY9u9RcVtaItrYQCqHxwGGws-KxA7LiM3mwWFoD_3TlFbgHVBlFmMDbTlXGAQE7dvxOOzECTUXNkJ7YWt8OU1AXYRYAVEhrL8lg75J6MHqgH5SzFCLXtrYpfymDiwHskouLnMtLBJK9L-9xReCxDPQK1v0ZLMMZGsZoW_byCXQud-MrRhicQloKXrwDDBb2cm335qB376jbxzbmPaL3xm2F2Pg-bdjc8YPTYwxg"
              />
            </div>
            {/* Tactical Overlays */}
            <div className="absolute inset-0 p-6 pointer-events-none">
              <div className="glass-panel p-4 rounded-lg absolute bottom-6 left-6 pointer-events-auto border-secondary/30">
                <h4 className="font-label-caps text-label-caps text-secondary mb-2">TARGET COORDINATES</h4>
                <div className="font-telemetry-sm text-white">LAT: 52.3765° N</div>
                <div className="font-telemetry-sm text-white">LON: 4.8949° E</div>
                <div className="mt-3 flex items-center gap-2 text-error text-[10px] font-bold">
                  <span className="w-2 h-2 rounded-full bg-error animate-pulse"></span>
                  SIGNAL PERSISTENT
                </div>
              </div>
              {/* Asset Markers */}
              <div className="absolute top-1/2 left-1/3 transform -translate-x-1/2 -translate-y-1/2 flex flex-col items-center">
                <div className="w-12 h-12 rounded-full bg-secondary/20 border-2 border-secondary flex items-center justify-center animate-bounce">
                  <span className="material-symbols-outlined text-secondary text-xl" data-icon="person_pin_circle">
                    person_pin_circle
                  </span>
                </div>
                <span className="mt-2 font-label-caps text-label-caps bg-secondary text-on-secondary px-2 py-0.5 rounded">
                  TARGET
                </span>
              </div>
              <div className="absolute top-1/4 right-1/4 transform -translate-x-1/2 -translate-y-1/2 flex flex-col items-center">
                <div className="w-10 h-10 border-2 border-primary rounded-lg flex items-center justify-center rotate-45 bg-primary/10">
                  <span className="material-symbols-outlined text-primary -rotate-45" data-icon="helicopter">
                    helicopter
                  </span>
                </div>
                <span className="mt-2 font-label-caps text-label-caps bg-primary text-on-primary px-2 py-0.5 rounded">
                  RESCUE-01
                </span>
              </div>
            </div>
            {/* Floating Map Controls */}
            <div className="absolute top-6 right-6 flex flex-col gap-2 pointer-events-auto">
              <button className="w-10 h-10 glass-panel flex items-center justify-center hover:bg-white/10">
                <span className="material-symbols-outlined" data-icon="add">
                  add
                </span>
              </button>
              <button className="w-10 h-10 glass-panel flex items-center justify-center hover:bg-white/10">
                <span className="material-symbols-outlined" data-icon="remove">
                  remove
                </span>
              </button>
              <button className="w-10 h-10 glass-panel flex items-center justify-center hover:bg-white/10 mt-4">
                <span className="material-symbols-outlined" data-icon="layers">
                  layers
                </span>
              </button>
            </div>
          </section>

          {/* Right Sidebar: Vitals & Comms */}
          <aside className="w-96 border-l border-white/10 bg-surface-container-low/50 backdrop-blur-xl flex flex-col">
            {/* Live Feed Thumbnail */}
            <div className="p-4 border-b border-white/10">
              <div className="relative aspect-video rounded-lg overflow-hidden border border-secondary/50">
                <img
                  className="w-full h-full object-cover"
                  alt="A thermal camera view from a rescue drone flying over dark ocean waves, highlighting a person in the water"
                  src="https://lh3.googleusercontent.com/aida-public/AB6AXuBaA-V4E1ZmlVk85zC-axO-lqXvXP-FN8SuoItoTh4AGGFddvg-GG-nBJgbRkDywjb45gBwiuN6aFNjlhQipe5OCoLJ7icKqv5xsCsGDM0ssKOcizY5wdY-wN4FQjq6wS6clqJltc1aW1OelpcPy2lW_YL5G_fpazFcnJukS-L6AIzvMjnrp4j1uIMnEfpNtSM7nRNxMIWmc5LvvPhQ7nJnNoeE3hm3gTV9Kl9bwYyMEqzmNintqbFRTmTCanaSEbitRazaPpZCio8"
                />
                <div className="absolute top-2 left-2 flex items-center gap-2">
                  <span className="w-2 h-2 rounded-full bg-error animate-pulse"></span>
                  <span className="font-label-caps text-[10px] text-white bg-black/50 px-2 py-0.5 rounded">
                    DRONE-04 FEED
                  </span>
                </div>
              </div>
            </div>
            {/* Team Vitals */}
            <div className="flex-1 overflow-y-auto p-6 space-y-6">
              <div>
                <h4 className="font-label-caps text-label-caps text-primary mb-4">RESCUE TEAM VITALS</h4>
                <div className="space-y-4">
                  <div className="glass-panel p-4 rounded-lg border-l-4 border-primary">
                    <div className="flex justify-between items-start mb-2">
                      <span className="font-body-md font-bold text-white">LIFETEK-01 (JET SKI)</span>
                      <span className="font-telemetry-sm text-primary">ONLINE</span>
                    </div>
                    <div className="grid grid-cols-2 gap-4">
                      <div>
                        <span className="text-[10px] text-on-surface-variant font-label-caps block">HR</span>
                        <span className="font-telemetry-lg text-white">
                          88 <span className="text-[12px] opacity-50">BPM</span>
                        </span>
                      </div>
                      <div>
                        <span className="text-[10px] text-on-surface-variant font-label-caps block">O2 SAT</span>
                        <span className="font-telemetry-lg text-white">
                          99 <span className="text-[12px] opacity-50">%</span>
                        </span>
                      </div>
                    </div>
                  </div>
                  <div className="glass-panel p-4 rounded-lg border-l-4 border-secondary">
                    <div className="flex justify-between items-start mb-2">
                      <span className="font-body-md font-bold text-white">DIVETEAM-02</span>
                      <span className="font-telemetry-sm text-secondary">ACTIVE</span>
                    </div>
                    <div className="grid grid-cols-2 gap-4">
                      <div>
                        <span className="text-[10px] text-on-surface-variant font-label-caps block">HR</span>
                        <span className="font-telemetry-lg text-white">
                          142 <span className="text-[12px] opacity-50 text-secondary">BPM</span>
                        </span>
                      </div>
                      <div>
                        <span className="text-[10px] text-on-surface-variant font-label-caps block">DEPTH</span>
                        <span className="font-telemetry-lg text-white">
                          3.4 <span className="text-[12px] opacity-50">M</span>
                        </span>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
              {/* Comms Log */}
              <div>
                <h4 className="font-label-caps text-label-caps text-primary mb-4">TACTICAL COMMS</h4>
                <div className="space-y-3">
                  <div className="text-[12px]">
                    <span className="text-primary font-bold">[14:02:11] </span>
                    <span className="text-on-surface-variant">LIFETEK-01: Arriving at LZ-Beta. Visual contact confirmed.</span>
                  </div>
                  <div className="text-[12px]">
                    <span className="text-secondary font-bold">[14:02:45] </span>
                    <span className="text-on-surface-variant">COMMAND: Deploying flotation buoy. Stabilize target.</span>
                  </div>
                  <div className="text-[12px] bg-secondary/10 p-2 border-l border-secondary">
                    <span className="text-secondary font-bold">[14:03:02] </span>
                    <span className="text-white">DIVETEAM-02: Target reached. Commencing medical assessment.</span>
                  </div>
                </div>
              </div>
            </div>
            {/* Comms Input */}
            <div className="p-4 border-t border-white/10">
              <div className="relative">
                <input
                  className="w-full bg-[#050B14] border border-white/10 text-[12px] py-3 pl-4 pr-12 rounded-lg focus:border-tertiary focus:ring-1 focus:ring-tertiary outline-none transition-all placeholder:text-white/20 font-telemetry-sm"
                  placeholder="SEND TACTICAL INSTRUCTION..."
                  type="text"
                />
                <button className="absolute right-3 top-1/2 -translate-y-1/2 text-primary hover:text-white transition-colors">
                  <span className="material-symbols-outlined" data-icon="send">
                    send
                  </span>
                </button>
              </div>
            </div>
          </aside>
        </div>
      </main>

    </div>
  );
}
