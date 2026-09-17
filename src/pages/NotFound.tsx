import "./NotFound.css";
import { Page } from "../types";

interface NotFoundProps {
  onNavigate: (page: Page) => void;
  activePage: Page;
}

const STATS = [
  { label: "LATITUDE", value: "--.------°" },
  { label: "LONGITUDE", value: "---.------°" },
  { label: "P_SIGNAL", value: "LOST_SYNC", danger: true },
  { label: "UPTIME", value: "00:00:00" },
];

const BREADCRUMB = ["SYSTEM_ROOT", "ERROR_LOGS", "ERR_404_SIGNAL_LOST"];

export default function NotFound({ onNavigate }: NotFoundProps) {
  return (
    <div className="not-found-page relative min-h-screen overflow-hidden bg-background text-on-surface">
      {/* Decorative frame glow - purely visual, sits above the background, below content */}
      <div className="frame-glow pointer-events-none fixed inset-0 z-10" />

      {/* Tiled ghost watermark */}
      <div className="ghost-grid pointer-events-none absolute inset-0 z-0 overflow-hidden" aria-hidden="true">
        {Array.from({ length: 6 }).map((_, row) => (
          <div className="ghost-row" key={row}>
            {Array.from({ length: 4 }).map((_, col) => (
              <span key={col}>
                404 ERROR<small>SIGNAL LOST »«</small>
              </span>
            ))}
          </div>
        ))}
      </div>

      {/* Radial warning glow behind the headline */}
      <div className="center-glow pointer-events-none absolute left-1/2 top-40 z-0 -translate-x-1/2" aria-hidden="true" />

      {/* Accent rail, left edge */}
      <div className="side-rail pointer-events-none fixed left-0 top-1/3 z-10" aria-hidden="true">
        <span className="side-rail-hot" />
      </div>

      {/* Header */}
      <header className="relative z-20 flex items-center justify-between border-b border-white/10 bg-surface-container-low/80 px-5 py-4 backdrop-blur-2xl md:px-8">
        <div className="flex items-center gap-4">
          <button
            type="button"
            aria-label="Open menu"
            className="text-on-surface-variant transition-colors hover:text-on-surface"
          >
            <span className="material-symbols-outlined">menu</span>
          </button>
          <span className="font-display-lg text-[20px] font-bold tracking-tight text-primary">SeaYou</span>
        </div>
        <nav className="hidden items-center gap-6 md:flex">
          <span className="font-label-caps text-label-caps text-on-surface-variant">SECTOR ALPHA</span>
          <span className="font-label-caps text-label-caps text-on-surface-variant">NETWORK STATUS</span>
        </nav>
        <div className="flex items-center gap-4">
          <button type="button" aria-label="Notifications" className="text-on-surface-variant hover:text-on-surface">
            <span className="material-symbols-outlined">notifications</span>
          </button>
          <button type="button" aria-label="Settings" className="text-on-surface-variant hover:text-on-surface">
            <span className="material-symbols-outlined">settings</span>
          </button>
        </div>
      </header>

      {/* Main content */}
      <main className="relative z-20 mx-auto flex max-w-2xl flex-col items-center px-6 pb-16 pt-16 text-center md:pt-24">
        <nav className="mb-10 flex items-center gap-1 font-telemetry-sm text-telemetry-sm text-on-surface-variant" aria-label="Breadcrumb">
          {BREADCRUMB.map((crumb, i) => (
            <span key={crumb} className="flex items-center gap-1">
              <span className={i === BREADCRUMB.length - 1 ? "text-[#FF6B35]" : undefined}>{crumb}</span>
              {i < BREADCRUMB.length - 1 && (
                <span className="material-symbols-outlined text-[14px] text-on-surface-variant/60">chevron_right</span>
              )}
            </span>
          ))}
        </nav>

        <h1 className="error-code select-none font-display-lg text-[110px] leading-none tracking-tight text-on-surface md:text-[160px]">
          404
        </h1>

        <h2 className="mt-6 font-headline-md text-[26px] font-bold uppercase tracking-wide text-[#FF6B35] md:text-[28px]">
          Connection Terminated
        </h2>

        <p className="mt-4 max-w-md font-body-md text-body-md text-on-surface-variant">
          A packet loss anomaly has occurred. The requested asset fragment or telemetry data cannot be located within
          Sector Alpha's current transmission range.
        </p>

        <div className="mt-8 grid w-full grid-cols-2 gap-3 md:grid-cols-4">
          {STATS.map((stat) => (
            <div key={stat.label} className="stat-box rounded border border-white/10 bg-surface-container px-4 py-3 text-left">
              <p className="font-label-caps text-label-caps text-on-surface-variant/70">{stat.label}</p>
              <p
                className={`mt-1 font-telemetry-sm text-telemetry-sm ${
                  stat.danger ? "text-[#FF6B35]" : "text-on-surface"
                }`}
              >
                {stat.value}
              </p>
            </div>
          ))}
        </div>

        <div className="mt-10 flex w-full flex-col gap-3 sm:w-auto sm:flex-row">
          <button
            type="button"
            onClick={() => onNavigate("rescue-response")}
            className="dashboard-btn flex items-center justify-center gap-2 px-6 py-3 font-label-caps text-label-caps"
          >
            <span className="material-symbols-outlined text-[18px]">grid_view</span>
            Return to Command Center
          </button>
          <button
            type="button"
            onClick={() => onNavigate("live-feeds")}
            className="flex items-center justify-center gap-2 rounded border border-white/15 bg-surface-container px-6 py-3 font-label-caps text-label-caps text-on-surface transition-colors hover:bg-white/5"
          >
            <span className="material-symbols-outlined text-[18px]">history</span>
            Check Local Cache
          </button>
        </div>
      </main>

      {/* Footer status strip */}
      <footer className="relative z-20 flex flex-col gap-4 px-6 pb-6 sm:flex-row sm:items-end sm:justify-between md:px-8">
        <div className="flex flex-col gap-1 font-telemetry-sm text-telemetry-sm">
          <span className="flex items-center gap-2 text-[#FF6B35]/90">
            <span className="status-dot" />
            UPLINK_FAILURE_DETECTED
          </span>
          <span className="text-on-surface-variant/60">IP_STACK_RETRYING_BACKOFF_2.4s</span>
        </div>
        <div className="flex items-center gap-3">
          <div className="text-right">
            <p className="font-label-caps text-label-caps text-on-surface-variant/70">OPERATOR_ID</p>
            <p className="font-telemetry-sm text-telemetry-sm font-bold text-on-surface">SEC_ALPHA_009</p>
          </div>
          <div className="flex h-10 w-10 items-center justify-center rounded border border-white/10 bg-surface-container-high">
            <span className="material-symbols-outlined text-[20px] text-on-surface-variant/60">person</span>
          </div>
        </div>
      </footer>
    </div>
  );
}
