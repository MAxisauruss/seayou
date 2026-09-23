// The drone's camera as the detection model sees it: every object it finds
// framed, labelled with its class and confidence.
//
// The frames are drawn by groundstation/ml_view.py, a separate process on
// the ground-station laptop - the model needs PyTorch, which the ground
// station itself deliberately does not carry. This component only shows its
// annotated stream (/ml.mjpg) and reads its detections (/ml/status).
//
// Where that process lives: the same machine that served this page, port
// 8095. Override with ?ml=host:port (remembered), for when the page comes
// from somewhere else - the Pi, or a teammate's laptop.
import { useEffect, useState } from "react";

interface Detection {
  label: string;
  conf: number;
  box: [number, number, number, number];
}

interface MlStatus {
  source: string;
  source_ok: boolean;
  source_status: string;
  model: string;
  classes: string[];
  fps: number;
  infer_ms: number | null;
  detections: Detection[];
  alert: { label: string; conf: number } | null;
  alert_threshold: number;
  conf_threshold: number;
}

function resolveMlBase(): string {
  let host = `${window.location.hostname}:8095`;
  try {
    const fromUrl = new URLSearchParams(window.location.search).get("ml");
    if (fromUrl) {
      window.localStorage.setItem("mlHost", fromUrl);
      host = fromUrl;
    } else {
      host = window.localStorage.getItem("mlHost") || host;
    }
  } catch {
    /* private mode - the default is right for the normal case */
  }
  return /^https?:\/\//.test(host) ? host.replace(/\/+$/, "") : `http://${host}`;
}

const ML_BASE = resolveMlBase();

/** Class colours - the same ones ml_view.py draws its boxes in (CLASS_BGR). */
const CLASS_TONE: Record<string, string> = {
  Drowning: "bg-red-600 text-white",
  "Person out of water": "bg-amber-500 text-black",
  Swimming: "bg-sky-500 text-black",
};

export default function MlView({ variant }: { variant: "main" | "tile" }) {
  const [status, setStatus] = useState<MlStatus | null>(null);
  const [reachable, setReachable] = useState<boolean | null>(null);
  const [imgFailed, setImgFailed] = useState(false);
  // Changing the src forces the browser to reconnect the stream after the
  // process comes back, instead of staying on a dead connection.
  const [attempt, setAttempt] = useState(0);

  useEffect(() => {
    let alive = true;
    const poll = async () => {
      try {
        const r = await fetch(`${ML_BASE}/ml/status`, { cache: "no-store" });
        const d = (await r.json()) as MlStatus;
        if (!alive) return;
        setStatus(d);
        setReachable((was) => {
          if (was === false) {
            setImgFailed(false);
            setAttempt((a) => a + 1);
          }
          return true;
        });
      } catch {
        if (alive) setReachable(false);
      }
    };
    poll();
    const t = window.setInterval(poll, variant === "main" ? 1000 : 3000);
    return () => {
      alive = false;
      window.clearInterval(t);
    };
  }, [variant]);

  const offline = reachable === false || imgFailed;

  if (offline) {
    return (
      <div className="w-full h-full bg-[#050B14] flex flex-col items-center justify-center gap-2 p-4 text-center">
        <span className="material-symbols-outlined text-4xl text-white/25" data-icon="videocam_off">
          videocam_off
        </span>
        <span className="font-label-caps text-label-caps text-on-surface-variant">ML VIEW OFFLINE</span>
        {variant === "main" && (
          <span className="text-[11px] text-on-surface-variant max-w-sm leading-relaxed">
            The detection process is not running on {ML_BASE.replace(/^https?:\/\//, "")}.
            On the ground-station laptop, double-click:
            <code className="block mt-2 text-[10px] text-primary break-all">
              groundstation\start_ml_view.bat
            </code>
          </span>
        )}
      </div>
    );
  }

  const dets = status?.detections ?? [];

  return (
    <div className="relative w-full h-full bg-black">
      <img
        key={attempt}
        alt="Drone camera with the detection model's framing drawn on it"
        className={`w-full h-full object-contain ${variant === "tile" ? "opacity-80" : ""}`}
        src={`${ML_BASE}/ml.mjpg?a=${attempt}`}
        onError={() => setImgFailed(true)}
      />

      {variant === "main" && status && (
        <div className="absolute left-4 bottom-4 right-4 flex flex-col gap-2 pointer-events-none">
          {status.alert && (
            <div className="self-start flex items-center gap-2 bg-red-700/90 text-white px-3 py-1.5 rounded font-label-caps text-[11px] animate-pulse">
              <span className="material-symbols-outlined text-base" data-icon="warning">
                warning
              </span>
              DROWNING DETECTED - {Math.round(status.alert.conf * 100)}% CONFIDENCE
            </div>
          )}
          <div className="self-start glass-panel rounded-lg px-3 py-2 flex flex-col gap-1.5 max-w-full">
            <div className="flex items-center gap-3 font-label-caps text-[10px] text-on-surface-variant">
              <span className="text-tertiary">ML VIEW // YOLOv8 {status.model}</span>
              <span>
                {status.source_ok
                  ? `${status.fps.toFixed(1)} FPS - ${status.infer_ms ?? "-"} MS/FRAME`
                  : "WAITING FOR VIDEO"}
              </span>
            </div>
            {!status.source_ok ? (
              <span className="text-[11px] text-on-surface-variant">{status.source_status}</span>
            ) : dets.length === 0 ? (
              <span className="text-[11px] text-on-surface-variant">
                No objects in frame (showing {status.classes.join(", ")} above{" "}
                {Math.round(status.conf_threshold * 100)}%)
              </span>
            ) : (
              <div className="flex flex-wrap gap-1.5">
                {dets.map((d, i) => (
                  <span
                    key={i}
                    className={`px-2 py-0.5 rounded text-[10px] font-label-caps ${CLASS_TONE[d.label] ?? "bg-white/80 text-black"}`}
                  >
                    {d.label.toUpperCase()} {Math.round(d.conf * 100)}%
                  </span>
                ))}
              </div>
            )}
          </div>
        </div>
      )}
    </div>
  );
}
