import { useState } from "react";
import "./css/App.css";
import AssetMap from "./pages/AssetMap";
import RescueResponse from "./pages/RescueResponse";
import LiveFeeds from "./pages/LiveFeeds";
import NotFound from "./pages/NotFound";
import { Page } from "./types";

type PageComponentProps = {
  onNavigate: (page: Page) => void;
  activePage: Page;
};

const PAGES: Record<Page, { label: string; component: React.ComponentType<PageComponentProps> }> = {
  "asset-map": { label: "Asset Map", component: AssetMap },
  "rescue-response": { label: "Dashboard", component: RescueResponse },
  "live-feeds": { label: "Live Feeds", component: LiveFeeds },
  // Rescue Logs isn't wired up yet - every nav link that points here (there's
  // one in each page's sidebar, see RescueResponse.tsx etc.) intentionally
  // lands on the 404 page instead of a half-built Analytics screen.
  "rescue-logs": { label: "Rescue Logs", component: NotFound },
  "not-found": { label: "Not Found", component: NotFound },
};

function App() {
  const [page, setPage] = useState<Page>("live-feeds");
  const ActivePage = PAGES[page].component;

  return (
    <div className="dark">
      <ActivePage onNavigate={setPage} activePage={page} />
    </div>
  );
}

export default App;