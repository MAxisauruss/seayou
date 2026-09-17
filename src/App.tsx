import { useState } from "react";
import "./css/App.css";
import AssetMap from "./pages/AssetMap";
import RescueResponse from "./pages/RescueResponse";
import LiveFeeds from "./pages/LiveFeeds";
import Analytics from "./pages/Analytics";
import { Page } from "./types";

type PageComponentProps = {
  onNavigate: (page: Page) => void;
  activePage: Page;
};

const PAGES: Record<Page, { label: string; component: React.ComponentType<PageComponentProps> }> = {
  "asset-map": { label: "Asset Map", component: AssetMap },
  "rescue-response": { label: "Dashboard", component: RescueResponse },
  "live-feeds": { label: "Live Feeds", component: LiveFeeds },
  "rescue-logs": { label: "Rescue Logs", component: Analytics },
};

function App() {
  const [page, setPage] = useState<Page>("rescue-response");
  const ActivePage = PAGES[page].component;

  return (
    <div className="dark">
      <ActivePage onNavigate={setPage} activePage={page} />
    </div>
  );
}

export default App;