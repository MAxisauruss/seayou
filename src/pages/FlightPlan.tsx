// The flight planner's own screen.
//
// Deliberately spare: the other pages carry the full operations chrome,
// but this one is a working tool. Planning a search is fiddly enough
// without a drift-prediction mock-up and a swell readout competing for
// the same glance, and the map wants every pixel it can get.
import "../components/FlightPlanner.css";
import FlightPlanner from "../components/FlightPlanner";
import { Page } from "../types";

interface FlightPlanProps {
  onNavigate: (page: Page) => void;
  activePage: Page;
}

export default function FlightPlan({ onNavigate }: FlightPlanProps) {
  return (
    <div className="flight-plan-page">
      <header className="flight-plan-bar">
        <button className="fp-back" onClick={() => onNavigate("asset-map")}>
          <span className="material-symbols-outlined" data-icon="explore">
            explore
          </span>
          ASSET MAP
        </button>
        <h1>SeaYou &middot; FLIGHT PLAN</h1>
        <button className="fp-back" onClick={() => onNavigate("live-feeds")}>
          <span className="material-symbols-outlined" data-icon="videocam">
            videocam
          </span>
          FLIGHT CONTROLS
        </button>
      </header>
      <div className="flight-plan-body">
        <FlightPlanner />
      </div>
    </div>
  );
}
