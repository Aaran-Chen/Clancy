import React from "react";
import ReactDOM from "react-dom/client";
import App from "./App";
import { InstallPasswordFetch } from "./lib/RemoteAccess";
import "@fontsource/play/latin-400.css";
import "@fontsource/play/latin-700.css";
import "./styles.css";
import "./components/ArcadeHud.css";
import "./components/SynthwaveTheme.css";
import "./components/ManualHud.css";

// Another device's requests to the robot's PC carry the arm password typed there (nothing changes on this PC).
InstallPasswordFetch();

ReactDOM.createRoot(document.getElementById("root") as HTMLElement).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>,
);
