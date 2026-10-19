import { useEffect, useRef, useState, type FormEvent } from "react";
import { KeyRound, X } from "lucide-react";
import type { AccessProblem } from "../lib/RemoteAccess";
import "./AccessCard.css";

// The arm password, asked once per device (lib/RemoteAccess.ts). Only another device's page shows it: the robot's PC
// refused this page, or it has no password stored yet. Unlocking opens the bridge; it never connects the arm.
type AccessCardProps = { Problem: AccessProblem | null; Host: string; OnSubmit: (Password: string) => void; OnRetry: () => void; OnClose: () => void };

export default function AccessCard({ Problem, Host, OnSubmit, OnRetry, OnClose }: AccessCardProps) {
  const [Password, SetPassword] = useState("");
  const InputRef = useRef<HTMLInputElement>(null);
  const CloseLatest = useRef(OnClose); CloseLatest.current = OnClose;
  const RemoteOff = Problem?.Code === "remote_off";
  const LockedOut = Problem?.Code === "locked_out";
  useEffect(() => {
    InputRef.current?.focus();
    const Escape = (Event: KeyboardEvent) => { if (Event.key === "Escape") CloseLatest.current(); };
    window.addEventListener("keydown", Escape);
    return () => window.removeEventListener("keydown", Escape);
  }, []);
  const Submit = (Event: FormEvent) => {
    Event.preventDefault();
    if (RemoteOff || LockedOut) { OnRetry(); return; }
    if (Password) OnSubmit(Password);
  };
  return (
    <div className="access-backdrop">
      <form className="access-card" role="dialog" aria-modal="true" aria-labelledby="access-title" aria-describedby="access-detail" onSubmit={Submit}>
        <div className="access-heading"><KeyRound size={18} /><h2 id="access-title">Arm password</h2>
          <button type="button" aria-label="Close" onClick={OnClose}><X size={18} /></button>
        </div>
        <p id="access-detail">Clancy is open on another device, not on the robot's PC ({Host}), so the arm needs the password set on that PC.
          This browser keeps it once you enter it.</p>
        {Problem && Problem.Code !== "password_required" && <p className="access-problem" role="alert">{Problem.Text}</p>}
        {RemoteOff ? <p className="access-help">On the robot's PC, run <code>python SetRemotePassword.py</code> in the
          <code>back end</code> folder (README: Using Clancy from another device), then check again.</p>
          : <label className="access-field"><span>Password</span>
            <input ref={InputRef} type="password" name="password" autoComplete="current-password" value={Password} disabled={LockedOut}
              onChange={Event => SetPassword(Event.currentTarget.value)} aria-invalid={Problem?.Code === "password_wrong" || undefined} />
          </label>}
        <div className="access-actions">
          <button type="submit" className="action-button primary" disabled={!RemoteOff && !LockedOut && !Password}>
            {RemoteOff || LockedOut ? "Check again" : "Unlock"}</button>
          <button type="button" className="action-button" onClick={OnClose}>Not now</button>
        </div>
        <small className="access-note">Unlocking never connects the arm or turns its torque on: from this device only the Connect button does.</small>
      </form>
    </div>
  );
}
