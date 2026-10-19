import { useId, useState, type ReactNode } from "react";
import { ChevronDown, ChevronRight } from "lucide-react";
import { ReadDashboardSetting, SaveDashboardSetting, type DashboardMode } from "../lib/DashboardLayout";

type DashboardSectionProps = {
  SectionId: string;
  Title: string;
  Mode: DashboardMode;
  Detail?: ReactNode;
  Actions?: ReactNode;
  ClassName?: string;
  Hidden?: boolean;
  children: ReactNode;
};

export default function DashboardSection({ SectionId, Title, Mode, Detail, Actions, ClassName = "", Hidden, children }: DashboardSectionProps) {
  const ContentId = useId();
  const [Collapsed, SetCollapsed] = useState(() => ({
    Manual: ReadDashboardSetting(`Manual.${SectionId}.Collapsed`) === true,
    Ai: ReadDashboardSetting(`Ai.${SectionId}.Collapsed`) === true,
  }));
  const Minimized = Collapsed[Mode];
  const Toggle = () => {
    SetCollapsed(Previous => ({ ...Previous, [Mode]: !Minimized }));
    SaveDashboardSetting(`${Mode}.${SectionId}.Collapsed`, !Minimized);
  };
  return (
    <section className={`dashboard-section command-panel ${ClassName}`} data-section={SectionId} data-minimized={Minimized} hidden={Hidden}>
      <div className="panel-heading">
        <button className="section-toggle" type="button" onClick={Toggle} aria-expanded={!Minimized}
          aria-controls={ContentId} aria-label={`${Minimized ? "Restore" : "Minimize"} ${Title}`}>
          {Minimized ? <ChevronRight size={14} /> : <ChevronDown size={14} />}
          <span>{Title}</span>
        </button>
        {Detail && <small title={typeof Detail === "string" ? Detail : undefined}>{Detail}</small>}
        {Actions}
      </div>
      <div id={ContentId} className="dashboard-section-content" hidden={Minimized}>{children}</div>
    </section>
  );
}
